#!/usr/bin/env bash
# -e: 未处理的命令失败时尽早终止脚本。
# -u: 使用未定义变量时报错，避免空变量写入错误路径。
# -o pipefail: 管道中前面的命令失败时，也把失败状态传递出来。
set -euo pipefail

readonly PROFILE="rk3588-rt"
readonly DEFAULT_ECAT_DEVICE_ID="fe1c0000.ethernet"
readonly EXPECTED_ECAT_DRIVER="rk_gmac-dwmac-ethercat"

# 启动参数分开保存，避免数组，同时让目标 RT CPU 布局一眼可见。
readonly EXPECTED_ISOLCPUS="isolcpus=domain,managed_irq,6-7"
readonly EXPECTED_RCU_NOCBS="rcu_nocbs=6-7"
readonly EXPECTED_IRQAFFINITY="irqaffinity=0-5"

PROC_ROOT="/proc"
SYS_ROOT="/sys"
RUN_ROOT="/run"
DEV_ROOT="/dev"
ETC_ROOT="/etc"
BOOT_UENV="/boot/uEnv/uEnv.txt"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
READY_FILE="${RUN_ROOT}/robot-rt-layout.ready"
ECAT_DEVICE_ID="${ROBOT_RT_ECAT_DEVICE_ID:-${DEFAULT_ECAT_DEVICE_ID}}"
NM_UNMANAGED_CONFIG="${ETC_ROOT}/NetworkManager/conf.d/99-ethercat-unmanaged.conf"

fail()
{
    echo "[robot-rt] ERROR: $*" >&2
    return 1
}

read_one_line()
{
    local path value

    path="$1"
    if [[ ! -r "${path}" ]]; then
        fail "cannot read ${path}"
        return 1
    fi

    # sysfs 和 procfs 中的值通常以换行结尾；读取第一行即可。
    IFS= read -r value < "${path}" || true
    printf '%s' "${value}"
}


normalize_cpu_list()
{
    local value

    value="$1"

    # 内核 CPU 列表可写成 6-7、6,7 或带 domain、managed_irq 等标记。
    # awk 在这里把它统一为逗号分隔的 CPU 编号，方便安全地做等价比较；
    printf '%s\n' "${value}" | awk -F ',' '
        {
            for (field_index = 1; field_index <= NF; ++field_index) {
                item = $field_index

                if (item == "" || item == "domain" ||
                    item == "managed_irq" || item == "nohz") {
                    continue
                }

                if (item ~ /^[0-9]+-[0-9]+$/) {
                    split(item, limits, "-")
                    first = limits[1] + 0
                    last = limits[2] + 0
                } else if (item ~ /^[0-9]+$/) {
                    first = item + 0
                    last = item + 0
                } else {
                    invalid = 1
                    exit 1
                }

                if (first > last) {
                    invalid = 1
                    exit 1
                }

                for (cpu = first; cpu <= last; ++cpu) {
                    present[cpu] = 1
                }
            }
        }
        END {
            if (invalid) {
                exit 1
            }

            output = ""
            for (cpu = 0; cpu < 1024; ++cpu) {
                if (present[cpu]) {
                    if (output != "") {
                        output = output ","
                    }
                    output = output cpu
                }
            }
            print output
        }
    '
}


cmdline_value()
{
    local key

    key="$1"

    # /proc/cmdline 是空白分隔的 key=value token。awk 不会发生 Bash 的
    # word splitting 或 globbing，并且找不到目标 key 时会返回失败状态。
    read_one_line "${PROC_ROOT}/cmdline" | awk -v key="${key}" '
        {
            for (field_index = 1; field_index <= NF; ++field_index) {
                token = $field_index
                if (token !~ "^" key "=") {
                    continue
                }

                sub("^[^=]*=", "", token)
                print token
                found = 1
                exit
            }
        }
        END {
            if (!found) {
                exit 1
            }
        }
    '
}


check_boot_layout()
{
    local cmdline isolcpus rcu_nocbs irqaffinity online isolated normalized

    if ! cmdline="$(read_one_line "${PROC_ROOT}/cmdline")"; then
        return 1
    fi

    # nohz_full 会改变周期 RT 基线的 tick 行为；本 profile 明确不允许它出现。
    if [[ " ${cmdline} " == *" nohz_full="* ]]; then
        fail "nohz_full must not be present in the periodic RT baseline"
        return 1
    fi

    # CPU6-7 留给 policy_cmd 与 ecat_rt，因此启动时必须隔离这两个 CPU。
    if ! isolcpus="$(cmdline_value isolcpus)"; then
        fail "isolcpus is missing"
        return 1
    fi
    if [[ ",${isolcpus}," != *,domain,* ]]; then
        fail "isolcpus must contain domain,managed_irq"
        return 1
    fi
    if [[ ",${isolcpus}," != *,managed_irq,* ]]; then
        fail "isolcpus must contain domain,managed_irq"
        return 1
    fi
    if ! normalized="$(normalize_cpu_list "${isolcpus}")"; then
        fail "invalid CPU list: ${isolcpus}"
        return 1
    fi
    if [[ "${normalized}" != "6,7" ]]; then
        fail "isolcpus CPU set must be exactly 6-7"
        return 1
    fi

    if ! rcu_nocbs="$(cmdline_value rcu_nocbs)"; then
        fail "rcu_nocbs is missing"
        return 1
    fi
    if ! normalized="$(normalize_cpu_list "${rcu_nocbs}")"; then
        fail "invalid CPU list: ${rcu_nocbs}"
        return 1
    fi
    if [[ "${normalized}" != "6,7" ]]; then
        fail "rcu_nocbs CPU set must be exactly 6-7"
        return 1
    fi

    if ! irqaffinity="$(cmdline_value irqaffinity)"; then
        fail "irqaffinity is missing"
        return 1
    fi
    if ! normalized="$(normalize_cpu_list "${irqaffinity}")"; then
        fail "invalid CPU list: ${irqaffinity}"
        return 1
    fi
    if [[ "${normalized}" != "0,1,2,3,4,5" ]]; then
        fail "irqaffinity CPU set must be exactly 0-5"
        return 1
    fi

    if ! online="$(read_one_line "${SYS_ROOT}/devices/system/cpu/online")"; then
        return 1
    fi
    if ! normalized="$(normalize_cpu_list "${online}")"; then
        fail "invalid CPU list: ${online}"
        return 1
    fi
    if [[ "${normalized}" != "0,1,2,3,4,5,6,7" ]]; then
        fail "RK3588 CPUs 0-7 must all be online"
        return 1
    fi

    if ! isolated="$(read_one_line "${SYS_ROOT}/devices/system/cpu/isolated")"; then
        return 1
    fi
    if ! normalized="$(normalize_cpu_list "${isolated}")"; then
        fail "invalid CPU list: ${isolated}"
        return 1
    fi
    if [[ "${normalized}" != "6,7" ]]; then
        fail "effective isolated CPU set must be exactly 6-7"
        return 1
    fi
}


interface_irqs()
{
    local interface_name

    interface_name="$1"
    if [[ ! -r "${PROC_ROOT}/interrupts" ]]; then
        fail "cannot read ${PROC_ROOT}/interrupts"
        return 1
    fi

    # /proc/interrupts 列出每个 IRQ 的处理设备；一个接口可能有多个 IRQ。
    awk -v interface_name="${interface_name}" '
        {
            irq = $1
            sub(/:$/, "", irq)
            if (irq !~ /^[0-9]+$/) {
                next
            }

            for (field_index = 2; field_index <= NF; ++field_index) {
                if ($field_index == interface_name) {
                    print irq
                    break
                }
            }
        }
    ' "${PROC_ROOT}/interrupts"
}


set_irq_affinity()
{
    local interface_name cpu irq_list irq path

    interface_name="$1"
    cpu="$2"
    if ! irq_list="$(interface_irqs "${interface_name}")"; then
        return 1
    fi
    if [[ -z "${irq_list}" ]]; then
        fail "no IRQ found for ${interface_name}"
        return 1
    fi

    # interface_irqs 的 awk 只输出十进制 IRQ 编号和换行，因此此处有意按空白
    # 分割列表是安全的：不会把包含空格的名称拆开，也不会触发 globbing。
    # shellcheck disable=SC2086
    for irq in ${irq_list}; do
        path="${PROC_ROOT}/irq/${irq}/smp_affinity_list"
        if [[ ! -w "${path}" ]]; then
            fail "cannot write ${path}"
            return 1
        fi

        # smp_affinity_list 是请求的 IRQ CPU 列表。CAN 放在 CPU2，避免进入 RT CPU。
        printf '%s\n' "${cpu}" > "${path}"
    done
}


check_irq_affinity()
{
    local interface_name expected_cpu irq_list irq actual normalized

    interface_name="$1"
    expected_cpu="$2"
    if ! irq_list="$(interface_irqs "${interface_name}")"; then
        return 1
    fi
    if [[ -z "${irq_list}" ]]; then
        fail "no IRQ found for ${interface_name}"
        return 1
    fi

    # interface_irqs 只生成数字 IRQ，见 set_irq_affinity 中的安全说明。
    # shellcheck disable=SC2086
    for irq in ${irq_list}; do
        # effective_affinity_list 是内核实际生效的亲和性，必须回读而非只相信写入。
        if ! actual="$(read_one_line \
            "${PROC_ROOT}/irq/${irq}/effective_affinity_list")"; then
            return 1
        fi
        if ! normalized="$(normalize_cpu_list "${actual}")"; then
            fail "invalid CPU list: ${actual}"
            return 1
        fi
        if [[ "${normalized}" != "${expected_cpu}" ]]; then
            fail "${interface_name} IRQ ${irq} effective affinity is ${actual}, expected ${expected_cpu}"
            return 1
        fi
    done
}


check_governors()
{
    local policy actual

    for policy in 0 4 6; do
        if ! actual="$(read_one_line \
            "${SYS_ROOT}/devices/system/cpu/cpufreq/policy${policy}/scaling_governor")"; then
            return 1
        fi
        if [[ "${actual}" != "performance" ]]; then
            fail "cpufreq policy${policy} governor is ${actual}, expected performance"
            return 1
        fi
    done
}


set_governors()
{
    local policy path

    for policy in 0 4 6; do
        path="${SYS_ROOT}/devices/system/cpu/cpufreq/policy${policy}/scaling_governor"
        if [[ ! -w "${path}" ]]; then
            fail "cannot write ${path}"
            return 1
        fi

        printf '%s\n' performance > "${path}"
    done
}


check_workqueue()
{
    local mask compact

    # unbound workqueue 不能进入 CPU6-7，否则后台内核工作会干扰 RT 线程。
    if ! mask="$(read_one_line "${SYS_ROOT}/devices/virtual/workqueue/cpumask")"; then
        return 1
    fi
    compact="$(printf '%s' "${mask}" | tr -d ',')"
    if [[ ! "${compact}" =~ ^[0-9a-fA-F]+$ ]]; then
        fail "invalid workqueue cpumask: ${mask}"
        return 1
    fi

    # Bash 的 16# 表示十六进制。先验证只含十六进制字符，才可安全比较；
    # 这样内核写出的补零或逗号分组掩码仍能与 CPU0-5 的 3f 等价。
    if (( 16#${compact} != 16#3f )); then
        fail "unbound workqueue cpumask is ${mask}, expected CPU0-5 (3f)"
        return 1
    fi
}


set_workqueue()
{
    local path

    path="${SYS_ROOT}/devices/virtual/workqueue/cpumask"
    if [[ ! -w "${path}" ]]; then
        fail "cannot write ${path}"
        return 1
    fi

    printf '%s\n' 3f > "${path}"
}


find_ethercat_netdev()
{
    local net_path device_path resolved resolved_name netdev_name candidate

    candidate=""

    # 安装前，fe1c0000.ethernet 可能仍以普通 Linux netdev 的形式出现。
    # 只能按固定的物理 platform device 反查，不能把 eth0/eth1 名字写死。
    #
    # 重要：EtherCAT 专用驱动加载后，这个物理口在当前板卡上不再注册
    # /sys/class/net/<iface>。因此“找不到 netdev”在运行阶段是正常状态，
    # 本函数只负责“如果存在就找出来”，找不到时静默返回 1。
    for net_path in "${SYS_ROOT}"/class/net/*; do
        if [[ ! -e "${net_path}" ]]; then
            continue
        fi

        device_path="${net_path}/device"
        if [[ ! -e "${device_path}" && ! -L "${device_path}" ]]; then
            continue
        fi
        if ! resolved="$(readlink -f -- "${device_path}")"; then
            continue
        fi

        resolved_name="$(basename -- "${resolved}")"
        if [[ "${resolved_name}" != "${ECAT_DEVICE_ID}" ]]; then
            continue
        fi

        netdev_name="$(basename -- "${net_path}")"
        if [[ -n "${candidate}" ]]; then
            fail "multiple netdevs map to EtherCAT device ${ECAT_DEVICE_ID}: ${candidate}, ${netdev_name}"
            return 1
        fi
        candidate="${netdev_name}"
    done

    if [[ -z "${candidate}" ]]; then
        return 1
    fi

    printf '%s' "${candidate}"
}


validate_mac()
{
    local mac description

    mac="$1"
    description="$2"

    # MAC 必须严格满足 xx:xx:xx:xx:xx:xx。
    if [[ ! "${mac}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]]; then
        fail "invalid MAC for ${description}: ${mac}"
        return 1
    fi
}


netdev_permanent_mac()
{
    local interface_name address_path assign_type_path mac assign_type

    interface_name="$1"
    address_path="${SYS_ROOT}/class/net/${interface_name}/address"
    assign_type_path="${SYS_ROOT}/class/net/${interface_name}/addr_assign_type"

    if ! mac="$(read_one_line "${address_path}" | tr '[:upper:]' '[:lower:]')"; then
        return 1
    fi
    if ! validate_mac "${mac}" "EtherCAT install netdev ${interface_name}"; then
        return 1
    fi

    # addr_assign_type=0 表示永久硬件 MAC，而不是内核随机生成的地址。
    if [[ -r "${assign_type_path}" ]]; then
        if ! assign_type="$(read_one_line "${assign_type_path}")"; then
            return 1
        fi
        if [[ "${assign_type}" != "0" ]]; then
            fail "EtherCAT install netdev ${interface_name} has non-permanent MAC type ${assign_type}"
            return 1
        fi
    fi

    printf '%s' "${mac}"
}


ethercat_platform_driver()
{
    local platform_path driver_link resolved

    platform_path="${SYS_ROOT}/bus/platform/devices/${ECAT_DEVICE_ID}"
    driver_link="${platform_path}/driver"

    if [[ ! -e "${platform_path}" ]]; then
        return 1
    fi
    if [[ ! -e "${driver_link}" && ! -L "${driver_link}" ]]; then
        return 1
    fi
    if ! resolved="$(readlink -f -- "${driver_link}")"; then
        return 1
    fi

    basename -- "${resolved}"
}


loaded_master_mac()
{
    local path mac

    path="${SYS_ROOT}/module/ec_master/parameters/main_devices"
    if [[ ! -r "${path}" ]]; then
        return 1
    fi
    if ! mac="$(read_one_line "${path}" | tr '[:upper:]' '[:lower:]')"; then
        return 1
    fi
    if ! validate_mac "${mac}" "loaded ec_master main_devices"; then
        return 1
    fi

    printf '%s' "${mac}"
}


resolve_ethercat_mac_for_install()
{
    local interface_name mac driver loaded

    # 情况 A：第一次安装、专用 EtherCAT 驱动尚未接管 fe1c0000。
    # 此时可以从物理 platform device 对应的普通 netdev 读取永久 MAC。
    if interface_name="$(find_ethercat_netdev)"; then
        if ! mac="$(netdev_permanent_mac "${interface_name}")"; then
            return 1
        fi
        printf '%s' "${mac}"
        return 0
    fi

    # 情况 B：已经重启过，fe1c0000 已由 rk_gmac-dwmac-ethercat 接管。
    # 当前驱动不会再暴露普通 netdev；此时以 ec_master 已加载的 main_devices
    # 作为正在运行的主站 MAC。这样 install 可以在运行态重复执行，不再错误地
    # 要求 /sys/class/net 中必须存在 EtherCAT 接口。
    if ! driver="$(ethercat_platform_driver)"; then
        fail "cannot identify driver for EtherCAT device ${ECAT_DEVICE_ID}"
        return 1
    fi
    if [[ "${driver}" != "${EXPECTED_ECAT_DRIVER}" ]]; then
        fail "EtherCAT device ${ECAT_DEVICE_ID} is bound to ${driver}, expected ${EXPECTED_ECAT_DRIVER}"
        return 1
    fi
    if ! loaded="$(loaded_master_mac)"; then
        fail "cannot determine EtherCAT MAC: no install-time netdev and ec_master main_devices is unavailable"
        return 1
    fi

    printf '%s' "${loaded}"
}


networkmanager_unmanages_mac_in_file()
{
    local mac config

    mac="$1"
    config="$2"
    if [[ ! -r "${config}" ]]; then
        return 1
    fi

    awk -v target="mac:${mac}" '
        BEGIN { found = 0; section = "" }
        /^[[:space:]]*[#;]/ { next }
        {
            line = tolower($0)
            gsub(/[[:space:]]/, "", line)
            if (line ~ /^\[[^]]+\]$/) {
                section = line
                next
            }
            if (section != "[keyfile]") {
                next
            }
            if (line !~ /^unmanaged-devices=/) {
                next
            }

            value = substr(line, index(line, "=") + 1)
            count = split(value, entries, ";")
            for (entry_index = 1; entry_index <= count; ++entry_index) {
                if (entries[entry_index] == target) {
                    found = 1
                }
            }
        }
        END { exit(found ? 0 : 1) }
    ' "${config}"
}


networkmanager_unmanages_mac()
{
    local mac

    mac="$1"
    networkmanager_unmanages_mac_in_file "${mac}" "${NM_UNMANAGED_CONFIG}"
}


check_networkmanager_config()
{
    local mac

    mac="$1"

    # 该配置仍然保留：安装前如果 fe1c0000 暂时还是普通 netdev，NetworkManager
    # 不应接管它。重启后专用 EtherCAT 驱动不再暴露 netdev，此配置则只是保护性冗余。
    # 关键变化是：它只影响 robot-rt 自己是否 ready，不再作为 NetworkManager 的
    # ExecStartPre，因此检查失败绝不能再把整台机器的管理网络一起阻断。
    if ! networkmanager_unmanages_mac "${mac}"; then
        fail "${NM_UNMANAGED_CONFIG} does not mark EtherCAT MAC ${mac} unmanaged"
        return 1
    fi
}


configured_master_mac_from_file()
{
    local config

    # ec_master main_devices 指定 IgH EtherCAT 主站绑定的网卡 MAC。
    # 它必须与上面按物理设备找到的永久 MAC 相同。
    config="$1"
    if [[ ! -r "${config}" ]]; then
        fail "cannot read ${config}"
        return 1
    fi

    awk '
        $1 == "options" && $2 == "ec_master" {
            for (field_index = 3; field_index <= NF; ++field_index) {
                if ($field_index ~ /^main_devices=/) {
                    sub(/^main_devices=/, "", $field_index)
                    print tolower($field_index)
                    exit
                }
            }
        }
    ' "${config}"
}


configured_master_mac()
{
    configured_master_mac_from_file "${ETC_ROOT}/modprobe.d/ethercat.conf"
}


check_ethercat_runtime()
{
    local driver configured loaded

    # 运行阶段不再寻找 Linux netdev。当前板卡的 fe1c0000.ethernet 在
    # rk_gmac-dwmac-ethercat 接管后不会出现在 /sys/class/net 中。
    if ! driver="$(ethercat_platform_driver)"; then
        fail "EtherCAT device ${ECAT_DEVICE_ID} has no bound platform driver"
        return 1
    fi
    if [[ "${driver}" != "${EXPECTED_ECAT_DRIVER}" ]]; then
        fail "EtherCAT device ${ECAT_DEVICE_ID} is bound to ${driver}, expected ${EXPECTED_ECAT_DRIVER}"
        return 1
    fi

    if ! configured="$(configured_master_mac)"; then
        return 1
    fi
    if [[ -z "${configured}" ]]; then
        fail "ec_master main_devices is not configured"
        return 1
    fi
    if ! validate_mac "${configured}" "configured ec_master main_devices"; then
        return 1
    fi

    if ! loaded="$(loaded_master_mac)"; then
        fail "loaded ec_master main_devices is unavailable"
        return 1
    fi
    if [[ "${loaded}" != "${configured}" ]]; then
        fail "loaded ec_master main_devices=${loaded} does not match configured value ${configured}; reboot after install"
        return 1
    fi

    # 保留 unmanaged-by-MAC 配置作为保护，但它的失败只会让 robot-rt 不 ready，
    # 不会再阻止 NetworkManager 自己启动。
    check_networkmanager_config "${loaded}"

    # /dev/EtherCAT0 是 IgH 主站给用户态暴露的字符设备。
    if [[ ! -e "${DEV_ROOT}/EtherCAT0" ]]; then
        fail "${DEV_ROOT}/EtherCAT0 is unavailable"
        return 1
    fi
}


check_irqbalance_guard()
{
    local dropin

    dropin="${ETC_ROOT}/systemd/system/irqbalance.service.d/robot-rt.conf"
    if [[ "${ETC_ROOT}" != "/etc" ]]; then
        return 0
    fi
    if ! command -v systemctl >/dev/null 2>&1; then
        return 0
    fi
    if ! systemctl is-active --quiet irqbalance.service; then
        return 0
    fi

    # irqbalance 会动态迁移普通 IRQ，必须显式禁用它对 CPU6-7 的使用。
    if [[ ! -r "${dropin}" ]]; then
        fail "irqbalance is active without the RT CPU ban drop-in"
        return 1
    fi
    if ! grep -q 'IRQBALANCE_BANNED_CPULIST=6-7' "${dropin}"; then
        fail "irqbalance drop-in does not ban CPUs 6-7"
        return 1
    fi
}


check_ready_marker()
{
    local boot_id

    if ! boot_id="$(read_one_line "${PROC_ROOT}/sys/kernel/random/boot_id")"; then
        return 1
    fi
    # /run 通常在重启时清空；仍需比对 boot_id，不能把旧 marker 当成本次 apply 成功。
    if [[ ! -r "${READY_FILE}" ]]; then
        fail "ready marker is missing: ${READY_FILE}"
        return 1
    fi
    if ! grep -qx "profile=${PROFILE}" "${READY_FILE}"; then
        fail "ready marker profile is incorrect"
        return 1
    fi
    if ! grep -qx "boot_id=${boot_id}" "${READY_FILE}"; then
        fail "ready marker belongs to another boot"
        return 1
    fi
}


check_runtime_without_marker()
{
    check_boot_layout
    check_workqueue
    check_governors

    # CAN0 在当前系统中有标准 Linux IRQ，可安全固定到 CPU2。
    check_irq_affinity can0 2

    check_irqbalance_guard
    check_ethercat_runtime
}


report_ethercat_context()
{
    local driver configured loaded interface_name run_on_cpu

    driver="unavailable"
    if ethercat_platform_driver >/dev/null 2>&1; then
        driver="$(ethercat_platform_driver)"
    fi

    configured="unavailable"
    if configured_master_mac >/dev/null 2>&1; then
        configured="$(configured_master_mac)"
    fi

    loaded="unavailable"
    if loaded_master_mac >/dev/null 2>&1; then
        loaded="$(loaded_master_mac)"
    fi

    echo "[robot-rt] EtherCAT physical device: ${ECAT_DEVICE_ID}"
    echo "[robot-rt] EtherCAT platform driver: ${driver}"
    echo "[robot-rt] EtherCAT configured main_devices: ${configured}"
    echo "[robot-rt] EtherCAT loaded main_devices: ${loaded}"

    # 有些驱动版本可能会额外暴露 netdev；当前 rk_gmac-dwmac-ethercat 运行态没有。
    if interface_name="$(find_ethercat_netdev)"; then
        echo "[robot-rt] EtherCAT Linux netdev currently exposed as: ${interface_name}"
    else
        echo "[robot-rt] EtherCAT Linux netdev: not exposed by the current dedicated driver"
    fi

    echo "[robot-rt] EtherCAT IRQ affinity: not modified; current profile does not assume a stable Linux IRQ mapping"

    if [[ -r "${SYS_ROOT}/module/ec_master/parameters/run_on_cpu" ]]; then
        if ! run_on_cpu="$(read_one_line "${SYS_ROOT}/module/ec_master/parameters/run_on_cpu")"; then
            return 1
        fi
        echo "[robot-rt] IgH run_on_cpu: ${run_on_cpu}"
    else
        echo "[robot-rt] IgH run_on_cpu is not exposed by this module build"
    fi
}


check_layout()
{
    check_runtime_without_marker
    check_ready_marker
    report_ethercat_context
    echo "[robot-rt] ${PROFILE} layout check passed"
}


apply_layout()
{
    local boot_id

    if [[ "${PROC_ROOT}" == "/proc" ]]; then
        if [[ "$(id -u)" -ne 0 ]]; then
            fail "apply must run as root"
            return 1
        fi
    fi

    check_boot_layout
    check_ethercat_runtime

    # 先删除旧 marker：任何中途失败都不能被误判为本次布局已就绪。
    rm -f -- "${READY_FILE}"

    set_workqueue
    set_governors

    # CAN0 有标准 Linux IRQ，因此继续固定到 CPU2。
    set_irq_affinity can0 2

    # 不再设置所谓“EtherCAT IRQ -> CPU3”。当前专用驱动运行时既没有对应
    # /sys/class/net netdev，也没有在 /proc/interrupts 中暴露稳定的 EtherCAT IRQ。
    # 如果以后驱动实现发生变化，应基于新的实际 IRQ/线程模型单独设计，而不是猜测。
    check_runtime_without_marker

    if ! mkdir -p -- "${RUN_ROOT}"; then
        fail "cannot create ${RUN_ROOT}"
        return 1
    fi
    if ! boot_id="$(read_one_line "${PROC_ROOT}/sys/kernel/random/boot_id")"; then
        return 1
    fi
    {
        echo "profile=${PROFILE}"
        echo "boot_id=${boot_id}"
    } > "${READY_FILE}"

    check_ready_marker
    report_ethercat_context
    echo "[robot-rt] ${PROFILE} layout applied and verified"
}


rewrite_uenv()
{
    local source resolved current inner new_args replacement temp original_mode

    source="$1"
    temp=""
    (
        # trap 是刻意保留的 Bash 特性：set -e 或收到信号时也删除未替换的临时文件。
        cleanup_temp()
        {
            if [[ -n "${temp}" && -e "${temp}" ]]; then
                rm -f -- "${temp}"
            fi
        }
        trap cleanup_temp EXIT
        trap 'cleanup_temp; exit 1' HUP INT TERM

        if ! resolved="$(readlink -f -- "${source}")"; then
            fail "active uEnv file not found: ${source}"
            exit 1
        fi
        if [[ ! -f "${resolved}" ]]; then
            fail "active uEnv file not found: ${source}"
            exit 1
        fi
        if ! current="$(grep -m1 '^cmdline="' "${resolved}")"; then
            fail "cmdline entry not found in ${resolved}"
            exit 1
        fi
        if ! printf '%s\n' "${current}" | grep -Eq '^cmdline="[^"]*"$'; then
            fail "cmdline entry is invalid in ${resolved}"
            exit 1
        fi

        inner="$(printf '%s\n' "${current}" | sed -e 's/^cmdline="//' -e 's/"$//')"
        # 内核命令行按空白分隔。awk 删除旧 RT token 后追加唯一的正式基线。
        new_args="$(printf '%s\n' "${inner}" | awk \
            -v isolcpus="${EXPECTED_ISOLCPUS}" \
            -v rcu_nocbs="${EXPECTED_RCU_NOCBS}" \
            -v irqaffinity="${EXPECTED_IRQAFFINITY}" '
                {
                    for (field_index = 1; field_index <= NF; ++field_index) {
                        token = $field_index
                        if (token ~ /^(isolcpus|rcu_nocbs|irqaffinity|nohz_full)=/) {
                            continue
                        }
                        if (output != "") {
                            output = output " "
                        }
                        output = output token
                    }
                }
                END {
                    if (output != "") {
                        output = output " "
                    }
                    print output isolcpus " " rcu_nocbs " " irqaffinity
                }
            ')"
        replacement="cmdline=\"${new_args}\""

        if [[ ! -e "${resolved}.pre-robot-rt" ]]; then
            if ! cp -a -- "${resolved}" "${resolved}.pre-robot-rt"; then
                fail "cannot back up ${resolved}"
                exit 1
            fi
        fi

        # 临时文件与目标文件放在同一目录，mv 才能作为同一文件系统内的原子替换。
        if ! temp="$(mktemp "${resolved}.tmp.XXXXXX")"; then
            fail "cannot create temporary uEnv file beside ${resolved}"
            exit 1
        fi
        if ! awk -v replacement="${replacement}" '
            BEGIN { changed = 0 }
            /^cmdline="/ { print replacement; changed = 1; next }
            { print }
            END { if (!changed) exit 42 }
        ' "${resolved}" > "${temp}"; then
            fail "cannot rewrite ${resolved}"
            exit 1
        fi
        if ! grep -Fxq "${replacement}" "${temp}"; then
            fail "rewritten cmdline is missing from temporary uEnv file"
            exit 1
        fi
        if ! original_mode="$(stat -c '%a' "${resolved}")"; then
            fail "cannot read mode of ${resolved}"
            exit 1
        fi
        if ! chmod "${original_mode}" "${temp}"; then
            fail "cannot preserve mode of ${resolved}"
            exit 1
        fi
        if ! mv -f -- "${temp}" "${resolved}"; then
            fail "cannot atomically replace ${resolved}"
            exit 1
        fi
        temp=""
    )
}


rewrite_ethercat_config()
{
    local mac config input temp original_mode configured

    mac="$1"
    config="${ETC_ROOT}/modprobe.d/ethercat.conf"
    temp=""
    (
        # 保留 trap，避免写配置失败时留下可被误用的临时文件。
        cleanup_temp()
        {
            if [[ -n "${temp}" && -e "${temp}" ]]; then
                rm -f -- "${temp}"
            fi
        }
        trap cleanup_temp EXIT
        trap 'cleanup_temp; exit 1' HUP INT TERM

        if ! mkdir -p -- "$(dirname -- "${config}")"; then
            fail "cannot create directory for ${config}"
            exit 1
        fi

        input="/dev/null"
        original_mode="0644"
        if [[ -e "${config}" ]]; then
            input="${config}"
            if ! original_mode="$(stat -c '%a' "${config}")"; then
                fail "cannot read mode of ${config}"
                exit 1
            fi
            if [[ ! -e "${config}.pre-robot-rt" ]]; then
                if ! cp -a -- "${config}" "${config}.pre-robot-rt"; then
                    fail "cannot back up ${config}"
                    exit 1
                fi
            fi
        fi

        if ! temp="$(mktemp "${config}.tmp.XXXXXX")"; then
            fail "cannot create temporary EtherCAT config beside ${config}"
            exit 1
        fi
        if ! awk -v mac="${mac}" '
            BEGIN { found_options = 0 }
            $1 == "options" && $2 == "ec_master" {
                found_options = 1
                found_main = 0
                for (field_index = 3; field_index <= NF; ++field_index) {
                    if ($field_index ~ /^main_devices=/) {
                        $field_index = "main_devices=" mac
                        found_main = 1
                    }
                }
                if (!found_main) {
                    $0 = $0 " main_devices=" mac
                }
            }
            { print }
            END {
                if (!found_options) {
                    print "options ec_master main_devices=" mac
                }
            }
        ' "${input}" > "${temp}"; then
            fail "cannot rewrite ${config}"
            exit 1
        fi
        if ! configured="$(configured_master_mac_from_file "${temp}")"; then
            exit 1
        fi
        if [[ "${configured}" != "${mac}" ]]; then
            fail "temporary EtherCAT config does not set ec_master main_devices=${mac}"
            exit 1
        fi
        if ! chmod "${original_mode}" "${temp}"; then
            fail "cannot set mode on temporary EtherCAT config"
            exit 1
        fi
        if ! mv -f -- "${temp}" "${config}"; then
            fail "cannot atomically replace ${config}"
            exit 1
        fi
        temp=""
    )
}


rewrite_networkmanager_config()
{
    local mac temp original_mode

    mac="$1"
    temp=""
    (
        # 保留 trap，避免写配置失败时留下可被误用的临时文件。
        cleanup_temp()
        {
            if [[ -n "${temp}" && -e "${temp}" ]]; then
                rm -f -- "${temp}"
            fi
        }
        trap cleanup_temp EXIT
        trap 'cleanup_temp; exit 1' HUP INT TERM

        if ! mkdir -p -- "$(dirname -- "${NM_UNMANAGED_CONFIG}")"; then
            fail "cannot create directory for ${NM_UNMANAGED_CONFIG}"
            exit 1
        fi

        original_mode="0644"
        if [[ -e "${NM_UNMANAGED_CONFIG}" ]]; then
            if ! original_mode="$(stat -c '%a' "${NM_UNMANAGED_CONFIG}")"; then
                fail "cannot read mode of ${NM_UNMANAGED_CONFIG}"
                exit 1
            fi
            if [[ ! -e "${NM_UNMANAGED_CONFIG}.pre-robot-rt" ]]; then
                if ! cp -a -- "${NM_UNMANAGED_CONFIG}" \
                    "${NM_UNMANAGED_CONFIG}.pre-robot-rt"; then
                    fail "cannot back up ${NM_UNMANAGED_CONFIG}"
                    exit 1
                fi
            fi
        fi

        if ! temp="$(mktemp "${NM_UNMANAGED_CONFIG}.tmp.XXXXXX")"; then
            fail "cannot create temporary NetworkManager config beside ${NM_UNMANAGED_CONFIG}"
            exit 1
        fi
        {
            echo "[keyfile]"
            echo "unmanaged-devices=mac:${mac}"
        } > "${temp}"
        if ! networkmanager_unmanages_mac_in_file "${mac}" "${temp}"; then
            fail "temporary NetworkManager config does not mark MAC ${mac} unmanaged"
            exit 1
        fi
        if ! chmod "${original_mode}" "${temp}"; then
            fail "cannot set mode on temporary NetworkManager config"
            exit 1
        fi
        if ! mv -f -- "${temp}" "${NM_UNMANAGED_CONFIG}"; then
            fail "cannot atomically replace ${NM_UNMANAGED_CONFIG}"
            exit 1
        fi
        temp=""
    )
}


install_layout()
{
    local mac interface_name

    if [[ "$(id -u)" -ne 0 ]]; then
        fail "install must run as root"
        return 1
    fi

    # 第一次安装时，如果 fe1c0000 仍以普通 netdev 暴露，则从该 netdev
    # 读取永久 MAC；如果专用 EtherCAT 驱动已经接管，则从 ec_master
    # 当前加载的 main_devices 读取 MAC。
    if ! mac="$(resolve_ethercat_mac_for_install)"; then
        return 1
    fi

    # NetworkManager 只通过独立配置文件忽略 EtherCAT MAC。
    # 不给 NetworkManager.service 增加 ExecStartPre，也不让 RT 检查决定
    # NetworkManager 能否启动，从而保证管理网口/Wi-Fi 始终可恢复。
    rewrite_networkmanager_config "${mac}"
    check_networkmanager_config "${mac}"

    rewrite_ethercat_config "${mac}"
    rewrite_uenv "${BOOT_UENV}"

    if ! install -D -m 0755 "${SCRIPT_DIR}/robot-rt-setup.sh" \
        "/usr/local/sbin/robot-rt-setup"; then
        fail "cannot install robot-rt-setup"
        return 1
    fi

    if ! install -D -m 0644 "${SCRIPT_DIR}/robot-rt-setup.service" \
        "/etc/systemd/system/robot-rt-setup.service"; then
        fail "cannot install robot-rt-setup.service"
        return 1
    fi

    if ! mkdir -p -- "/etc/systemd/system/irqbalance.service.d"; then
        fail "cannot create irqbalance drop-in directory"
        return 1
    fi

    if ! install -m 0644 "${SCRIPT_DIR}/irqbalance-robot-rt.conf" \
        "/etc/systemd/system/irqbalance.service.d/robot-rt.conf"; then
        fail "cannot install irqbalance RT drop-in"
        return 1
    fi

    systemctl daemon-reload
    systemctl enable robot-rt-setup.service

    interface_name=""
    if interface_name="$(find_ethercat_netdev)"; then
        echo "[robot-rt] EtherCAT install discovery: ${ECAT_DEVICE_ID} -> ${interface_name} -> ${mac}"
    else
        echo "[robot-rt] EtherCAT runtime discovery: ${ECAT_DEVICE_ID} -> ${EXPECTED_ECAT_DRIVER} -> ${mac}"
    fi

    echo "[robot-rt] installed ${PROFILE}; reboot is required before apply/check can pass"
    echo "[robot-rt] NetworkManager is independent from EtherCAT runtime checks"
    echo "[robot-rt] CAN0 IRQ will be pinned to CPU2"
    echo "[robot-rt] EtherCAT IRQ affinity is not modified because this driver exposes no stable Linux IRQ mapping"
}


usage()
{
    echo "Usage: $0 {check|install}" >&2
    exit 2
}


# 正式脚本命令入口。
command_name="${1:-}"


case "${command_name}" in
    check)
        check_layout
        ;;
    install)
        install_layout
        ;;
    __apply)
        # __apply 只供 systemd service 调用，避免用户误把 apply 当成普通安装命令。
        if [[ "${ROBOT_RT_INTERNAL_COMMAND:-0}" != "1" ]]; then
            fail "__apply is reserved for the installed systemd service"
            exit 1
        fi
        apply_layout
        ;;
    *)
        usage
        ;;
esac
