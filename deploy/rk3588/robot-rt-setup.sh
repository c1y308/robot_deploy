#!/usr/bin/env bash
set -euo pipefail

readonly PROFILE="rk3588-v1"
readonly DEFAULT_ECAT_DEVICE_ID="fe1c0000.ethernet"
readonly EXPECTED_BOOT_ARGS=(
    "isolcpus=domain,managed_irq,6-7"
    "rcu_nocbs=6-7"
    "irqaffinity=0-5"
)

PROC_ROOT="${ROBOT_RT_PROC_ROOT:-/proc}"
SYS_ROOT="${ROBOT_RT_SYS_ROOT:-/sys}"
RUN_ROOT="${ROBOT_RT_RUN_ROOT:-/run}"
DEV_ROOT="${ROBOT_RT_DEV_ROOT:-/dev}"
ETC_ROOT="${ROBOT_RT_ETC_ROOT:-/etc}"
BOOT_UENV="${ROBOT_RT_BOOT_UENV:-/boot/uEnv/uEnv.txt}"
INSTALL_ROOT="${ROBOT_RT_INSTALL_ROOT:-}"
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
    local path="$1"
    [[ -r "${path}" ]] || fail "cannot read ${path}"
    local value
    IFS= read -r value < "${path}" || true
    printf '%s' "${value}"
}

normalize_cpu_list()
{
    local value="$1"
    local item first last cpu
    local -A present=()
    IFS=',' read -r -a items <<< "${value}"
    for item in "${items[@]}"; do
        case "${item}" in
            ""|domain|managed_irq|nohz) continue ;;
        esac
        if [[ "${item}" == *-* ]]; then
            first="${item%-*}"
            last="${item#*-}"
        else
            first="${item}"
            last="${item}"
        fi
        [[ "${first}" =~ ^[0-9]+$ && "${last}" =~ ^[0-9]+$ &&
           "${first}" -le "${last}" ]] || fail "invalid CPU list: ${value}"
        for ((cpu=first; cpu<=last; ++cpu)); do
            present["${cpu}"]=1
        done
    done

    local output=""
    for ((cpu=0; cpu<1024; ++cpu)); do
        if [[ -n "${present[${cpu}]+x}" ]]; then
            output+="${output:+,}${cpu}"
        fi
    done
    printf '%s' "${output}"
}

cmdline_value()
{
    local key="$1"
    local token
    for token in $(read_one_line "${PROC_ROOT}/cmdline"); do
        if [[ "${token}" == "${key}="* ]]; then
            printf '%s' "${token#*=}"
            return 0
        fi
    done
    return 1
}

check_boot_layout()
{
    local cmdline isolcpus rcu_nocbs irqaffinity isolated online
    cmdline="$(read_one_line "${PROC_ROOT}/cmdline")"
    [[ " ${cmdline} " != *" nohz_full="* ]] ||
        fail "nohz_full must not be present in the periodic RT baseline"

    isolcpus="$(cmdline_value isolcpus)" || fail "isolcpus is missing"
    [[ ",${isolcpus}," == *,domain,* && ",${isolcpus}," == *,managed_irq,* ]] ||
        fail "isolcpus must contain domain,managed_irq"
    [[ "$(normalize_cpu_list "${isolcpus}")" == "6,7" ]] ||
        fail "isolcpus CPU set must be exactly 6-7"

    rcu_nocbs="$(cmdline_value rcu_nocbs)" || fail "rcu_nocbs is missing"
    [[ "$(normalize_cpu_list "${rcu_nocbs}")" == "6,7" ]] ||
        fail "rcu_nocbs CPU set must be exactly 6-7"

    irqaffinity="$(cmdline_value irqaffinity)" || fail "irqaffinity is missing"
    [[ "$(normalize_cpu_list "${irqaffinity}")" == "0,1,2,3,4,5" ]] ||
        fail "irqaffinity CPU set must be exactly 0-5"

    online="$(read_one_line "${SYS_ROOT}/devices/system/cpu/online")"
    [[ "$(normalize_cpu_list "${online}")" == "0,1,2,3,4,5,6,7" ]] ||
        fail "RK3588 CPUs 0-7 must all be online"
    isolated="$(read_one_line "${SYS_ROOT}/devices/system/cpu/isolated")"
    [[ "$(normalize_cpu_list "${isolated}")" == "6,7" ]] ||
        fail "effective isolated CPU set must be exactly 6-7"
}

interface_irqs()
{
    local interface_name="$1"
    [[ -r "${PROC_ROOT}/interrupts" ]] || fail "cannot read ${PROC_ROOT}/interrupts"
    awk -v interface_name="${interface_name}" '
        {
            irq=$1
            sub(/:$/, "", irq)
            if (irq !~ /^[0-9]+$/) next
            for (i=2; i<=NF; ++i) {
                if ($i == interface_name) {
                    print irq
                    break
                }
            }
        }
    ' "${PROC_ROOT}/interrupts"
}

set_irq_affinity()
{
    local interface_name="$1"
    local cpu="$2"
    local irq found=0 path
    while IFS= read -r irq; do
        [[ -n "${irq}" ]] || continue
        found=1
        path="${PROC_ROOT}/irq/${irq}/smp_affinity_list"
        [[ -w "${path}" ]] || fail "cannot write ${path}"
        printf '%s\n' "${cpu}" > "${path}"
    done < <(interface_irqs "${interface_name}")
    [[ "${found}" -eq 1 ]] || fail "no IRQ found for ${interface_name}"
}

check_irq_affinity()
{
    local interface_name="$1"
    local expected_cpu="$2"
    local irq found=0 actual
    while IFS= read -r irq; do
        [[ -n "${irq}" ]] || continue
        found=1
        actual="$(read_one_line \
            "${PROC_ROOT}/irq/${irq}/effective_affinity_list")"
        [[ "$(normalize_cpu_list "${actual}")" == "${expected_cpu}" ]] ||
            fail "${interface_name} IRQ ${irq} effective affinity is ${actual}, expected ${expected_cpu}"
    done < <(interface_irqs "${interface_name}")
    [[ "${found}" -eq 1 ]] || fail "no IRQ found for ${interface_name}"
}

check_governors()
{
    local policy actual
    for policy in 0 4 6; do
        actual="$(read_one_line \
            "${SYS_ROOT}/devices/system/cpu/cpufreq/policy${policy}/scaling_governor")"
        [[ "${actual}" == "performance" ]] ||
            fail "cpufreq policy${policy} governor is ${actual}, expected performance"
    done
}

set_governors()
{
    local policy path
    for policy in 0 4 6; do
        path="${SYS_ROOT}/devices/system/cpu/cpufreq/policy${policy}/scaling_governor"
        [[ -w "${path}" ]] || fail "cannot write ${path}"
        printf '%s\n' performance > "${path}"
    done
}

check_workqueue()
{
    local mask compact
    mask="$(read_one_line "${SYS_ROOT}/devices/virtual/workqueue/cpumask")"
    compact="${mask//,/}"
    [[ "${compact}" =~ ^[0-9a-fA-F]+$ ]] || fail "invalid workqueue cpumask: ${mask}"
    (( 16#${compact} == 16#3f )) ||
        fail "unbound workqueue cpumask is ${mask}, expected CPU0-5 (3f)"
}

set_workqueue()
{
    local path="${SYS_ROOT}/devices/virtual/workqueue/cpumask"
    [[ -w "${path}" ]] || fail "cannot write ${path}"
    printf '%s\n' 3f > "${path}"
}

resolve_ethercat_interface()
{
    local net_path device_path resolved candidate=""
    for net_path in "${SYS_ROOT}"/class/net/*; do
        [[ -e "${net_path}" ]] || continue
        device_path="${net_path}/device"
        [[ -e "${device_path}" || -L "${device_path}" ]] || continue
        resolved="$(readlink -f -- "${device_path}")" || continue
        [[ "${resolved##*/}" == "${ECAT_DEVICE_ID}" ]] || continue
        if [[ -n "${candidate}" ]]; then
            fail "multiple netdevs map to EtherCAT device ${ECAT_DEVICE_ID}: ${candidate}, ${net_path##*/}"
            return 1
        fi
        candidate="${net_path##*/}"
    done
    [[ -n "${candidate}" ]] ||
        fail "no netdev maps to EtherCAT device ${ECAT_DEVICE_ID}"
    printf '%s' "${candidate}"
}

ethercat_mac()
{
    local interface_name="$1"
    local address_path="${SYS_ROOT}/class/net/${interface_name}/address"
    local assign_type_path="${SYS_ROOT}/class/net/${interface_name}/addr_assign_type"
    local mac assign_type
    mac="$(read_one_line "${address_path}")"
    mac="${mac,,}"
    [[ "${mac}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] ||
        fail "invalid MAC for EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}): ${mac}"
    if [[ -r "${assign_type_path}" ]]; then
        assign_type="$(read_one_line "${assign_type_path}")"
        [[ "${assign_type}" == "0" ]] ||
            fail "EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}) has non-permanent MAC type ${assign_type}"
    fi
    printf '%s' "${mac}"
}

networkmanager_unmanages_mac()
{
    local mac="$1"
    [[ -r "${NM_UNMANAGED_CONFIG}" ]] || return 1
    awk -v target="mac:${mac}" '
        BEGIN { found=0; section="" }
        /^[[:space:]]*[#;]/ { next }
        {
            line=tolower($0)
            gsub(/[[:space:]]/, "", line)
            if (line ~ /^\[[^]]+\]$/) {
                section=line
                next
            }
            if (section != "[keyfile]") next
            if (line !~ /^unmanaged-devices=/) next
            value=substr(line, index(line, "=") + 1)
            count=split(value, entries, ";")
            for (i=1; i<=count; ++i) {
                if (entries[i] == target) found=1
            }
        }
        END { exit(found ? 0 : 1) }
    ' "${NM_UNMANAGED_CONFIG}"
}

check_networkmanager_guard()
{
    local interface_name mac
    interface_name="$(resolve_ethercat_interface)"
    mac="$(ethercat_mac "${interface_name}")"
    networkmanager_unmanages_mac "${mac}" ||
        fail "${NM_UNMANAGED_CONFIG} does not mark EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}, ${mac}) unmanaged"
    echo "[robot-rt] NetworkManager guard passed for ${ECAT_DEVICE_ID} (${interface_name}, ${mac})"
}

check_networkmanager_dropin()
{
    local dropin="${ETC_ROOT}/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf"
    [[ -r "${dropin}" ]] || fail "NetworkManager guard drop-in is missing: ${dropin}"
    grep -Fxq 'ExecStartPre=/usr/local/sbin/robot-rt-setup check-nm-guard' \
        "${dropin}" || fail "NetworkManager guard drop-in is invalid: ${dropin}"
}

configured_master_mac()
{
    local config="${ETC_ROOT}/modprobe.d/ethercat.conf"
    [[ -r "${config}" ]] || fail "cannot read ${config}"
    awk '
        $1 == "options" && $2 == "ec_master" {
            for (i=3; i<=NF; ++i) {
                if ($i ~ /^main_devices=/) {
                    sub(/^main_devices=/, "", $i)
                    print tolower($i)
                    exit
                }
            }
        }
    ' "${config}"
}

check_ethercat()
{
    local interface_name="${1:-}"
    local configured mac loaded_path loaded
    [[ -n "${interface_name}" ]] || interface_name="$(resolve_ethercat_interface)"
    mac="$(ethercat_mac "${interface_name}")"
    configured="$(configured_master_mac)"
    [[ -n "${configured}" ]] || fail "ec_master main_devices is not configured"
    [[ "${configured}" == "${mac}" ]] ||
        fail "ec_master main_devices=${configured} does not match EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}) MAC ${mac}"
    loaded_path="${SYS_ROOT}/module/ec_master/parameters/main_devices"
    if [[ -r "${loaded_path}" ]]; then
        loaded="$(read_one_line "${loaded_path}")"
        loaded="${loaded,,}"
        [[ "${loaded}" == "${mac}" ]] ||
            fail "loaded ec_master main_devices=${loaded} does not match EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}) MAC ${mac}; reboot after install"
    fi
    networkmanager_unmanages_mac "${mac}" ||
        fail "${NM_UNMANAGED_CONFIG} does not mark EtherCAT device ${ECAT_DEVICE_ID} (${interface_name}, ${mac}) unmanaged"
    [[ -e "${DEV_ROOT}/EtherCAT0" ]] ||
        fail "${DEV_ROOT}/EtherCAT0 is unavailable"
}

check_irqbalance_guard()
{
    local dropin="${ETC_ROOT}/systemd/system/irqbalance.service.d/robot-rt.conf"
    if [[ "${ETC_ROOT}" == "/etc" ]] &&
       command -v systemctl >/dev/null 2>&1 &&
       systemctl is-active --quiet irqbalance.service; then
        [[ -r "${dropin}" ]] || fail "irqbalance is active without the RT CPU ban drop-in"
        grep -q 'IRQBALANCE_BANNED_CPULIST=6-7' "${dropin}" ||
            fail "irqbalance drop-in does not ban CPUs 6-7"
    fi
}

check_napi_context()
{
    local interface_name="$1"
    local threaded="${SYS_ROOT}/class/net/${interface_name}/threaded"
    [[ -r "${threaded}" ]] || return 0
    local state
    state="$(read_one_line "${threaded}")"
    case "${state}" in
        0|N|n|no|false) return 0 ;;
        1|Y|y|yes|true) ;;
        *) fail "unknown ${interface_name} threaded NAPI state: ${state}" ;;
    esac

    local comm_path comm status_path affinity found=0
    for comm_path in "${PROC_ROOT}"/[0-9]*/task/[0-9]*/comm; do
        [[ -r "${comm_path}" ]] || continue
        comm="$(read_one_line "${comm_path}")"
        [[ "${comm}" == "napi/${interface_name}" ||
           "${comm}" == "napi/${interface_name}-"* ]] || continue
        found=1
        status_path="${comm_path%/comm}/status"
        affinity="$(awk '/^Cpus_allowed_list:/ { print $2; exit }' "${status_path}")"
        [[ "$(normalize_cpu_list "${affinity}")" == "3" ]] ||
            fail "threaded NAPI ${comm} affinity is ${affinity}, expected CPU3"
    done
    [[ "${found}" -eq 1 ]] ||
        fail "${interface_name} reports threaded NAPI but no napi/${interface_name} thread was found"
}

check_ready_marker()
{
    local boot_id
    boot_id="$(read_one_line "${PROC_ROOT}/sys/kernel/random/boot_id")"
    [[ -r "${READY_FILE}" ]] || fail "ready marker is missing: ${READY_FILE}"
    grep -qx "profile=${PROFILE}" "${READY_FILE}" ||
        fail "ready marker profile is incorrect"
    grep -qx "boot_id=${boot_id}" "${READY_FILE}" ||
        fail "ready marker belongs to another boot"
}

check_runtime_without_marker()
{
    local ethercat_interface
    ethercat_interface="$(resolve_ethercat_interface)"
    check_boot_layout
    check_workqueue
    check_governors
    check_irq_affinity can0 2
    check_irq_affinity "${ethercat_interface}" 3
    check_irqbalance_guard
    check_networkmanager_dropin
    check_napi_context "${ethercat_interface}"
    check_ethercat "${ethercat_interface}"
}

report_network_context()
{
    local interface_name
    interface_name="$(resolve_ethercat_interface)"
    local threaded="${SYS_ROOT}/class/net/${interface_name}/threaded"
    echo "[robot-rt] EtherCAT device ${ECAT_DEVICE_ID}: ${interface_name} ($(ethercat_mac "${interface_name}"))"
    if [[ -r "${threaded}" ]]; then
        echo "[robot-rt] ${interface_name} threaded NAPI: $(read_one_line "${threaded}")"
    else
        echo "[robot-rt] ${interface_name} threaded NAPI state is not exposed; inspect NET_RX and ksoftirqd/3 during trace"
    fi
    if [[ -r "${PROC_ROOT}/softirqs" ]]; then
        awk '/NET_RX:/ { print "[robot-rt] " $0 }' "${PROC_ROOT}/softirqs"
    fi
    if [[ -r "${SYS_ROOT}/module/ec_master/parameters/run_on_cpu" ]]; then
        echo "[robot-rt] IgH run_on_cpu: $(read_one_line "${SYS_ROOT}/module/ec_master/parameters/run_on_cpu")"
    else
        echo "[robot-rt] IgH run_on_cpu is not exposed by this module build"
    fi
}

check_layout()
{
    check_runtime_without_marker
    check_ready_marker
    report_network_context
    echo "[robot-rt] ${PROFILE} layout check passed"
}

apply_layout()
{
    [[ "$(id -u)" -eq 0 || "${PROC_ROOT}" != "/proc" ]] ||
        fail "apply must run as root"
    local ethercat_interface
    ethercat_interface="$(resolve_ethercat_interface)"
    check_boot_layout
    check_ethercat "${ethercat_interface}"
    rm -f -- "${READY_FILE}"
    set_workqueue
    set_governors
    set_irq_affinity can0 2
    set_irq_affinity "${ethercat_interface}" 3
    check_runtime_without_marker

    mkdir -p -- "${RUN_ROOT}"
    {
        echo "profile=${PROFILE}"
        echo "boot_id=$(read_one_line "${PROC_ROOT}/sys/kernel/random/boot_id")"
    } > "${READY_FILE}"
    check_ready_marker
    report_network_context
    echo "[robot-rt] ${PROFILE} layout applied and verified"
}

rewrite_uenv()
{
    local source="$1"
    local resolved current inner token new_args replacement temp
    resolved="$(readlink -f -- "${source}")"
    [[ -f "${resolved}" ]] || fail "active uEnv file not found: ${source}"
    current="$(grep -m1 '^cmdline="' "${resolved}")" ||
        fail "cmdline entry not found in ${resolved}"
    inner="${current#cmdline=\"}"
    inner="${inner%\"}"
    new_args=""
    for token in ${inner}; do
        case "${token}" in
            isolcpus=*|rcu_nocbs=*|irqaffinity=*|nohz_full=*) continue ;;
        esac
        new_args+="${new_args:+ }${token}"
    done
    new_args+=" ${EXPECTED_BOOT_ARGS[*]}"
    replacement="cmdline=\"${new_args}\""

    [[ -e "${resolved}.pre-robot-rt" ]] || cp -a -- "${resolved}" "${resolved}.pre-robot-rt"
    temp="$(mktemp)"
    awk -v replacement="${replacement}" '
        BEGIN { changed=0 }
        /^cmdline="/ { print replacement; changed=1; next }
        { print }
        END { if (!changed) exit 42 }
    ' "${resolved}" > "${temp}"
    install -m "$(stat -c '%a' "${resolved}")" "${temp}" "${resolved}"
    rm -f -- "${temp}"
}

rewrite_ethercat_config()
{
    local mac="$1"
    local config="${ETC_ROOT}/modprobe.d/ethercat.conf"
    local temp
    mkdir -p -- "$(dirname -- "${config}")"
    [[ -e "${config}" ]] || : > "${config}"
    [[ -e "${config}.pre-robot-rt" ]] || cp -a -- "${config}" "${config}.pre-robot-rt"
    temp="$(mktemp)"
    awk -v mac="${mac}" '
        BEGIN { found_options=0 }
        $1 == "options" && $2 == "ec_master" {
            found_options=1
            found_main=0
            for (i=3; i<=NF; ++i) {
                if ($i ~ /^main_devices=/) {
                    $i="main_devices=" mac
                    found_main=1
                }
            }
            if (!found_main) $0=$0 " main_devices=" mac
        }
        { print }
        END { if (!found_options) print "options ec_master main_devices=" mac }
    ' "${config}" > "${temp}"
    install -m 0644 "${temp}" "${config}"
    rm -f -- "${temp}"
}

rewrite_networkmanager_config()
{
    local mac="$1"
    local temp
    mkdir -p -- "$(dirname -- "${NM_UNMANAGED_CONFIG}")"
    if [[ -e "${NM_UNMANAGED_CONFIG}" &&
          ! -e "${NM_UNMANAGED_CONFIG}.pre-robot-rt" ]]; then
        cp -a -- "${NM_UNMANAGED_CONFIG}" \
            "${NM_UNMANAGED_CONFIG}.pre-robot-rt"
    fi
    temp="$(mktemp "${NM_UNMANAGED_CONFIG}.tmp.XXXXXX")"
    {
        echo "[keyfile]"
        echo "unmanaged-devices=mac:${mac}"
    } > "${temp}"
    chmod 0644 "${temp}"
    mv -f -- "${temp}" "${NM_UNMANAGED_CONFIG}"
}

install_layout()
{
    [[ "$(id -u)" -eq 0 || -n "${INSTALL_ROOT}" ]] ||
        fail "install must run as root"

    local root="${INSTALL_ROOT}"
    local ethercat_interface mac
    ethercat_interface="$(resolve_ethercat_interface)"
    mac="$(ethercat_mac "${ethercat_interface}")"

    rewrite_networkmanager_config "${mac}"
    check_networkmanager_guard
    rewrite_ethercat_config "${mac}"
    rewrite_uenv "${BOOT_UENV}"

    install -D -m 0755 "${SCRIPT_DIR}/robot-rt-setup.sh" \
        "${root}/usr/local/sbin/robot-rt-setup"
    install -D -m 0644 "${SCRIPT_DIR}/robot-rt-setup.service" \
        "${root}/etc/systemd/system/robot-rt-setup.service"
    mkdir -p -- "${root}/etc/systemd/system/irqbalance.service.d"
    install -m 0644 "${SCRIPT_DIR}/irqbalance-robot-rt.conf" \
        "${root}/etc/systemd/system/irqbalance.service.d/robot-rt.conf"
    install -D -m 0644 "${SCRIPT_DIR}/networkmanager-ethercat-guard.conf" \
        "${root}/etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf"

    if [[ -z "${root}" && "${ROBOT_RT_SKIP_SYSTEMD:-0}" != "1" ]]; then
        systemctl daemon-reload
        systemctl enable robot-rt-setup.service
    fi

    echo "[robot-rt] EtherCAT device ${ECAT_DEVICE_ID} resolved as ${ethercat_interface} (${mac})"
    echo "[robot-rt] installed ${PROFILE}; reboot is required before apply/check can pass"
    echo "[robot-rt] NetworkManager was not restarted; its next start is guarded by the unmanaged-MAC check"
    echo "[robot-rt] no IRQ or kernel-thread scheduling priority was changed"
}

usage()
{
    echo "Usage: $0 {check|check-nm-guard|apply|install}" >&2
    exit 2
}

case "${1:-}" in
    check) check_layout ;;
    check-nm-guard) check_networkmanager_guard ;;
    apply) apply_layout ;;
    install) install_layout ;;
    *) usage ;;
esac
