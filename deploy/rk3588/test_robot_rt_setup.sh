#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SETUP="${SCRIPT_DIR}/robot-rt-setup.sh"
FIXTURE="$(mktemp -d)"
trap 'rm -rf -- "${FIXTURE}"' EXIT

write_file()
{
    local path="$1"
    local value="$2"
    mkdir -p -- "$(dirname -- "${path}")"
    printf '%s' "${value}" > "${path}"
}

write_file "${FIXTURE}/proc/cmdline" \
    $'console=ttyS2 isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-3\n'
write_file "${FIXTURE}/proc/interrupts" \
    $' 77: 0 0 0 0 0 0 0 0 GIC can0\n142: 0 0 0 0 0 0 0 0 GIC eth0\n143: 0 0 0 0 0 0 0 0 GIC eth0\n'
write_file "${FIXTURE}/proc/sys/kernel/random/boot_id" $'fixture-boot\n'
for irq in 77 142 143; do
    write_file "${FIXTURE}/proc/irq/${irq}/smp_affinity_list" $'0-7\n'
done
write_file "${FIXTURE}/proc/irq/77/effective_affinity_list" $'2\n'
write_file "${FIXTURE}/proc/irq/142/effective_affinity_list" $'3\n'
write_file "${FIXTURE}/proc/irq/143/effective_affinity_list" $'3\n'

write_file "${FIXTURE}/sys/devices/system/cpu/online" $'0-7\n'
write_file "${FIXTURE}/sys/devices/system/cpu/isolated" $'4-7\n'
write_file "${FIXTURE}/sys/devices/virtual/workqueue/cpumask" $'ff\n'
for policy in 0 4 6; do
    write_file "${FIXTURE}/sys/devices/system/cpu/cpufreq/policy${policy}/scaling_governor" \
        $'powersave\n'
done
mkdir -p -- "${FIXTURE}/sys/devices/platform/fe1c0000.ethernet"
mkdir -p -- "${FIXTURE}/sys/devices/platform/fe1b0000.ethernet"
mkdir -p -- "${FIXTURE}/sys/bus/platform/devices/fe1c0000.ethernet"
mkdir -p -- "${FIXTURE}/sys/bus/platform/drivers/rk_gmac-dwmac-ethercat"
ln -s "${FIXTURE}/sys/bus/platform/drivers/rk_gmac-dwmac-ethercat" \
    "${FIXTURE}/sys/bus/platform/devices/fe1c0000.ethernet/driver"
write_file "${FIXTURE}/sys/class/net/eth0/address" $'fa:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/sys/class/net/eth0/addr_assign_type" $'0\n'
write_file "${FIXTURE}/sys/class/net/eth1/address" $'f6:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/sys/class/net/eth1/addr_assign_type" $'0\n'
ln -s "${FIXTURE}/sys/devices/platform/fe1c0000.ethernet" \
    "${FIXTURE}/sys/class/net/eth0/device"
ln -s "${FIXTURE}/sys/devices/platform/fe1b0000.ethernet" \
    "${FIXTURE}/sys/class/net/eth1/device"
write_file "${FIXTURE}/etc/modprobe.d/ethercat.conf" \
    $'options ec_master main_devices=fa:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/sys/module/ec_master/parameters/main_devices" \
    $'fa:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[keyfile]\nunmanaged-devices=mac:fa:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf" \
    $'[Service]\nExecStartPre=/usr/bin/env ROBOT_RT_INTERNAL_COMMAND=1 /usr/local/sbin/robot-rt-setup __check-nm-guard\n'
write_file "${FIXTURE}/dev/EtherCAT0" ''

run_fixture_command()
{
    TEST_FIXTURE="${FIXTURE}" TEST_SETUP="${SETUP}" bash -c '
        source "${TEST_SETUP}"
        PROC_ROOT="${TEST_FIXTURE}/proc"
        SYS_ROOT="${TEST_FIXTURE}/sys"
        RUN_ROOT="${TEST_FIXTURE}/run"
        DEV_ROOT="${TEST_FIXTURE}/dev"
        ETC_ROOT="${TEST_FIXTURE}/etc"
        BOOT_UENV="${TEST_FIXTURE}/boot/uEnv/uEnv.txt"
        INSTALL_ROOT="${TEST_FIXTURE}/stage"
        READY_FILE="${RUN_ROOT}/robot-rt-layout.ready"
        NM_UNMANAGED_CONFIG="${ETC_ROOT}/NetworkManager/conf.d/99-ethercat-unmanaged.conf"
        ROBOT_RT_SKIP_SYSTEMD=1
        SCRIPT_DIR="$(dirname -- "${TEST_SETUP}")"

        case "$1" in
            check)
                check_layout
                ;;
            install)
                install_layout
                ;;
            __apply)
                apply_layout
                ;;
            __check-nm-guard)
                check_networkmanager_config "$(loaded_master_mac)"
                ;;
            *)
                echo "unexpected fixture command: $1" >&2
                exit 1
                ;;
        esac
    ' robot-rt-fixture "$1"
}

run_setup()
{
    run_fixture_command "$1"
}

run_internal_setup()
{
    run_fixture_command "$1"
}

run_internal_setup __apply
run_internal_setup __apply
run_setup check
run_internal_setup __check-nm-guard

if "${SETUP}" apply >/dev/null 2>&1; then
    echo "public apply command unexpectedly passed" >&2
    exit 1
fi
if "${SETUP}" check-nm-guard >/dev/null 2>&1; then
    echo "public check-nm-guard command unexpectedly passed" >&2
    exit 1
fi

write_file "${FIXTURE}/run/robot-rt-layout.ready" \
    $'profile=wrong-profile\nboot_id=fixture-boot\n'
if run_setup check >/dev/null 2>&1; then
    echo "ready marker with wrong profile unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/run/robot-rt-layout.ready" \
    $'profile=rk3588-v1\nboot_id=old-boot\n'
if run_setup check >/dev/null 2>&1; then
    echo "ready marker from another boot unexpectedly passed" >&2
    exit 1
fi
rm -f -- "${FIXTURE}/run/robot-rt-layout.ready"
if run_setup check >/dev/null 2>&1; then
    echo "missing ready marker unexpectedly passed" >&2
    exit 1
fi
run_internal_setup __apply >/dev/null
run_setup check >/dev/null

[[ "$(< "${FIXTURE}/proc/irq/77/smp_affinity_list")" == "2" ]]
[[ "$(< "${FIXTURE}/proc/irq/142/smp_affinity_list")" == "0-7" ]]
[[ "$(< "${FIXTURE}/proc/irq/143/smp_affinity_list")" == "0-7" ]]
[[ "$(< "${FIXTURE}/sys/devices/virtual/workqueue/cpumask")" == "0f" ]]

write_file "${FIXTURE}/proc/cmdline" \
    $'isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5\n'
if run_setup check >/dev/null 2>&1; then
    echo "old CPU6-7 isolation layout unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/proc/cmdline" \
    $'isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-5\n'
if run_setup check >/dev/null 2>&1; then
    echo "wrong default IRQ affinity unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/proc/cmdline" \
    $'console=ttyS2 isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-3\n'
write_file "${FIXTURE}/sys/devices/virtual/workqueue/cpumask" $'3f\n'
if run_setup check >/dev/null 2>&1; then
    echo "old workqueue mask unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/sys/devices/virtual/workqueue/cpumask" $'0f\n'

write_file "${FIXTURE}/etc/modprobe.d/ethercat.conf" \
    $'options ec_master main_devices=f6:fd:53:a0:a5:55\n'
if run_setup check >/dev/null 2>&1; then
    echo "normal Linux NIC MAC unexpectedly passed as EtherCAT MAC" >&2
    exit 1
fi
write_file "${FIXTURE}/etc/modprobe.d/ethercat.conf" \
    $'options ec_master main_devices=fa:fd:53:a0:a5:55\n'

write_file "${FIXTURE}/sys/module/ec_master/parameters/main_devices" \
    $'f6:fd:53:a0:a5:55\n'
if run_setup check >/dev/null 2>&1; then
    echo "stale loaded ec_master MAC unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/sys/module/ec_master/parameters/main_devices" \
    $'fa:fd:53:a0:a5:55\n'

write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[keyfile]\nunmanaged-devices=mac:f6:fd:53:a0:a5:55\n'
if run_internal_setup __check-nm-guard >/dev/null 2>&1; then
    echo "wrong NetworkManager unmanaged MAC unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[connection]\nunmanaged-devices=mac:fa:fd:53:a0:a5:55\n'
if run_internal_setup __check-nm-guard >/dev/null 2>&1; then
    echo "NetworkManager unmanaged MAC in the wrong section unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[keyfile]\nunmanaged-devices=mac:fa:fd:53:a0:a5:55\n'

mv "${FIXTURE}/sys/class/net/eth0" "${FIXTURE}/sys/class/net/ecat0"
write_file "${FIXTURE}/proc/interrupts" \
    $' 77: 0 0 0 0 0 0 0 0 GIC can0\n142: 0 0 0 0 0 0 0 0 GIC ecat0\n143: 0 0 0 0 0 0 0 0 GIC ecat0\n'
run_setup check >/dev/null
mv "${FIXTURE}/sys/class/net/ecat0" "${FIXTURE}/sys/class/net/eth0"
write_file "${FIXTURE}/proc/interrupts" \
    $' 77: 0 0 0 0 0 0 0 0 GIC can0\n142: 0 0 0 0 0 0 0 0 GIC eth0\n143: 0 0 0 0 0 0 0 0 GIC eth0\n'

write_file "${FIXTURE}/proc/cmdline" \
    $'isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-3 nohz_full=7\n'
if run_setup check >/dev/null 2>&1; then
    echo "nohz_full unexpectedly passed" >&2
    exit 1
fi

write_file "${FIXTURE}/boot/uEnv/active.txt" \
    $'uname_r=6.1.99-rt36-rk3588\ncmdline="console=ttyS2 isolcpus=7 nohz_full=7 rcu_nocbs=7"\n'
ln -s active.txt "${FIXTURE}/boot/uEnv/uEnv.txt"
write_file "${FIXTURE}/etc/modprobe.d/ethercat.conf" \
    $'options ec_master main_devices=f6:fd:53:a0:a5:55\n'
write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[keyfile]\nunmanaged-devices=mac:f6:fd:53:a0:a5:55\n'
chmod 0640 "${FIXTURE}/boot/uEnv/active.txt"
chmod 0600 "${FIXTURE}/etc/modprobe.d/ethercat.conf"
chmod 0640 "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf"
mv "${FIXTURE}/sys/class/net/eth0" "${FIXTURE}/sys/class/net/net-swap"
mv "${FIXTURE}/sys/class/net/eth1" "${FIXTURE}/sys/class/net/eth0"
mv "${FIXTURE}/sys/class/net/net-swap" "${FIXTURE}/sys/class/net/eth1"
run_setup install
run_setup install

grep -q 'isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 irqaffinity=0-3' \
    "${FIXTURE}/boot/uEnv/active.txt"
if grep -q 'nohz_full=' "${FIXTURE}/boot/uEnv/active.txt"; then
    echo "install retained nohz_full" >&2
    exit 1
fi
grep -q 'main_devices=fa:fd:53:a0:a5:55' \
    "${FIXTURE}/etc/modprobe.d/ethercat.conf"
grep -q 'unmanaged-devices=mac:fa:fd:53:a0:a5:55' \
    "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf"
grep -q 'unmanaged-devices=mac:f6:fd:53:a0:a5:55' \
    "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf.pre-robot-rt"
[[ "$(stat -c '%a' "${FIXTURE}/boot/uEnv/active.txt")" == "640" ]]
[[ -r "${FIXTURE}/boot/uEnv/active.txt.pre-robot-rt" ]]
[[ "$(stat -c '%a' "${FIXTURE}/etc/modprobe.d/ethercat.conf")" == "600" ]]
[[ "$(stat -c '%a' "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf")" == "640" ]]
if find "${FIXTURE}" -name '*.tmp.*' -print -quit | grep -q .; then
    echo "configuration rewrite left a temporary file behind" >&2
    exit 1
fi
[[ -x "${FIXTURE}/stage/usr/local/sbin/robot-rt-setup" ]]
cmp -s "${SETUP}" "${FIXTURE}/stage/usr/local/sbin/robot-rt-setup"
grep -q '__apply' "${FIXTURE}/stage/etc/systemd/system/robot-rt-setup.service"
grep -q 'IRQBALANCE_BANNED_CPULIST=4-7' \
    "${FIXTURE}/stage/etc/systemd/system/irqbalance.service.d/robot-rt.conf"
grep -q 'IRQBALANCE_BANNED_CPUS=000000f0' \
    "${FIXTURE}/stage/etc/systemd/system/irqbalance.service.d/robot-rt.conf"
if grep -R -qE 'chrt|SCHED_FIFO|nohz_full' \
    "${FIXTURE}/stage/etc/systemd/system"; then
    echo "installed systemd configuration changed RT priorities or added nohz_full" >&2
    exit 1
fi

echo "test_robot_rt_setup passed"
