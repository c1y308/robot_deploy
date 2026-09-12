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
    $'console=ttyS2 isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5\n'
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
write_file "${FIXTURE}/sys/devices/system/cpu/isolated" $'6-7\n'
write_file "${FIXTURE}/sys/devices/virtual/workqueue/cpumask" $'ff\n'
for policy in 0 4 6; do
    write_file "${FIXTURE}/sys/devices/system/cpu/cpufreq/policy${policy}/scaling_governor" \
        $'powersave\n'
done
mkdir -p -- "${FIXTURE}/sys/devices/platform/fe1c0000.ethernet"
mkdir -p -- "${FIXTURE}/sys/devices/platform/fe1b0000.ethernet"
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
    $'[Service]\nExecStartPre=/usr/local/sbin/robot-rt-setup check-nm-guard\n'
write_file "${FIXTURE}/dev/EtherCAT0" ''

run_setup()
{
    ROBOT_RT_PROC_ROOT="${FIXTURE}/proc" \
    ROBOT_RT_SYS_ROOT="${FIXTURE}/sys" \
    ROBOT_RT_RUN_ROOT="${FIXTURE}/run" \
    ROBOT_RT_DEV_ROOT="${FIXTURE}/dev" \
    ROBOT_RT_ETC_ROOT="${FIXTURE}/etc" \
    "${SETUP}" "$@"
}

run_setup apply
run_setup apply
run_setup check
run_setup check-nm-guard

[[ "$(< "${FIXTURE}/proc/irq/77/smp_affinity_list")" == "2" ]]
[[ "$(< "${FIXTURE}/proc/irq/142/smp_affinity_list")" == "3" ]]
[[ "$(< "${FIXTURE}/proc/irq/143/smp_affinity_list")" == "3" ]]
[[ "$(< "${FIXTURE}/sys/devices/virtual/workqueue/cpumask")" == "3f" ]]

write_file "${FIXTURE}/sys/class/net/eth0/threaded" $'1\n'
write_file "${FIXTURE}/proc/42/task/43/comm" $'napi/eth0-7\n'
write_file "${FIXTURE}/proc/42/task/43/status" \
    $'Name:\tnapi/eth0-7\nCpus_allowed_list:\t3\n'
run_setup check >/dev/null
write_file "${FIXTURE}/proc/42/task/43/status" \
    $'Name:\tnapi/eth0-7\nCpus_allowed_list:\t7\n'
if run_setup check >/dev/null 2>&1; then
    echo "threaded NAPI on CPU7 unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/sys/class/net/eth0/threaded" $'0\n'

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
if run_setup check-nm-guard >/dev/null 2>&1; then
    echo "wrong NetworkManager unmanaged MAC unexpectedly passed" >&2
    exit 1
fi
write_file "${FIXTURE}/etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf" \
    $'[connection]\nunmanaged-devices=mac:fa:fd:53:a0:a5:55\n'
if run_setup check-nm-guard >/dev/null 2>&1; then
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
    $'isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5 nohz_full=7\n'
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
mv "${FIXTURE}/sys/class/net/eth0" "${FIXTURE}/sys/class/net/net-swap"
mv "${FIXTURE}/sys/class/net/eth1" "${FIXTURE}/sys/class/net/eth0"
mv "${FIXTURE}/sys/class/net/net-swap" "${FIXTURE}/sys/class/net/eth1"
ROBOT_RT_PROC_ROOT="${FIXTURE}/proc" \
ROBOT_RT_SYS_ROOT="${FIXTURE}/sys" \
ROBOT_RT_ETC_ROOT="${FIXTURE}/etc" \
ROBOT_RT_BOOT_UENV="${FIXTURE}/boot/uEnv/uEnv.txt" \
ROBOT_RT_INSTALL_ROOT="${FIXTURE}/stage" \
ROBOT_RT_SKIP_SYSTEMD=1 \
"${SETUP}" install
ROBOT_RT_PROC_ROOT="${FIXTURE}/proc" \
ROBOT_RT_SYS_ROOT="${FIXTURE}/sys" \
ROBOT_RT_ETC_ROOT="${FIXTURE}/etc" \
ROBOT_RT_BOOT_UENV="${FIXTURE}/boot/uEnv/uEnv.txt" \
ROBOT_RT_INSTALL_ROOT="${FIXTURE}/stage" \
ROBOT_RT_SKIP_SYSTEMD=1 \
"${SETUP}" install

grep -q 'isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 irqaffinity=0-5' \
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
[[ -x "${FIXTURE}/stage/usr/local/sbin/robot-rt-setup" ]]
[[ -r "${FIXTURE}/stage/etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf" ]]
grep -q 'check-nm-guard' \
    "${FIXTURE}/stage/etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf"
if grep -R -qE 'chrt|SCHED_FIFO|nohz_full' \
    "${FIXTURE}/stage/etc/systemd/system"; then
    echo "installed systemd configuration changed RT priorities or added nohz_full" >&2
    exit 1
fi

echo "test_robot_rt_setup passed"
