#include "robot/rk3588_host_preflight.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {

void expect(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void write_file(const std::filesystem::path& path, const std::string& value)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    expect(static_cast<bool>(output), "cannot create " + path.string());
    output << value;
}

class Fixture {
public:
    Fixture()
        : root(std::filesystem::temp_directory_path() /
               ("rk3588-preflight-" + std::to_string(::getpid()) + "-" +
                std::to_string(next_id++)))
    {
        paths.proc_root = (root / "proc").string();
        paths.sys_root = (root / "sys").string();
        paths.run_root = (root / "run").string();
        paths.dev_root = (root / "dev").string();
        paths.etc_root = (root / "etc").string();

        write_file(root / "proc/cmdline",
                   "console=ttyS2 isolcpus=domain,managed_irq,6-7 "
                   "rcu_nocbs=6-7 irqaffinity=0-5\n");
        write_file(root / "proc/interrupts",
                   " 77: 0 0 0 0 0 0 0 0 GIC can0\n"
                   "142: 0 0 0 0 0 0 0 0 GIC eth0\n"
                   "143: 0 0 0 0 0 0 0 0 GIC eth0\n");
        write_file(root / "proc/irq/77/effective_affinity_list", "2\n");
        write_file(root / "proc/irq/142/effective_affinity_list", "3\n");
        write_file(root / "proc/irq/143/effective_affinity_list", "3\n");
        write_file(root / "proc/sys/kernel/random/boot_id", "test-boot-id\n");
        write_file(root / "sys/devices/system/cpu/online", "0-7\n");
        write_file(root / "sys/devices/system/cpu/isolated", "6-7\n");
        write_file(root / "sys/devices/virtual/workqueue/cpumask", "0000003f\n");
        const auto ethercat_device =
            root / "sys/devices/platform/fe1c0000.ethernet";
        const auto linux_device =
            root / "sys/devices/platform/fe1b0000.ethernet";
        std::filesystem::create_directories(ethercat_device);
        std::filesystem::create_directories(linux_device);
        write_file(root / "sys/class/net/eth0/address", "fa:fd:53:a0:a5:55\n");
        write_file(root / "sys/class/net/eth0/addr_assign_type", "0\n");
        write_file(root / "sys/class/net/eth1/address", "f6:fd:53:a0:a5:55\n");
        write_file(root / "sys/class/net/eth1/addr_assign_type", "0\n");
        std::filesystem::create_directory_symlink(
            ethercat_device, root / "sys/class/net/eth0/device");
        std::filesystem::create_directory_symlink(
            linux_device, root / "sys/class/net/eth1/device");
        for (const int policy : {0, 4, 6}) {
            write_file(root / "sys/devices/system/cpu/cpufreq" /
                           ("policy" + std::to_string(policy)) /
                           "scaling_governor",
                       "performance\n");
        }
        write_file(root / "run/robot-rt-layout.ready",
                   "profile=rk3588-v1\nboot_id=test-boot-id\n");
        write_file(root / "dev/EtherCAT0", "");
        write_file(root / "etc/modprobe.d/ethercat.conf",
                   "options ec_master main_devices=fa:fd:53:a0:a5:55\n");
        write_file(root / "sys/module/ec_master/parameters/main_devices",
                   "fa:fd:53:a0:a5:55\n");
        write_file(root /
                       "etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf",
                   "[keyfile]\n"
                   "unmanaged-devices=mac:fa:fd:53:a0:a5:55\n");
        write_file(root /
                       "etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf",
                   "[Service]\n"
                   "ExecStartPre=/usr/local/sbin/robot-rt-setup check-nm-guard\n");
    }

    ~Fixture()
    {
        std::error_code ignored;
        std::filesystem::remove_all(root, ignored);
    }

    std::filesystem::path root;
    inference::Rk3588HostPaths paths;

private:
    static int next_id;
};

int Fixture::next_id = 0;

void test_valid_layout()
{
    Fixture fixture;
    std::string error;
    expect(inference::verify_rk3588_host_layout(error, fixture.paths), error);
}

void test_nohz_full_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/cmdline",
               "isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 "
               "irqaffinity=0-5 nohz_full=7\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "nohz_full unexpectedly passed");
    expect(error.find("nohz_full") != std::string::npos,
           "nohz_full failure was not reported");
}

void test_wrong_irq_affinity_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/irq/142/effective_affinity_list", "7\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "wrong EtherCAT IRQ affinity unexpectedly passed");
    expect(error.find("eth0 IRQ 142") != std::string::npos,
           "wrong EtherCAT IRQ was not identified");
}

void test_stale_marker_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "run/robot-rt-layout.ready",
               "profile=rk3588-v1\nboot_id=old-boot\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "stale ready marker unexpectedly passed");
}

void test_threaded_napi_affinity_is_verified()
{
    Fixture fixture;
    write_file(fixture.root / "sys/class/net/eth0/threaded", "1\n");
    write_file(fixture.root / "proc/42/task/43/comm", "napi/eth0-7\n");
    write_file(fixture.root / "proc/42/task/43/status",
               "Name:\tnapi/eth0-7\nCpus_allowed_list:\t3\n");
    std::string error;
    expect(inference::verify_rk3588_host_layout(error, fixture.paths), error);

    write_file(fixture.root / "proc/42/task/43/status",
               "Name:\tnapi/eth0-7\nCpus_allowed_list:\t7\n");
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "threaded NAPI on CPU7 unexpectedly passed");
    expect(error.find("threaded NAPI") != std::string::npos,
           "threaded NAPI failure was not reported");
}

void test_interface_rename_is_supported()
{
    Fixture fixture;
    std::filesystem::rename(fixture.root / "sys/class/net/eth0",
                            fixture.root / "sys/class/net/ecat0");
    write_file(fixture.root / "proc/interrupts",
               " 77: 0 0 0 0 0 0 0 0 GIC can0\n"
               "142: 0 0 0 0 0 0 0 0 GIC ecat0\n"
               "143: 0 0 0 0 0 0 0 0 GIC ecat0\n");
    std::string error;
    expect(inference::verify_rk3588_host_layout(error, fixture.paths), error);
}

void test_wrong_master_mac_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "etc/modprobe.d/ethercat.conf",
               "options ec_master main_devices=f6:fd:53:a0:a5:55\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "normal Linux NIC MAC unexpectedly passed as EtherCAT MAC");
    expect(error.find("fe1c0000.ethernet") != std::string::npos,
           "physical EtherCAT device was not identified");
}

void test_missing_networkmanager_guard_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root /
                   "etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf",
               "[keyfile]\nunmanaged-devices=mac:f6:fd:53:a0:a5:55\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "wrong NetworkManager unmanaged MAC unexpectedly passed");
    expect(error.find("unmanaged") != std::string::npos,
           "NetworkManager guard failure was not reported");
}

void test_stale_loaded_master_mac_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "sys/module/ec_master/parameters/main_devices",
               "f6:fd:53:a0:a5:55\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "stale loaded ec_master MAC unexpectedly passed");
    expect(error.find("reboot after install") != std::string::npos,
           "stale loaded ec_master MAC did not request reboot");
}

void test_networkmanager_guard_requires_keyfile_section()
{
    Fixture fixture;
    write_file(fixture.root /
                   "etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf",
               "[connection]\n"
               "unmanaged-devices=mac:fa:fd:53:a0:a5:55\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "unmanaged MAC outside [keyfile] unexpectedly passed");
}

void test_non_permanent_ethercat_mac_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "sys/class/net/eth0/addr_assign_type", "1\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "non-permanent EtherCAT MAC unexpectedly passed");
    expect(error.find("non-permanent") != std::string::npos,
           "non-permanent MAC failure was not reported");
}

void test_missing_networkmanager_dropin_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root /
                   "etc/systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf",
               "[Service]\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "missing NetworkManager startup guard unexpectedly passed");
    expect(error.find("guard drop-in") != std::string::npos,
           "NetworkManager drop-in failure was not reported");
}

}  // namespace

int main()
{
    test_valid_layout();
    test_nohz_full_is_rejected();
    test_wrong_irq_affinity_is_rejected();
    test_stale_marker_is_rejected();
    test_threaded_napi_affinity_is_verified();
    test_interface_rename_is_supported();
    test_wrong_master_mac_is_rejected();
    test_stale_loaded_master_mac_is_rejected();
    test_missing_networkmanager_guard_is_rejected();
    test_networkmanager_guard_requires_keyfile_section();
    test_non_permanent_ethercat_mac_is_rejected();
    test_missing_networkmanager_dropin_is_rejected();
    std::cout << "rk3588_host_preflight_test passed\n";
    return 0;
}
