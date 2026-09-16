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
                   "console=ttyS2 isolcpus=domain,managed_irq,4-7 "
                   "rcu_nocbs=4-7 irqaffinity=0-3\n");
        write_file(root / "proc/interrupts",
                   " 77: 0 0 0 0 0 0 0 0 GIC can0\n");
        write_file(root / "proc/irq/77/effective_affinity_list", "2\n");
        write_file(root / "proc/sys/kernel/random/boot_id", "test-boot-id\n");
        write_file(root / "sys/devices/system/cpu/online", "0-7\n");
        write_file(root / "sys/devices/system/cpu/isolated", "4-7\n");
        write_file(root / "sys/devices/virtual/workqueue/cpumask", "0000000f\n");
        const auto ethercat_device =
            root / "sys/bus/platform/devices/fe1c0000.ethernet";
        const auto ethercat_driver =
            root / "sys/bus/platform/drivers/rk_gmac-dwmac-ethercat";
        std::filesystem::create_directories(ethercat_device);
        std::filesystem::create_directories(ethercat_driver);
        std::filesystem::create_directory_symlink(
            ethercat_driver, ethercat_device / "driver");
        for (const int policy : {0, 4, 6}) {
            write_file(root / "sys/devices/system/cpu/cpufreq" /
                           ("policy" + std::to_string(policy)) /
                           "scaling_governor",
                       "performance\n");
        }
        write_file(root / "run/robot-rt-layout.ready",
                   "profile=rk3588-rt\nboot_id=test-boot-id\n");
        write_file(root / "dev/EtherCAT0", "");
        write_file(root / "etc/modprobe.d/ethercat.conf",
                   "options ec_master main_devices=fa:fd:53:a0:a5:55\n");
        write_file(root / "sys/module/ec_master/parameters/main_devices",
                   "fa:fd:53:a0:a5:55\n");
        write_file(root /
                       "etc/NetworkManager/conf.d/99-ethercat-unmanaged.conf",
                   "[keyfile]\n"
                   "unmanaged-devices=mac:fa:fd:53:a0:a5:55\n");
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
               "isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 "
               "irqaffinity=0-3 nohz_full=7\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "nohz_full unexpectedly passed");
    expect(error.find("nohz_full") != std::string::npos,
           "nohz_full failure was not reported");
}

void test_old_6_7_layout_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/cmdline",
               "isolcpus=domain,managed_irq,6-7 rcu_nocbs=6-7 "
               "irqaffinity=0-5\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "old CPU6-7 isolation layout unexpectedly passed");
    expect(error.find("isolcpus") != std::string::npos,
           "old isolation layout failure was not identified");
}

void test_wrong_default_irq_mask_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/cmdline",
               "isolcpus=domain,managed_irq,4-7 rcu_nocbs=4-7 "
               "irqaffinity=0-5\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "wrong default IRQ affinity unexpectedly passed");
    expect(error.find("irqaffinity") != std::string::npos,
           "wrong default IRQ affinity was not identified");
}

void test_wrong_rcu_mask_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/cmdline",
               "isolcpus=domain,managed_irq,4-7 rcu_nocbs=6-7 "
               "irqaffinity=0-3\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "old RCU mask unexpectedly passed");
    expect(error.find("rcu_nocbs") != std::string::npos,
           "wrong RCU mask was not identified");
}

void test_wrong_effective_isolation_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "sys/devices/system/cpu/isolated", "6-7\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "old effective isolation unexpectedly passed");
    expect(error.find("isolated CPU") != std::string::npos,
           "wrong effective isolation was not identified");
}

void test_wrong_workqueue_mask_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "sys/devices/virtual/workqueue/cpumask",
               "0000003f\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "old workqueue mask unexpectedly passed");
    expect(error.find("workqueue") != std::string::npos,
           "wrong workqueue mask was not identified");
}

void test_wrong_irq_affinity_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "proc/irq/77/effective_affinity_list", "7\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "wrong CAN IRQ affinity unexpectedly passed");
    expect(error.find("can0 IRQ 77") != std::string::npos,
           "wrong CAN IRQ was not identified");
}

void test_stale_marker_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "run/robot-rt-layout.ready",
               "profile=rk3588-rt\nboot_id=old-boot\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "stale ready marker unexpectedly passed");
}

void test_wrong_ethercat_driver_is_rejected()
{
    Fixture fixture;
    const auto device =
        fixture.root / "sys/bus/platform/devices/fe1c0000.ethernet";
    std::filesystem::remove(device / "driver");
    const auto wrong_driver =
        fixture.root / "sys/bus/platform/drivers/rk_gmac-dwmac";
    std::filesystem::create_directories(wrong_driver);
    std::filesystem::create_directory_symlink(wrong_driver, device / "driver");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "wrong EtherCAT platform driver unexpectedly passed");
    expect(error.find("expected rk_gmac-dwmac-ethercat") != std::string::npos,
           "wrong EtherCAT platform driver was not identified");
}

void test_wrong_master_mac_is_rejected()
{
    Fixture fixture;
    write_file(fixture.root / "etc/modprobe.d/ethercat.conf",
               "options ec_master main_devices=f6:fd:53:a0:a5:55\n");
    std::string error;
    expect(!inference::verify_rk3588_host_layout(error, fixture.paths),
           "normal Linux NIC MAC unexpectedly passed as EtherCAT MAC");
    expect(error.find("does not match configured value") != std::string::npos,
           "configured/loaded EtherCAT MAC mismatch was not identified");
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

}  // namespace

int main()
{
    test_valid_layout();
    test_nohz_full_is_rejected();
    test_old_6_7_layout_is_rejected();
    test_wrong_default_irq_mask_is_rejected();
    test_wrong_rcu_mask_is_rejected();
    test_wrong_effective_isolation_is_rejected();
    test_wrong_workqueue_mask_is_rejected();
    test_wrong_irq_affinity_is_rejected();
    test_stale_marker_is_rejected();
    test_wrong_ethercat_driver_is_rejected();
    test_wrong_master_mac_is_rejected();
    test_stale_loaded_master_mac_is_rejected();
    test_missing_networkmanager_guard_is_rejected();
    test_networkmanager_guard_requires_keyfile_section();
    std::cout << "rk3588_host_preflight_test passed\n";
    return 0;
}
