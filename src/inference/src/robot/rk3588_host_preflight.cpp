#include "robot/rk3588_host_preflight.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace inference {
namespace {

bool read_text(const std::filesystem::path& path,
               std::string& output,
               std::string& error)
{
    std::ifstream input(path);
    if (!input) {
        error = "cannot read " + path.string();
        return false;
    }
    std::ostringstream contents;
    contents << input.rdbuf();
    output = contents.str();
    while (!output.empty() && std::isspace(
               static_cast<unsigned char>(output.back()))) {
        output.pop_back();
    }
    return true;
}

bool parse_cpu_list(const std::string& text,
                    std::set<int>& cpus,
                    std::string& error)
{
    cpus.clear();
    std::istringstream input(text);
    std::string item;
    while (std::getline(input, item, ',')) {
        if (item.empty() || item == "domain" || item == "managed_irq" ||
            item == "nohz") {
            continue;
        }
        const std::size_t dash = item.find('-');
        try {
            const int first = std::stoi(item.substr(0, dash));
            const int last = dash == std::string::npos
                                 ? first
                                 : std::stoi(item.substr(dash + 1));
            if (first < 0 || last < first || last >= 1024) {
                error = "invalid CPU list: " + text;
                return false;
            }
            for (int cpu = first; cpu <= last; ++cpu) {
                cpus.insert(cpu);
            }
        } catch (const std::exception&) {
            error = "invalid CPU list: " + text;
            return false;
        }
    }
    return true;
}

bool extract_cmdline_value(const std::string& cmdline,
                           const std::string& key,
                           std::string& value)
{
    std::istringstream input(cmdline);
    std::string token;
    const std::string prefix = key + "=";
    while (input >> token) {
        if (token.compare(0, prefix.size(), prefix) == 0) {
            value = token.substr(prefix.size());
            return true;
        }
    }
    return false;
}

bool require_cpus(const std::set<int>& actual,
                  const std::set<int>& required,
                  const std::string& context,
                  std::string& error)
{
    for (const int cpu : required) {
        if (actual.count(cpu) == 0U) {
            error = context + " does not contain CPU" + std::to_string(cpu);
            return false;
        }
    }
    return true;
}

bool verify_irq_affinity(const Rk3588HostPaths& paths,
                         const std::string& interface_name,
                         const std::set<int>& expected,
                         std::string& error)
{
    std::string interrupts;
    if (!read_text(std::filesystem::path(paths.proc_root) / "interrupts",
                   interrupts,
                   error)) {
        return false;
    }

    bool found = false;
    std::istringstream lines(interrupts);
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream tokens(line);
        std::string first;
        if (!(tokens >> first) || first.empty() || first.back() != ':') {
            continue;
        }
        bool matches = false;
        std::string token;
        while (tokens >> token) {
            if (token == interface_name) {
                matches = true;
            }
        }
        if (!matches) {
            continue;
        }
        first.pop_back();
        if (!std::all_of(first.begin(), first.end(), [](unsigned char value) {
                return std::isdigit(value) != 0;
            })) {
            continue;
        }

        found = true;
        std::string affinity;
        const auto path = std::filesystem::path(paths.proc_root) / "irq" / first /
                          "effective_affinity_list";
        if (!read_text(path, affinity, error)) {
            return false;
        }
        std::set<int> actual;
        if (!parse_cpu_list(affinity, actual, error)) {
            return false;
        }
        if (actual != expected) {
            error = interface_name + " IRQ " + first +
                    " effective affinity is " + affinity;
            return false;
        }
    }

    if (!found) {
        error = "no IRQ found for " + interface_name;
        return false;
    }
    return true;
}

bool verify_governor(const Rk3588HostPaths& paths,
                     int policy,
                     std::string& error)
{
    std::string governor;
    const auto path = std::filesystem::path(paths.sys_root) / "devices/system/cpu/cpufreq" /
                      ("policy" + std::to_string(policy)) / "scaling_governor";
    if (!read_text(path, governor, error)) {
        return false;
    }
    if (governor != "performance") {
        error = path.string() + " is " + governor + ", expected performance";
        return false;
    }
    return true;
}

bool valid_mac_address(const std::string& address)
{
    if (address.size() != 17U) {
        return false;
    }
    for (std::size_t index = 0; index < address.size(); ++index) {
        if (index % 3U == 2U) {
            if (address[index] != ':') {
                return false;
            }
        } else if (std::isxdigit(
                       static_cast<unsigned char>(address[index])) == 0) {
            return false;
        }
    }
    return true;
}

bool verify_ethercat_platform_driver(const Rk3588HostPaths& paths,
                                     std::string& error)
{
    const auto device_path = std::filesystem::path(paths.sys_root) /
                             "bus/platform/devices" /
                             paths.ethercat_device_id;
    if (!std::filesystem::exists(device_path)) {
        error = "EtherCAT device " + paths.ethercat_device_id +
                " is unavailable";
        return false;
    }

    std::error_code canonical_error;
    const auto driver_path =
        std::filesystem::canonical(device_path / "driver", canonical_error);
    if (canonical_error) {
        error = "EtherCAT device " + paths.ethercat_device_id +
                " has no bound platform driver";
        return false;
    }

    const std::string driver = driver_path.filename().string();
    if (driver != paths.ethercat_driver) {
        error = "EtherCAT device " + paths.ethercat_device_id +
                " is bound to " + driver + ", expected " +
                paths.ethercat_driver;
        return false;
    }
    return true;
}

bool configured_master_mac(const Rk3588HostPaths& paths,
                           std::string& address,
                           std::string& error)
{
    std::string config;
    if (!read_text(std::filesystem::path(paths.etc_root) /
                       "modprobe.d/ethercat.conf",
                   config,
                   error)) {
        return false;
    }

    std::istringstream lines(config);
    std::string line;
    while (std::getline(lines, line)) {
        std::istringstream tokens(line);
        std::string first;
        std::string second;
        if (!(tokens >> first >> second) || first != "options" ||
            second != "ec_master") {
            continue;
        }
        std::string token;
        while (tokens >> token) {
            const std::string prefix = "main_devices=";
            if (token.compare(0, prefix.size(), prefix) != 0) {
                continue;
            }
            address = token.substr(prefix.size());
            std::transform(address.begin(), address.end(), address.begin(),
                           [](unsigned char value) {
                               return static_cast<char>(std::tolower(value));
                           });
            if (!valid_mac_address(address)) {
                error = "invalid configured ec_master main_devices: " + address;
                return false;
            }
            return true;
        }
    }

    error = "ec_master main_devices is not configured";
    return false;
}

bool networkmanager_unmanages_mac(const std::string& config,
                                  const std::string& address)
{
    std::istringstream lines(config);
    std::string line;
    std::string section;
    const std::string expected = "mac:" + address;
    while (std::getline(lines, line)) {
        line.erase(std::remove_if(line.begin(), line.end(),
                                  [](unsigned char value) {
                                      return std::isspace(value) != 0;
                                  }),
                   line.end());
        std::transform(line.begin(), line.end(), line.begin(),
                       [](unsigned char value) {
                           return static_cast<char>(std::tolower(value));
                       });
        if (line.empty() || line.front() == '#' || line.front() == ';') {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            section = line;
            continue;
        }
        if (section != "[keyfile]") {
            continue;
        }
        const std::string prefix = "unmanaged-devices=";
        if (line.compare(0, prefix.size(), prefix) != 0) {
            continue;
        }
        std::istringstream entries(line.substr(prefix.size()));
        std::string entry;
        while (std::getline(entries, entry, ';')) {
            if (entry == expected) {
                return true;
            }
        }
    }
    return false;
}

bool verify_ethercat_runtime(const Rk3588HostPaths& paths,
                             std::string& error)
{
    if (!verify_ethercat_platform_driver(paths, error)) {
        return false;
    }

    std::string configured_address;
    if (!configured_master_mac(paths, configured_address, error)) {
        return false;
    }

    std::string loaded_address;
    if (!read_text(std::filesystem::path(paths.sys_root) /
                       "module/ec_master/parameters/main_devices",
                   loaded_address,
                   error)) {
        return false;
    }
    std::transform(loaded_address.begin(), loaded_address.end(),
                   loaded_address.begin(), [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (!valid_mac_address(loaded_address)) {
        error = "invalid loaded ec_master main_devices: " + loaded_address;
        return false;
    }
    if (loaded_address != configured_address) {
        error = "loaded ec_master main_devices=" + loaded_address +
                " does not match configured value " + configured_address +
                "; reboot after install";
        return false;
    }

    std::string nm_config;
    const auto nm_path = std::filesystem::path(paths.etc_root) /
                         "NetworkManager/conf.d/99-ethercat-unmanaged.conf";
    if (!read_text(nm_path, nm_config, error)) {
        return false;
    }
    if (!networkmanager_unmanages_mac(nm_config, loaded_address)) {
        error = nm_path.string() + " does not mark EtherCAT MAC " +
                loaded_address + " unmanaged";
        return false;
    }
    return true;
}

}  // namespace

bool verify_rk3588_host_layout(std::string& error,
                               const Rk3588HostPaths& paths)
{
    error.clear();
    std::string online;
    if (!read_text(std::filesystem::path(paths.sys_root) /
                       "devices/system/cpu/online",
                   online,
                   error)) {
        return false;
    }
    std::set<int> online_cpus;
    if (!parse_cpu_list(online, online_cpus, error) ||
        !require_cpus(online_cpus, {0, 1, 2, 3, 4, 5, 6, 7},
                      "online CPU list", error)) {
        return false;
    }

    std::string isolated;
    if (!read_text(std::filesystem::path(paths.sys_root) /
                       "devices/system/cpu/isolated",
                   isolated,
                   error)) {
        return false;
    }
    std::set<int> isolated_cpus;
    if (!parse_cpu_list(isolated, isolated_cpus, error) ||
        isolated_cpus != std::set<int>({4, 5, 6, 7})) {
        if (error.empty()) {
            error = "isolated CPU list must be exactly CPUs 4-7";
        }
        return false;
    }

    std::string cmdline;
    if (!read_text(std::filesystem::path(paths.proc_root) / "cmdline",
                   cmdline,
                   error)) {
        return false;
    }
    if (cmdline.find("nohz_full=") != std::string::npos) {
        error = "nohz_full must not be configured for the periodic RT baseline";
        return false;
    }

    std::string value;
    std::set<int> parsed;
    if (!extract_cmdline_value(cmdline, "isolcpus", value) ||
        value.find("domain") == std::string::npos ||
        value.find("managed_irq") == std::string::npos ||
        !parse_cpu_list(value, parsed, error) ||
        parsed != std::set<int>({4, 5, 6, 7})) {
        if (error.empty()) {
            error = "isolcpus must include domain,managed_irq and CPUs 4-7";
        }
        return false;
    }
    if (!extract_cmdline_value(cmdline, "rcu_nocbs", value) ||
        !parse_cpu_list(value, parsed, error) ||
        parsed != std::set<int>({4, 5, 6, 7})) {
        if (error.empty()) {
            error = "rcu_nocbs must include CPUs 4-7";
        }
        return false;
    }
    if (!extract_cmdline_value(cmdline, "irqaffinity", value) ||
        !parse_cpu_list(value, parsed, error) ||
        parsed != std::set<int>({0, 1, 2, 3})) {
        if (error.empty()) {
            error = "irqaffinity must be exactly CPUs 0-3";
        }
        return false;
    }

    std::string workqueue_mask;
    if (!read_text(std::filesystem::path(paths.sys_root) /
                       "devices/virtual/workqueue/cpumask",
                   workqueue_mask,
                   error)) {
        return false;
    }
    workqueue_mask.erase(
        std::remove(workqueue_mask.begin(), workqueue_mask.end(), ','),
        workqueue_mask.end());
    try {
        std::size_t parsed_length = 0;
        const auto mask = std::stoull(workqueue_mask, &parsed_length, 16);
        if (parsed_length != workqueue_mask.size() || mask != 0x0FULL) {
            error = "unbound workqueue cpumask must be CPU0-3 (0f)";
            return false;
        }
    } catch (const std::exception&) {
        error = "invalid unbound workqueue cpumask: " + workqueue_mask;
        return false;
    }

    for (const int policy : {0, 4, 6}) {
        if (!verify_governor(paths, policy, error)) {
            return false;
        }
    }
    if (!verify_irq_affinity(paths, "can0", {2}, error) ||
        !verify_ethercat_runtime(paths, error)) {
        return false;
    }

    std::string ready;
    if (!read_text(std::filesystem::path(paths.run_root) /
                       "robot-rt-layout.ready",
                   ready,
                   error)) {
        return false;
    }
    std::string boot_id;
    if (!read_text(std::filesystem::path(paths.proc_root) /
                       "sys/kernel/random/boot_id",
                   boot_id,
                   error)) {
        return false;
    }
    if (ready.find("profile=" + paths.profile) == std::string::npos ||
        ready.find("boot_id=" + boot_id) == std::string::npos) {
        error = "robot RT ready marker is stale or has the wrong profile";
        return false;
    }

    if (!std::filesystem::exists(
            std::filesystem::path(paths.dev_root) / "EtherCAT0")) {
        error = "EtherCAT master device /dev/EtherCAT0 is unavailable";
        return false;
    }

    error.clear();
    return true;
}

}  // namespace inference
