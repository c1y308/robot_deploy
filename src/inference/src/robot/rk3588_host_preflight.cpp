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

bool resolve_ethercat_interface(const Rk3588HostPaths& paths,
                                std::string& interface_name,
                                std::string& address,
                                std::string& error)
{
    interface_name.clear();
    const auto net_root = std::filesystem::path(paths.sys_root) / "class/net";
    try {
        for (const auto& entry : std::filesystem::directory_iterator(net_root)) {
            const auto device_link = entry.path() / "device";
            if (!std::filesystem::exists(device_link)) {
                continue;
            }
            std::error_code canonical_error;
            const auto device_path =
                std::filesystem::canonical(device_link, canonical_error);
            if (canonical_error ||
                device_path.filename() != paths.ethercat_device_id) {
                continue;
            }
            if (!interface_name.empty()) {
                error = "multiple netdevs map to EtherCAT device " +
                        paths.ethercat_device_id + ": " + interface_name +
                        ", " + entry.path().filename().string();
                return false;
            }
            interface_name = entry.path().filename().string();
        }
    } catch (const std::exception& exception) {
        error = "failed to resolve EtherCAT device " +
                paths.ethercat_device_id + ": " + exception.what();
        return false;
    }

    if (interface_name.empty()) {
        error = "no netdev maps to EtherCAT device " +
                paths.ethercat_device_id;
        return false;
    }

    const auto interface_root = net_root / interface_name;
    if (!read_text(interface_root / "address", address, error)) {
        return false;
    }
    std::transform(address.begin(), address.end(), address.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });
    if (!valid_mac_address(address)) {
        error = "invalid MAC for EtherCAT device " +
                paths.ethercat_device_id + " (" + interface_name + "): " +
                address;
        return false;
    }

    const auto assign_type_path = interface_root / "addr_assign_type";
    if (std::filesystem::exists(assign_type_path)) {
        std::string assign_type;
        if (!read_text(assign_type_path, assign_type, error)) {
            return false;
        }
        if (assign_type != "0") {
            error = "EtherCAT device " + paths.ethercat_device_id + " (" +
                    interface_name + ") has non-permanent MAC type " +
                    assign_type;
            return false;
        }
    }
    return true;
}

bool verify_threaded_napi(const Rk3588HostPaths& paths,
                          const std::string& interface_name,
                          std::string& error)
{
    const auto threaded_path = std::filesystem::path(paths.sys_root) /
                               "class/net" / interface_name / "threaded";
    if (!std::filesystem::exists(threaded_path)) {
        return true;
    }
    std::string state;
    if (!read_text(threaded_path, state, error)) {
        return false;
    }
    if (state == "0" || state == "N" || state == "n" || state == "no" ||
        state == "false") {
        return true;
    }
    if (state != "1" && state != "Y" && state != "y" && state != "yes" &&
        state != "true") {
        error = "unknown " + interface_name + " threaded NAPI state: " + state;
        return false;
    }

    bool found = false;
    try {
        for (const auto& process :
             std::filesystem::directory_iterator(paths.proc_root)) {
            const std::string pid = process.path().filename().string();
            if (pid.empty() || !std::all_of(
                                   pid.begin(), pid.end(), [](unsigned char value) {
                                       return std::isdigit(value) != 0;
                                   })) {
                continue;
            }
            const auto task_root = process.path() / "task";
            if (!std::filesystem::exists(task_root)) {
                continue;
            }
            for (const auto& task : std::filesystem::directory_iterator(task_root)) {
                std::string comm;
                if (!read_text(task.path() / "comm", comm, error)) {
                    continue;
                }
                const std::string napi_name = "napi/" + interface_name;
                if (comm != napi_name && comm.rfind(napi_name + "-", 0) != 0) {
                    continue;
                }
                found = true;
                std::string status;
                if (!read_text(task.path() / "status", status, error)) {
                    return false;
                }
                std::istringstream lines(status);
                std::string line;
                std::string affinity;
                while (std::getline(lines, line)) {
                    const std::string prefix = "Cpus_allowed_list:";
                    if (line.compare(0, prefix.size(), prefix) == 0) {
                        affinity = line.substr(prefix.size());
                        affinity.erase(
                            std::remove_if(affinity.begin(), affinity.end(),
                                           [](unsigned char value) {
                                               return std::isspace(value) != 0;
                                           }),
                            affinity.end());
                        break;
                    }
                }
                std::set<int> actual;
                if (affinity.empty() || !parse_cpu_list(affinity, actual, error) ||
                    actual != std::set<int>({3})) {
                    if (error.empty()) {
                        error = "threaded NAPI " + comm + " affinity is " +
                                affinity + ", expected CPU3";
                    }
                    return false;
                }
            }
        }
    } catch (const std::exception& exception) {
        error = std::string("failed to inspect threaded NAPI: ") +
                exception.what();
        return false;
    }
    if (!found) {
        error = interface_name + " reports threaded NAPI but no napi/" +
                interface_name + " thread was found";
        return false;
    }
    return true;
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

bool verify_ethercat_configuration(const Rk3588HostPaths& paths,
                                   const std::string& interface_name,
                                   const std::string& address,
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
    bool configured_matches = false;
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
            if (token.compare(0, prefix.size(), prefix) == 0) {
                std::string configured = token.substr(prefix.size());
                std::transform(configured.begin(), configured.end(),
                               configured.begin(), [](unsigned char value) {
                                   return static_cast<char>(std::tolower(value));
                               });
                if (configured != address) {
                    error = "ec_master main_devices=" + configured +
                            " does not match EtherCAT device " +
                            paths.ethercat_device_id + " (" + interface_name +
                            ") MAC " + address;
                    return false;
                }
                configured_matches = true;
            }
        }
    }
    if (!configured_matches) {
        error = "ec_master main_devices is not configured";
        return false;
    }

    const auto loaded_mac_path = std::filesystem::path(paths.sys_root) /
                                 "module/ec_master/parameters/main_devices";
    if (std::filesystem::exists(loaded_mac_path)) {
        std::string loaded_address;
        if (!read_text(loaded_mac_path, loaded_address, error)) {
            return false;
        }
        std::transform(loaded_address.begin(), loaded_address.end(),
                       loaded_address.begin(), [](unsigned char value) {
                           return static_cast<char>(std::tolower(value));
                       });
        if (loaded_address != address) {
            error = "loaded ec_master main_devices=" + loaded_address +
                    " does not match EtherCAT device " +
                    paths.ethercat_device_id + " (" + interface_name +
                    ") MAC " + address + "; reboot after install";
            return false;
        }
    }

    std::string nm_config;
    const auto nm_path = std::filesystem::path(paths.etc_root) /
                         "NetworkManager/conf.d/99-ethercat-unmanaged.conf";
    if (!read_text(nm_path, nm_config, error)) {
        return false;
    }
    if (!networkmanager_unmanages_mac(nm_config, address)) {
        error = nm_path.string() + " does not mark EtherCAT device " +
                paths.ethercat_device_id + " (" + interface_name + ", " +
                address + ") unmanaged";
        return false;
    }

    std::string nm_dropin;
    const auto nm_dropin_path =
        std::filesystem::path(paths.etc_root) /
        "systemd/system/NetworkManager.service.d/robot-ethercat-guard.conf";
    if (!read_text(nm_dropin_path, nm_dropin, error)) {
        return false;
    }
    const std::string guard_command =
        "ExecStartPre=/usr/local/sbin/robot-rt-setup check-nm-guard";
    bool guard_present = false;
    std::istringstream dropin_lines(nm_dropin);
    std::string dropin_line;
    while (std::getline(dropin_lines, dropin_line)) {
        dropin_line.erase(
            std::remove_if(dropin_line.begin(), dropin_line.end(),
                           [](unsigned char value) {
                               return value == '\r';
                           }),
            dropin_line.end());
        if (dropin_line == guard_command) {
            guard_present = true;
        }
    }
    if (!guard_present) {
        error = "NetworkManager guard drop-in is invalid: " +
                nm_dropin_path.string();
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
        isolated_cpus != std::set<int>({6, 7})) {
        if (error.empty()) {
            error = "isolated CPU list must be exactly CPUs 6-7";
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
        parsed != std::set<int>({6, 7})) {
        if (error.empty()) {
            error = "isolcpus must include domain,managed_irq and CPUs 6-7";
        }
        return false;
    }
    if (!extract_cmdline_value(cmdline, "rcu_nocbs", value) ||
        !parse_cpu_list(value, parsed, error) ||
        parsed != std::set<int>({6, 7})) {
        if (error.empty()) {
            error = "rcu_nocbs must include CPUs 6-7";
        }
        return false;
    }
    if (!extract_cmdline_value(cmdline, "irqaffinity", value) ||
        !parse_cpu_list(value, parsed, error) ||
        parsed != std::set<int>({0, 1, 2, 3, 4, 5})) {
        if (error.empty()) {
            error = "irqaffinity must be exactly CPUs 0-5";
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
        if (parsed_length != workqueue_mask.size() || mask != 0x3FULL) {
            error = "unbound workqueue cpumask must be CPU0-5 (3f)";
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
    std::string ethercat_interface;
    std::string ethercat_address;
    if (!resolve_ethercat_interface(paths,
                                    ethercat_interface,
                                    ethercat_address,
                                    error)) {
        return false;
    }
    if (!verify_irq_affinity(paths, "can0", {2}, error) ||
        !verify_irq_affinity(paths, ethercat_interface, {3}, error) ||
        !verify_threaded_napi(paths, ethercat_interface, error) ||
        !verify_ethercat_configuration(paths,
                                       ethercat_interface,
                                       ethercat_address,
                                       error)) {
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
    if (ready.find("profile=rk3588-v1") == std::string::npos ||
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
