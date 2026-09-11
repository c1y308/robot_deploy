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

bool verify_threaded_napi(const Rk3588HostPaths& paths, std::string& error)
{
    const auto threaded_path = std::filesystem::path(paths.sys_root) /
                               "class/net/eth0/threaded";
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
        error = "unknown eth0 threaded NAPI state: " + state;
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
                if (comm != "napi/eth0" && comm.rfind("napi/eth0-", 0) != 0) {
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
        error = "eth0 reports threaded NAPI but no napi/eth0 thread was found";
        return false;
    }
    return true;
}

bool verify_ethercat_mac(const Rk3588HostPaths& paths, std::string& error)
{
    const auto address_path = std::filesystem::path(paths.sys_root) /
                              "class/net/eth0/address";
    if (!std::filesystem::exists(address_path)) {
        return true;
    }
    std::string address;
    if (!read_text(address_path, address, error)) {
        return false;
    }
    std::transform(address.begin(), address.end(), address.begin(),
                   [](unsigned char value) {
                       return static_cast<char>(std::tolower(value));
                   });

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
            if (token.compare(0, prefix.size(), prefix) == 0) {
                std::string configured = token.substr(prefix.size());
                std::transform(configured.begin(), configured.end(),
                               configured.begin(), [](unsigned char value) {
                                   return static_cast<char>(std::tolower(value));
                               });
                if (configured == address) {
                    return true;
                }
                error = "ec_master main_devices=" + configured +
                        " does not match eth0 MAC " + address;
                return false;
            }
        }
    }
    error = "ec_master main_devices is not configured";
    return false;
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
    if (!verify_irq_affinity(paths, "can0", {2}, error) ||
        !verify_irq_affinity(paths, "eth0", {3}, error) ||
        !verify_threaded_napi(paths, error) ||
        !verify_ethercat_mac(paths, error)) {
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
