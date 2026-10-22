// Offline benchmark: uses the real PolicyRuntime, never initializes hardware.
#include "config/deploy_config.hpp"
#include "policy/policy_runtime.hpp"
#include "robot/robot_config.hpp"
#include <ATen/Context.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

std::vector<pid_t> task_ids()
{
    std::vector<pid_t> result;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
        result.push_back(std::stoi(entry.path().filename().string()));
    }
    std::sort(result.begin(), result.end());
    return result;
}

void print_tasks(const char* phase)
{
    for (const auto tid : task_ids()) {
        const std::string base = "/proc/self/task/" + std::to_string(tid);
        std::ifstream stat(base + "/stat");
        std::string line;
        // Backend worker threads can exit between directory enumeration and read.
        if (!std::getline(stat, line)) continue;
        std::istringstream fields(line.substr(line.rfind(')') + 2));
        std::string value;
        long last_cpu = -1;
        for (int field = 3; field <= 39 && fields >> value; ++field) {
            if (field == 39) last_cpu = std::stol(value);
        }
        std::ifstream schedstat(base + "/schedstat");
        std::uint64_t run = 0, wait = 0, slices = 0;
        schedstat >> run >> wait >> slices;
        cpu_set_t mask;
        CPU_ZERO(&mask);
        if (sched_getaffinity(tid, sizeof(mask), &mask) != 0) {
            if (errno == ESRCH) continue;
            throw std::runtime_error("cannot read task affinity");
        }
        std::cout << "TASK " << phase << " tid=" << tid << " last_cpu=" << last_cpu
                  << " run_ns=" << run << " wait_ns=" << wait << " slices=" << slices
                  << " affinity=";
        for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
            if (CPU_ISSET(cpu, &mask)) std::cout << cpu << ',';
        }
        std::cout << '\n';
    }
}

std::int64_t process_cpu_ns()
{
    timespec ts{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) != 0) {
        throw std::runtime_error("cannot read process CPU time");
    }
    return ts.tv_sec * 1'000'000'000LL + ts.tv_nsec;
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        std::string config_path = ROBOT_DEPLOY_CONFIG_PATH;
        int threads = 2, iterations = 500;
        bool pin_workers = false;
        bool disable_mkldnn = false;
        double period_ms = 0.0;
        for (int i = 1; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--pin-workers") {
                pin_workers = true;
            } else if (option == "--no-mkldnn") {
                disable_mkldnn = true;
            } else if (option == "--help") {
                std::cout << "policy_benchmark [--config PATH] [--threads 1|2] "
                             "[--iterations N] [--period-ms N] [--pin-workers] [--no-mkldnn]\n";
                return 0;
            } else if (i + 1 < argc) {
                const std::string value = argv[++i];
                if (option == "--config") config_path = value;
                else if (option == "--threads") threads = std::stoi(value);
                else if (option == "--iterations") iterations = std::stoi(value);
                else if (option == "--period-ms") period_ms = std::stod(value);
                else throw std::runtime_error("unknown option: " + option);
            } else {
                throw std::runtime_error("missing value for " + option);
            }
        }
        if ((threads != 1 && threads != 2) || iterations <= 0 ||
            !std::isfinite(period_ms) || period_ms < 0) {
            throw std::runtime_error("threads must be 1/2, iterations positive, period-ms nonnegative");
        }

        for (const char* name : {"OMP_WAIT_POLICY", "GOMP_SPINCOUNT", "OMP_NUM_THREADS",
                                 "OPENBLAS_NUM_THREADS"}) {
            const char* value = std::getenv(name);
            std::cout << "ENV " << name << '=' << (value ? value : "<unset>") << '\n';
        }
        inference::RobotInterfaceConfig config;
        std::string error;
        if (!inference::load_deploy_config(config_path, config, error)) {
            throw std::runtime_error(error);
        }
        const auto profile = inference::make_rk3588_runtime_profile();
        const auto setup = robot_base::configure_current_thread("policy_bench", profile.policy_main);
        if (!setup.success) throw std::runtime_error(setup.error);
        if (disable_mkldnn) at::globalContext().setUserEnabledMkldnn(false);
        inference::PolicyRuntime runtime;
        if (!runtime.load(config.policy, threads, 1, 1, profile.policy_main.cpu_ids)) {
            throw std::runtime_error(runtime.last_error());
        }

        std::cout << "MODEL " << config.policy.model_path << " threads=" << threads
                  << " observations=" << runtime.observation_size() << " period_ms=" << period_ms
                  << " pin_workers=" << pin_workers
                  << " mkldnn=" << at::globalContext().userEnabledMkldnn() << '\n';
        print_tasks("loaded");
        if (pin_workers) {
            for (const auto tid : task_ids()) {
                cpu_set_t mask;
                CPU_ZERO(&mask);
                CPU_SET(profile.policy_main.cpu_ids[tid == gettid() ? 0 : 1], &mask);
                if (sched_setaffinity(tid, sizeof(mask), &mask) != 0) {
                    if (errno == ESRCH) continue;
                    throw std::runtime_error("cannot pin benchmark task");
                }
            }
            print_tasks("pinned");
        }

        inference::PolicyObservation observation{};
        // Finite deterministic nonzero inputs, not all-zero special cases.
        for (std::size_t i = 0; i < runtime.observation_size(); ++i) {
            observation[i] = static_cast<float>(std::sin(i * 0.13) * 0.1);
        }
        inference::PolicyRuntimeStepResult result;
        for (int i = 0; i < 20; ++i) {
            if (!runtime.infer(observation, result)) throw std::runtime_error(runtime.last_error());
        }
        print_tasks("before");
        std::vector<std::int64_t> durations;
        durations.reserve(iterations);
        using Clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(period_ms));
        auto next_tick = Clock::now();
        const auto begin = next_tick;
        const auto cpu_begin = process_cpu_ns();
        for (int i = 0; i < iterations; ++i) {
            next_tick += period;
            if (!runtime.infer(observation, result)) throw std::runtime_error(runtime.last_error());
            durations.push_back(result.inference_end_ns - result.inference_start_ns);
            if (period_ms > 0) {
                std::this_thread::sleep_until(next_tick);
                if (Clock::now() > next_tick + period) next_tick = Clock::now();
            }
        }
        const auto cpu_elapsed = process_cpu_ns() - cpu_begin;
        const double wall_s = std::chrono::duration<double>(Clock::now() - begin).count();
        print_tasks("after");
        std::sort(durations.begin(), durations.end());
        const double mean_ms = std::accumulate(durations.begin(), durations.end(), 0.0) /
                               iterations / 1e6;
        std::cout << std::fixed << std::setprecision(6)
                  << "RESULT iterations=" << iterations << " wall_s=" << wall_s
                  << " hz=" << iterations / wall_s << " mean_ms=" << mean_ms
                  << " p50_ms=" << durations[(iterations * 50 + 99) / 100 - 1] / 1e6
                  << " p99_ms=" << durations[(iterations * 99 + 99) / 100 - 1] / 1e6
                  << " max_ms=" << durations.back() / 1e6
                  << " over_20ms=" << std::count_if(durations.begin(), durations.end(),
                        [](auto value) { return value > 20'000'000; })
                  << " process_cpu_s=" << cpu_elapsed / 1e9
                  << " cpu_per_wall=" << cpu_elapsed / 1e9 / wall_s << '\n';
        std::cout << std::setprecision(9) << "OUTPUT";
        for (const auto value : result.raw_action) std::cout << ' ' << value;
        std::cout << '\n';
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
