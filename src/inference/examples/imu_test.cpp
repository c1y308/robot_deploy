#include "config/deploy_config.hpp"
#include "robot/robot_imu_session.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#ifndef ROBOT_DEPLOY_CONFIG_PATH
#define ROBOT_DEPLOY_CONFIG_PATH ""
#endif

namespace {

constexpr auto kPollInterval = std::chrono::milliseconds(1);

std::atomic<bool> g_running{true};

void signal_handler(int)
{
    g_running.store(false);
}

struct IntervalStats {
    std::uint64_t count = 0;
    std::int64_t sum_ns = 0;
    std::int64_t min_ns = std::numeric_limits<std::int64_t>::max();
    std::int64_t max_ns = 0;

    void observe(std::int64_t interval_ns)
    {
        if (interval_ns <= 0) {
            return;
        }
        ++count;
        sum_ns += interval_ns;
        min_ns = std::min(min_ns, interval_ns);
        max_ns = std::max(max_ns, interval_ns);
    }

    void reset()
    {
        count = 0;
        sum_ns = 0;
        min_ns = std::numeric_limits<std::int64_t>::max();
        max_ns = 0;
    }
};

struct AhrsChannelStats {
    std::uint64_t total_frames = 0;
    std::uint64_t window_frames = 0;

    bool has_last_receive_timestamp = false;
    bool has_last_sample_timestamp = false;

    std::int64_t last_receive_timestamp_ns = 0;
    std::uint64_t last_sample_timestamp_ns = 0;

    bool has_latest = false;
    inference::AhrsStateSnapshot latest;

    IntervalStats receive_interval;
    IntervalStats sample_interval;

    bool observe_if_new(const inference::AhrsStateSnapshot& snapshot)
    {
        if (snapshot.receive_timestamp_ns <= 0) {
            return false;
        }
        if (has_last_receive_timestamp &&
            snapshot.receive_timestamp_ns == last_receive_timestamp_ns) {
            return false;
        }

        ++total_frames;
        ++window_frames;
        has_latest = true;
        latest = snapshot;

        if (has_last_receive_timestamp &&
            snapshot.receive_timestamp_ns > last_receive_timestamp_ns) {
            receive_interval.observe(snapshot.receive_timestamp_ns -
                                     last_receive_timestamp_ns);
        }
        has_last_receive_timestamp = true;
        last_receive_timestamp_ns = snapshot.receive_timestamp_ns;

        if (snapshot.sample_timestamp_ns > 0) {
            if (has_last_sample_timestamp &&
                snapshot.sample_timestamp_ns > last_sample_timestamp_ns) {
                const std::uint64_t interval_ns =
                    snapshot.sample_timestamp_ns - last_sample_timestamp_ns;
                if (interval_ns <=
                    static_cast<std::uint64_t>(
                        std::numeric_limits<std::int64_t>::max())) {
                    sample_interval.observe(static_cast<std::int64_t>(interval_ns));
                }
            }
            has_last_sample_timestamp = true;
            last_sample_timestamp_ns = snapshot.sample_timestamp_ns;
        }

        return true;
    }

    void reset_window()
    {
        window_frames = 0;
        receive_interval.reset();
        sample_interval.reset();
    }
};

enum class ParseResult {
    Ok,
    Help,
    Error,
};

const char* reader_type_name(imu_base::ReaderType type)
{
    switch (type) {
        case imu_base::ReaderType::A100_SERIAL:
            return "a100";
        case imu_base::ReaderType::XSENS_MTI_CAN:
            return "xsens";
    }
    return "unknown";
}

void print_usage(const char* program)
{
    std::cout
        << "Usage: " << program
        << " [--config deploy.yaml] [--type a100|xsens] [--device PATH]"
        << " [--baudrate N] [--can-bitrate N] [--no-configure-can]"
        << " [--report-ms N]\n";
}

ParseResult parse_args(int argc,
                       char** argv,
                       inference::ImuConfig& config,
                       std::string& config_path,
                       std::chrono::milliseconds& report_interval)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return ParseResult::Help;
        }
        if (arg == "--type" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "a100") {
                config.type = imu_base::ReaderType::A100_SERIAL;
            } else if (value == "xsens") {
                config.type = imu_base::ReaderType::XSENS_MTI_CAN;
            } else {
                std::cerr << "[IMU_TEST] Unknown IMU type: " << value << "\n";
                return ParseResult::Error;
            }
        } else if (arg == "--a100") {
            config.type = imu_base::ReaderType::A100_SERIAL;
        } else if (arg == "--xsens") {
            config.type = imu_base::ReaderType::XSENS_MTI_CAN;
        } else if (arg == "--device" && i + 1 < argc) {
            config.device = argv[++i];
        } else if (arg == "--baudrate" && i + 1 < argc) {
            config.baudrate = std::stoi(argv[++i]);
        } else if (arg == "--can-bitrate" && i + 1 < argc) {
            config.can_bitrate = std::stoi(argv[++i]);
            if (config.can_bitrate <= 0) {
                std::cerr << "[IMU_TEST] --can-bitrate must be positive\n";
                return ParseResult::Error;
            }
        } else if (arg == "--no-configure-can") {
            config.configure_can = false;
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--report-ms" && i + 1 < argc) {
            report_interval = std::chrono::milliseconds(std::stoi(argv[++i]));
            if (report_interval.count() <= 0) {
                std::cerr << "[IMU_TEST] --report-ms must be positive\n";
                return ParseResult::Error;
            }
        } else {
            std::cerr << "[IMU_TEST] Unknown or incomplete argument: "
                      << arg << "\n";
            print_usage(argv[0]);
            return ParseResult::Error;
        }
    }

    return ParseResult::Ok;
}

double interval_ms(std::int64_t interval_ns)
{
    return static_cast<double>(interval_ns) / 1'000'000.0;
}

void print_interval_stats(const char* label, const IntervalStats& stats)
{
    if (stats.count == 0) {
        std::cout << ' ' << label << "_dt_ms=n/a";
        return;
    }

    const double avg_ms =
        static_cast<double>(stats.sum_ns) /
        static_cast<double>(stats.count) / 1'000'000.0;
    std::cout << ' ' << label << "_dt_ms(avg/min/max)="
              << avg_ms << '/'
              << interval_ms(stats.min_ns) << '/'
              << interval_ms(stats.max_ns);
}

void print_ahrs_report(const AhrsChannelStats& stats, double elapsed_s)
{
    const double hz =
        elapsed_s > 0.0
            ? static_cast<double>(stats.window_frames) / elapsed_s
            : 0.0;
    std::cout << " ahrs_hz=" << hz
              << " ahrs_frames=" << stats.window_frames
              << " ahrs_total=" << stats.total_frames;
    print_interval_stats("ahrs_sample", stats.sample_interval);
    print_interval_stats("ahrs_receive", stats.receive_interval);
    if (stats.has_latest) {
        std::cout << std::setprecision(6)
                  << " euler_rad=["
                  << stats.latest.euler[0] << ','
                  << stats.latest.euler[1] << ','
                  << stats.latest.euler[2] << ']';
    } else {
        std::cout << " euler_rad=n/a";
    }
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    /* 先从 deploy.yaml 取 imu 配置，命令行参数可覆盖 */
    std::string config_path = ROBOT_DEPLOY_CONFIG_PATH;
    inference::RobotInterfaceConfig robot_cfg;
    std::string config_error;
    inference::ImuConfig cfg;
    std::chrono::milliseconds report_interval(1000);

    /* 先解析 --config，再加载配置，其余参数覆盖配置值 */
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--config") {
            config_path = argv[i + 1];
        }
    }
    if (!inference::load_deploy_config(config_path, robot_cfg, config_error)) {
        std::cerr << "[IMU_TEST] Failed to load deploy config: "
                  << config_error << "\n";
        return 1;
    }
    cfg = robot_cfg.imu;

    try {
        const ParseResult parse_result =
            parse_args(argc, argv, cfg, config_path, report_interval);
        if (parse_result == ParseResult::Help) {
            return 0;
        }
        if (parse_result == ParseResult::Error) {
            return 1;
        }
    } catch (const std::exception& error) {
        std::cerr << "[IMU_TEST] Argument parse failed: "
                  << error.what() << "\n";
        return 1;
    }

    inference::RobotImuSession imu(cfg);

    std::cout << "[IMU_TEST] Starting IMU channel test: type="
              << reader_type_name(cfg.type)
              << " device=" << cfg.device
              << " baudrate=" << cfg.baudrate
              << " configure_can="
              << (cfg.configure_can ? "true" : "false")
              << " can_bitrate=" << cfg.can_bitrate << "\n";
    if (!imu.initialize_and_start()) {
        std::cerr << "[IMU_TEST] Failed to start IMU." << std::endl;
        return -1;
    }

    std::cout << "[IMU_TEST] Polling AHRS channel every "
              << kPollInterval.count()
              << " ms and reporting every "
              << report_interval.count()
              << " ms. Press Ctrl+C to stop." << std::endl;

    using Clock = std::chrono::steady_clock;
    auto last_report = Clock::now();
    AhrsChannelStats stats;

    while (g_running.load()) {
        inference::AhrsStateSnapshot snapshot;
        if (imu.get_ahrs_snapshot(snapshot)) {
            stats.observe_if_new(snapshot);
        }

        std::this_thread::sleep_for(kPollInterval);

        const auto now = Clock::now();
        if (now - last_report < report_interval) {
            continue;
        }

        const double elapsed_s =
            std::chrono::duration<double>(now - last_report).count();
        last_report = now;

        std::cout << std::fixed << std::setprecision(3)
                  << "[IMU_TEST] window_s=" << elapsed_s;
        print_ahrs_report(stats, elapsed_s);
        std::cout << '\n';

        stats.reset_window();
    }

    imu.deinitialize();
    std::cout << "[IMU_TEST] IMU stopped." << std::endl;
    return 0;
}
