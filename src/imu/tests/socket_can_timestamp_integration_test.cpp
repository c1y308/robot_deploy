#include "driver/socket_can_port.hpp"
#include "protocol/xsens_mti/can_parser.hpp"
#include "tool/tool.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

void expect(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

}  // namespace

// Explicit, passive hardware test: never configures the interface or transmits.
// Deliberately excluded from CTest because it requires a connected CAN device.
int main(int argc, char** argv)
{
    if (argc != 3 || std::string(argv[1]) != "--interface") {
        std::cerr << "Usage: " << argv[0] << " --interface can0\n";
        return 2;
    }
    try {
        imu::SocketCanPort port;
        expect(port.open(argv[2]), "cannot open the requested CAN interface");
        expect(port.wait_readable(2000) == 1, "no incoming CAN frames");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        can_frame frame{};
        std::int64_t received_ns = 0;
        expect(port.read_nonblocking(frame, &received_ns) == 1,
               "failed to receive the first queued frame");
        const auto backlog_age_ns = robot_base::monotonic_now_ns() - received_ns;
        expect(backlog_age_ns >= 95'000'000,
               "first queued frame lost its kernel RX age (expected >=95ms)");
        std::cout << "backlog_first_frame_age_ms=" << backlog_age_ns / 1.0e6 << '\n';

        // Discard the test backlog before measuring continuous reception.
        int result;
        do {
            result = port.read_nonblocking(frame, &received_ns);
        } while (result == 1);
        expect(result == 0, "backlog drain failed");

        imu::XsensMtiCanParser parser;
        std::uint64_t ahrs_count = 0, quaternion_count = 0, rate_count = 0, frame_count = 0;
        std::int64_t max_receive_duration_ns = 0, total_receive_duration_ns = 0;
        parser.set_ahrs_callback([&](const imu_base::AHRSData& data) {
            expect(data.receive_timestamp_ns > 0 && data.projected_gravity_valid,
                   "invalid AHRS snapshot during continuous reception");
            ++ahrs_count;
        });
        const auto start_ns = robot_base::monotonic_now_ns();
        while (robot_base::monotonic_now_ns() - start_ns < 5'000'000'000LL) {
            const int ready = port.wait_readable(100);
            expect(ready >= 0, "CAN poll failed during continuous reception");
            if (ready == 0) continue;
            do {
                const auto begin_ns = robot_base::monotonic_now_ns();
                result = port.read_nonblocking(frame, &received_ns);
                const auto duration_ns = robot_base::monotonic_now_ns() - begin_ns;
                expect(result >= 0, "CAN timestamp mapping failed during continuous reception");
                if (result == 1) {
                    ++frame_count;
                    total_receive_duration_ns += duration_ns;
                    if (duration_ns > max_receive_duration_ns) max_receive_duration_ns = duration_ns;
                    if ((frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) == 0) {
                        const auto id = frame.can_id & CAN_SFF_MASK;
                        if (id == imu::XCDI_QUATERNION_ID) ++quaternion_count;
                        if (id == imu::XCDI_RATE_OF_TURN_ID) ++rate_count;
                        parser.feed(id, frame.data, frame.can_dlc, received_ns);
                    }
                }
            } while (result == 1);
        }
        const double seconds = (robot_base::monotonic_now_ns() - start_ns) / 1.0e9;
        expect(ahrs_count > 0 && parser.get_info().error_frames == 0,
               "no valid AHRS publications or parser errors on the connected device");
        std::cout << "continuous_seconds=" << seconds
                  << " frames=" << frame_count
                  << " quaternion_hz=" << quaternion_count / seconds
                  << " rate_hz=" << rate_count / seconds
                  << " ahrs_hz=" << ahrs_count / seconds
                  << " average_recv_us=" << total_receive_duration_ns / (1.0e3 * frame_count)
                  << " max_recv_us=" << max_receive_duration_ns / 1.0e3
                  << " parser_errors=" << parser.get_info().error_frames << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[SOCKET_CAN_TIMESTAMP_INTEGRATION_TEST] " << error.what() << '\n';
        return 1;
    }
}
