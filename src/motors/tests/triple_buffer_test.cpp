/// @file triple_buffer_test.cpp
/// @brief LatestStatusChannel single-atomic middle-slot triple-buffer tests.

#include "motor_base/status_channel.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

namespace {

struct TestSnapshot {
    std::uint64_t sequence_begin{0};
    std::array<std::uint64_t, 32> values{};
    std::uint64_t sequence_end{0};
};

int g_failures = 0;

bool check(bool condition, const char* file, int line, const char* msg)
{
    if (!condition) {
        std::cerr << "[FAIL] " << file << ":" << line << " - "
                  << msg << "\n";
        ++g_failures;
    }
    return condition;
}

#define CHECK(cond, msg) check((cond), __FILE__, __LINE__, msg)

using Channel = motor_base::LatestStatusChannel<TestSnapshot>;

void fill_snapshot(TestSnapshot& snapshot, std::uint64_t frame_id)
{
    snapshot.sequence_begin = frame_id;
    for (auto& value : snapshot.values) {
        value = frame_id;
    }
    snapshot.sequence_end = frame_id;
}

bool publish_frame(Channel& channel,
                   std::uint64_t frame_id,
                   std::size_t motor_count = 1)
{
    Channel::WriteToken token;
    if (!CHECK(channel.write(token), "write should succeed")) {
        return false;
    }
    for (std::size_t i = 0; i < motor_count; ++i) {
        fill_snapshot(token.data[i], frame_id);
    }
    channel.publish(token);
    return true;
}

bool snapshot_is_complete(const TestSnapshot& snapshot)
{
    if (snapshot.sequence_begin != snapshot.sequence_end) {
        return false;
    }
    for (const auto value : snapshot.values) {
        if (value != snapshot.sequence_begin) {
            return false;
        }
    }
    return true;
}

bool expect_frame(const std::vector<TestSnapshot>& snapshots,
                  std::uint64_t expected)
{
    if (!CHECK(!snapshots.empty(), "consumer output should not be empty")) {
        return false;
    }
    CHECK(snapshots[0].sequence_begin == expected,
          "consumer should receive expected latest frame");
    CHECK(snapshot_is_complete(snapshots[0]),
          "snapshot should not be torn");
    return snapshots[0].sequence_begin == expected &&
           snapshot_is_complete(snapshots[0]);
}

void test_initial_state()
{
    std::cout << "=== Test 1: initial state ===\n";

    Channel channel;
    channel.configure(1, 1);

    std::vector<TestSnapshot> out;
    CHECK(!channel.copy_latest_status_for_test(out),
          "configure should start with clean Middle and no new data");
    CHECK(out.size() == 1, "output should be sized to motor_count");
}

void test_single_publish()
{
    std::cout << "=== Test 2: single publish ===\n";

    Channel channel;
    channel.configure(1, 1);

    CHECK(publish_frame(channel, 1), "frame1 should publish");

    std::vector<TestSnapshot> out;
    CHECK(channel.copy_latest_status_for_test(out),
          "consumer should receive frame1");
    expect_frame(out, 1);
    CHECK(!channel.copy_latest_status_for_test(out),
          "second consume should see no new frame");
    CHECK(channel.get_publish_count() == 1, "publish_count should be 1");
    CHECK(channel.get_overwritten_count() == 0,
          "first publish should not overwrite");
}

void test_latest_semantics()
{
    std::cout << "=== Test 3: latest semantics ===\n";

    Channel channel;
    channel.configure(1, 1);

    CHECK(publish_frame(channel, 1), "frame1 should publish");
    CHECK(publish_frame(channel, 2), "frame2 should publish");
    CHECK(publish_frame(channel, 3), "frame3 should publish");

    std::vector<TestSnapshot> out;
    CHECK(channel.copy_latest_status_for_test(out),
          "consumer should receive one latest frame");
    expect_frame(out, 3);
    CHECK(!channel.copy_latest_status_for_test(out),
          "intermediate frames should not queue");
    CHECK(channel.get_publish_count() == 3, "publish_count should be 3");
    CHECK(channel.get_overwritten_count() == 2,
          "frame1 and frame2 should be overwritten");
}

void test_consume_then_publish_again()
{
    std::cout << "=== Test 4: consume then publish again ===\n";

    Channel channel;
    channel.configure(1, 1);

    std::vector<TestSnapshot> out;

    CHECK(publish_frame(channel, 1), "frame1 should publish");
    CHECK(channel.copy_latest_status_for_test(out),
          "consumer should receive frame1");
    expect_frame(out, 1);

    CHECK(publish_frame(channel, 2), "frame2 should publish");
    CHECK(channel.copy_latest_status_for_test(out),
          "consumer should receive frame2");
    expect_frame(out, 2);

    CHECK(channel.get_publish_count() == 2, "publish_count should be 2");
    CHECK(channel.get_overwritten_count() == 0,
          "consumed frames should not count as overwritten");
}

void test_fast_producer_slow_consumer()
{
    std::cout << "=== Test 5: fast producer / slow consumer ===\n";

    constexpr std::uint64_t kTotalFrames = 100000;

    Channel channel;
    channel.configure(1, 1);

    std::atomic<bool> producer_done{false};
    std::atomic<bool> error{false};
    std::atomic<int> consumed{0};
    std::atomic<std::uint64_t> last_seen{0};

    std::thread consumer([&]() {
        std::vector<TestSnapshot> out;
        std::uint64_t previous = 0;
        while (!producer_done.load(std::memory_order_acquire)) {
            if (channel.copy_latest_status_for_test(out)) {
                const auto seq = out[0].sequence_begin;
                if (seq < previous || !snapshot_is_complete(out[0])) {
                    error.store(true, std::memory_order_relaxed);
                }
                previous = seq;
                last_seen.store(seq, std::memory_order_relaxed);
                consumed.fetch_add(1, std::memory_order_relaxed);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        for (int i = 0; i < 20; ++i) {
            if (channel.copy_latest_status_for_test(out)) {
                const auto seq = out[0].sequence_begin;
                if (seq < previous || !snapshot_is_complete(out[0])) {
                    error.store(true, std::memory_order_relaxed);
                }
                previous = seq;
                last_seen.store(seq, std::memory_order_relaxed);
                consumed.fetch_add(1, std::memory_order_relaxed);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    });

    std::thread producer([&]() {
        for (std::uint64_t frame = 1; frame <= kTotalFrames; ++frame) {
            if (!publish_frame(channel, frame)) {
                error.store(true, std::memory_order_relaxed);
                break;
            }
            if ((frame % 256) == 0) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    producer.join();
    consumer.join();

    CHECK(!error.load(std::memory_order_relaxed),
          "consumer should see monotonic complete snapshots");
    CHECK(channel.get_publish_count() == kTotalFrames,
          "producer should publish all frames");
    CHECK(consumed.load(std::memory_order_relaxed) > 0,
          "slow consumer should receive at least one latest frame");
    CHECK(consumed.load(std::memory_order_relaxed) <
              static_cast<int>(kTotalFrames / 10),
          "consumer should skip frames instead of draining a queue");

    std::cout << "  consumed=" << consumed.load(std::memory_order_relaxed)
              << " last_seen=" << last_seen.load(std::memory_order_relaxed)
              << " overwritten=" << channel.get_overwritten_count() << "\n";
}

void test_no_torn_read()
{
    std::cout << "=== Test 6: torn-read detection ===\n";

    constexpr std::uint64_t kTotalFrames = 200000;

    Channel channel;
    channel.configure(1, 1);

    std::atomic<bool> producer_done{false};
    std::atomic<bool> torn_detected{false};
    std::atomic<int> successful_reads{0};

    std::thread consumer([&]() {
        std::vector<TestSnapshot> out;
        while (!producer_done.load(std::memory_order_acquire)) {
            if (channel.copy_latest_status_for_test(out)) {
                if (!snapshot_is_complete(out[0])) {
                    torn_detected.store(true, std::memory_order_relaxed);
                }
                successful_reads.fetch_add(1, std::memory_order_relaxed);
            }
        }
        while (channel.copy_latest_status_for_test(out)) {
            if (!snapshot_is_complete(out[0])) {
                torn_detected.store(true, std::memory_order_relaxed);
            }
            successful_reads.fetch_add(1, std::memory_order_relaxed);
        }
    });

    std::thread producer([&]() {
        for (std::uint64_t frame = 1; frame <= kTotalFrames; ++frame) {
            if (!publish_frame(channel, frame)) {
                torn_detected.store(true, std::memory_order_relaxed);
                break;
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    producer.join();
    consumer.join();

    CHECK(!torn_detected.load(std::memory_order_relaxed),
          "no torn snapshot should be observed");
    CHECK(successful_reads.load(std::memory_order_relaxed) > 0,
          "consumer should observe published snapshots");

    std::cout << "  reads=" << successful_reads.load(std::memory_order_relaxed)
              << " published=" << channel.get_publish_count()
              << " overwritten=" << channel.get_overwritten_count() << "\n";
}

} // namespace

int main()
{
    test_initial_state();
    test_single_publish();
    test_latest_semantics();
    test_consume_then_publish_again();
    test_fast_producer_slow_consumer();
    test_no_torn_read();

    std::cout << "\n=============================\n";
    if (g_failures == 0) {
        std::cout << "ALL TESTS PASSED\n";
    } else {
        std::cout << g_failures << " TEST(S) FAILED\n";
    }
    std::cout << "=============================\n";

    return g_failures == 0 ? 0 : 1;
}
