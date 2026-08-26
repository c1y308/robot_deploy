#include "base/spsc_latest_value.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

struct TestFrame {
    std::uint64_t sequence_begin{0};
    std::array<std::uint64_t, 32> values{};
    std::uint64_t sequence_end{0};
};

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << "\n";
        std::exit(1);
    }
}

void fill_frame(TestFrame& frame, std::uint64_t sequence)
{
    frame.sequence_begin = sequence;
    for (auto& value : frame.values) {
        value = sequence;
    }
    frame.sequence_end = sequence;
}

TestFrame make_frame(std::uint64_t sequence)
{
    TestFrame frame;
    fill_frame(frame, sequence);
    return frame;
}

bool frame_is_complete(const TestFrame& frame)
{
    if (frame.sequence_begin != frame.sequence_end) {
        return false;
    }
    for (const auto value : frame.values) {
        if (value != frame.sequence_begin) {
            return false;
        }
    }
    return true;
}

void test_initial_clean()
{
    robot_base::SpscLatestValue<TestFrame> channel;
    TestFrame out = make_frame(99);

    expect(!channel.try_consume_latest(out),
           "initial channel should have no latest value");
    expect(out.sequence_begin == 99 && out.sequence_end == 99,
           "failed consume must not modify output");
}

void test_single_publish_consume()
{
    robot_base::SpscLatestValue<TestFrame> channel;
    const bool overwritten = channel.publish(make_frame(1));
    TestFrame out;

    expect(!overwritten, "first publish should not overwrite dirty middle");
    expect(channel.try_consume_latest(out),
           "consumer should receive first publish");
    expect(out.sequence_begin == 1 && frame_is_complete(out),
           "consumer should receive complete first frame");
}

void test_latest_only()
{
    robot_base::SpscLatestValue<TestFrame> channel;
    channel.publish(make_frame(1));
    channel.publish(make_frame(2));
    channel.publish(make_frame(3));

    TestFrame out;
    expect(channel.try_consume_latest(out),
           "consumer should receive latest frame");
    expect(out.sequence_begin == 3 && frame_is_complete(out),
           "consumer should skip to latest frame");
    expect(!channel.try_consume_latest(out),
           "second consume without publish should return false");
    expect(out.sequence_begin == 3,
           "failed consume after latest should leave output unchanged");
}

void test_no_publish_never_reads_old_slot()
{
    robot_base::SpscLatestValue<TestFrame> channel;
    channel.publish(make_frame(7));

    TestFrame out = make_frame(0);
    expect(channel.try_consume_latest(out),
           "consumer should receive published frame");
    expect(out.sequence_begin == 7, "first consume should read frame 7");

    out = make_frame(1234);
    for (int i = 0; i < 64; ++i) {
        expect(!channel.try_consume_latest(out),
               "consume without new publish should return false");
        expect(out.sequence_begin == 1234 && out.sequence_end == 1234,
               "consume without new publish must not modify output");
    }
}

void test_reset_with_value()
{
    robot_base::SpscLatestValue<TestFrame> channel;
    channel.reset_with_value(make_frame(42));

    TestFrame out;
    expect(channel.try_consume_latest(out),
           "reset_with_value should expose one initial value");
    expect(out.sequence_begin == 42 && frame_is_complete(out),
           "reset value should be complete");
    expect(!channel.try_consume_latest(out),
           "reset value should be consumed once");
}

void test_fast_producer_slow_consumer()
{
    constexpr std::uint64_t kTotalFrames = 100000;

    robot_base::SpscLatestValue<TestFrame> channel;
    std::atomic<bool> producer_done{false};
    std::atomic<bool> error{false};
    std::atomic<int> consumed{0};

    std::thread consumer([&]() {
        TestFrame out;
        std::uint64_t previous = 0;
        while (!producer_done.load(std::memory_order_acquire)) {
            if (channel.try_consume_latest(out)) {
                if (!frame_is_complete(out) ||
                    out.sequence_begin < previous) {
                    error.store(true, std::memory_order_relaxed);
                }
                previous = out.sequence_begin;
                consumed.fetch_add(1, std::memory_order_relaxed);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        for (int i = 0; i < 32; ++i) {
            if (channel.try_consume_latest(out)) {
                if (!frame_is_complete(out) ||
                    out.sequence_begin < previous) {
                    error.store(true, std::memory_order_relaxed);
                }
                previous = out.sequence_begin;
                consumed.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    std::thread producer([&]() {
        for (std::uint64_t sequence = 1; sequence <= kTotalFrames; ++sequence) {
            channel.publish(make_frame(sequence));
            if ((sequence % 256) == 0) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    producer.join();
    consumer.join();

    expect(!error.load(std::memory_order_relaxed),
           "consumer should see complete monotonic frames");
    expect(consumed.load(std::memory_order_relaxed) > 0,
           "consumer should receive at least one frame");
    expect(consumed.load(std::memory_order_relaxed) <
               static_cast<int>(kTotalFrames / 10),
           "slow consumer should skip frames instead of draining a queue");
}

}  // namespace

int main()
{
    test_initial_clean();
    test_single_publish_consume();
    test_latest_only();
    test_no_publish_never_reads_old_slot();
    test_reset_with_value();
    test_fast_producer_slow_consumer();

    std::cout << "spsc_latest_value_test passed\n";
    return 0;
}
