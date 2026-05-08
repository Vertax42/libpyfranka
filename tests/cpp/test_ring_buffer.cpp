#include <gtest/gtest.h>
#include <array>
#include <atomic>
#include <thread>
#include <vector>

#include "realtime_control/shared_memory.hpp"

using franka_rt::RealTimeBuffer;

// ============================================================================
// Single-threaded sanity
// ============================================================================

TEST(RingBuffer, EmptyRead) {
    RealTimeBuffer<int, 8> buf;
    int x = 42;
    EXPECT_FALSE(buf.try_read(x));
    EXPECT_EQ(x, 42);  // unchanged
    EXPECT_TRUE(buf.empty());
}

TEST(RingBuffer, WriteThenRead) {
    RealTimeBuffer<int, 8> buf;
    EXPECT_TRUE(buf.try_write(7));
    int x = 0;
    EXPECT_TRUE(buf.try_read(x));
    EXPECT_EQ(x, 7);
    EXPECT_TRUE(buf.empty());
}

TEST(RingBuffer, FillToCapacityThenFail) {
    RealTimeBuffer<int, 4> buf;
    EXPECT_TRUE(buf.try_write(1));
    EXPECT_TRUE(buf.try_write(2));
    EXPECT_TRUE(buf.try_write(3));
    EXPECT_TRUE(buf.try_write(4));
    EXPECT_FALSE(buf.try_write(5));  // full
}

TEST(RingBuffer, FifoOrder) {
    RealTimeBuffer<int, 8> buf;
    for (int i = 0; i < 5; ++i) buf.try_write(i);
    for (int i = 0; i < 5; ++i) {
        int x = -1;
        EXPECT_TRUE(buf.try_read(x));
        EXPECT_EQ(x, i);
    }
}

TEST(RingBuffer, WrapAroundManyTimes) {
    RealTimeBuffer<int, 4> buf;
    int next_to_write = 0, next_to_read = 0;
    for (int round = 0; round < 1000; ++round) {
        // write 3, read 3 — never fills
        for (int j = 0; j < 3; ++j) {
            EXPECT_TRUE(buf.try_write(next_to_write++));
        }
        for (int j = 0; j < 3; ++j) {
            int x = -1;
            EXPECT_TRUE(buf.try_read(x));
            EXPECT_EQ(x, next_to_read++);
        }
    }
}

TEST(RingBuffer, ClearResets) {
    RealTimeBuffer<int, 8> buf;
    for (int i = 0; i < 5; ++i) buf.try_write(i);
    buf.clear();
    EXPECT_TRUE(buf.empty());
    int x = 0;
    EXPECT_FALSE(buf.try_read(x));
}

// ============================================================================
// Pose-sized struct (typical use case)
// ============================================================================

struct Cmd {
    std::array<double, 16> T;
    uint64_t seq;
};

TEST(RingBuffer, StructValueSemantics) {
    RealTimeBuffer<Cmd, 8> buf;
    Cmd a{};
    a.T[12] = 0.5; a.T[13] = -0.1; a.T[14] = 0.3; a.seq = 42;
    EXPECT_TRUE(buf.try_write(a));
    Cmd b{};
    EXPECT_TRUE(buf.try_read(b));
    EXPECT_EQ(b.seq, 42u);
    EXPECT_EQ(b.T[12], 0.5);
    EXPECT_EQ(b.T[14], 0.3);
}

// ============================================================================
// Producer/consumer concurrency stress
// ============================================================================

TEST(RingBuffer, SpscStressNoLoss) {
    constexpr int N_ITEMS = 200000;
    RealTimeBuffer<int, 256> buf;
    std::atomic<bool> producer_done{false};
    std::vector<int> received;
    received.reserve(N_ITEMS);

    std::thread producer([&] {
        for (int i = 0; i < N_ITEMS; ++i) {
            while (!buf.try_write(i)) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true);
    });

    std::thread consumer([&] {
        int last_seen = -1;
        while (true) {
            int x;
            if (buf.try_read(x)) {
                EXPECT_EQ(x, last_seen + 1);  // FIFO order maintained
                last_seen = x;
                received.push_back(x);
                if (last_seen == N_ITEMS - 1) break;
            } else if (producer_done.load() && buf.empty()) {
                break;
            }
        }
    });

    producer.join();
    consumer.join();
    EXPECT_EQ(received.size(), static_cast<size_t>(N_ITEMS));
}

TEST(RingBuffer, ConsumerSeesLatestNotJustOne) {
    // Pattern used by RT loop: pop ALL pending, keep last.
    RealTimeBuffer<int, 8> buf;
    for (int i = 0; i < 5; ++i) buf.try_write(i);
    int x = -1;
    int last = -1;
    while (buf.try_read(x)) last = x;
    EXPECT_EQ(last, 4);
    EXPECT_TRUE(buf.empty());
}
