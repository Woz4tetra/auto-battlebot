#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "serial_worker.hpp"

namespace auto_battlebot {
namespace {

TEST(SerialWorkerTest, RunsJobsInPostOrderOffTheCallersThread) {
    std::mutex mutex;
    std::vector<int> order;
    std::vector<std::thread::id> threads;
    SerialWorker worker;
    for (int i = 0; i < 50; ++i) {
        worker.post([&, i] {
            std::lock_guard<std::mutex> lock(mutex);
            order.push_back(i);
            threads.push_back(std::this_thread::get_id());
        });
    }
    worker.stop();
    ASSERT_EQ(order.size(), 50U);
    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(order[i], i);
        EXPECT_NE(threads[i], std::this_thread::get_id());
    }
}

TEST(SerialWorkerTest, PostReturnsWithoutWaitingForTheJob) {
    SerialWorker worker;
    std::atomic<bool> release{false};
    const auto start = std::chrono::steady_clock::now();
    worker.post([&] {
        while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });
    worker.post([] {});
    const double post_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    release.store(true);
    EXPECT_LT(post_ms, 50.0);
}

TEST(SerialWorkerTest, StopRunsWhatWasPostedAndDropsLaterPosts) {
    std::atomic<int> runs{0};
    SerialWorker worker;
    for (int i = 0; i < 5; ++i) {
        worker.post([&] {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            runs.fetch_add(1);
        });
    }
    worker.stop();
    EXPECT_EQ(runs.load(), 5);
    worker.post([&] { runs.fetch_add(1); });
    worker.stop();
    EXPECT_EQ(runs.load(), 5);
}

TEST(SerialWorkerTest, StopTwiceRunsNothingExtra) {
    std::atomic<int> runs{0};
    SerialWorker worker;
    worker.post([&] { runs.fetch_add(1); });
    worker.stop();
    worker.stop();
    EXPECT_EQ(runs.load(), 1);
}

struct PostsFromDestructor {
    SerialWorker *worker;
    std::atomic<int> *ran;
    ~PostsFromDestructor() {
        worker->post([ran = ran] { ran->fetch_add(1); });
    }
};

TEST(SerialWorkerTest, JobCapturesAreDestroyedOutsideTheWorkerLock) {
    auto worker = std::make_unique<SerialWorker>();
    std::atomic<int> ran{0};
    std::shared_ptr<PostsFromDestructor> capture(new PostsFromDestructor{worker.get(), &ran});
    worker->post([owned = std::move(capture)] {});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (ran.load() == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (ran.load() == 0) {
        worker.release();
        FAIL() << "worker deadlocked destroying a job that posts from its destructor";
    }
    worker->stop();
}

}  // namespace
}  // namespace auto_battlebot
