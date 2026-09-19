// gui/davis/batch_worker.h — decouples event-batch processing from the USB
// reaping thread.
//
// The reference stacks (libcaer/DV) keep their USB data thread minimal: it
// parses packets and hands them to a ringbuffer; noise filtering and every
// consumer run on separate module threads. Our port ran the WHOLE
// per-batch pipeline (statistics, raw tap, conditioner, algorithm
// listeners, display push) inline on the libusb event thread — under a
// motion-driven event flood that thread saturates, subsequent transfers
// (with the interleaved IMU/APS words) sit unreaped in the kernel queue
// for seconds, and EVERYTHING lags: the IMU pose visibly trails the
// camera and closed-path returns are judged against a stale attitude
// (hardware-observed on the DAVIS346; the same class as the documented
// 2026-08-16 wizard USB-flood lesson).
//
// Usage: the USB thread decodes into a parser batch, swaps it into a
// pooled slot and submits it; the worker thread invokes the sink in FIFO
// order. Slot vectors are recycled through a free pool so the steady
// state does no allocation. Stop() drains what was submitted and joins.
//
// Header-only; unit-testable without any device.

#ifndef GUI_DAVIS_BATCH_WORKER_H
#define GUI_DAVIS_BATCH_WORKER_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

namespace gui::davis {

class BatchWorker {
public:
    using Batch = std::vector<Metavision::EventCD>;
    /// Same signature as the parser's EventSink — the span is only valid
    /// for the duration of the call.
    using Sink =
        std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;

    /// Starts the worker thread. @p sink_provider is read on EVERY batch
    /// (so the sink may be installed or replaced after start — the devices
    /// install it after construction, before streaming starts).
    void start(std::function<Sink()> sink_provider) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (run_.load()) return;
        sink_provider_ = std::move(sink_provider);
        run_.store(true);
        thread_ = std::thread([this]() {
            std::unique_lock<std::mutex> lock(mutex_);
            while (true) {
                cv_.wait(lock, [this]() {
                    return !queue_.empty() || !run_.load();
                });
                while (!queue_.empty()) {
                    auto batch = std::move(queue_.front());
                    queue_.pop_front();
                    // Copy the provider under the lock (it may be swapped
                    // by a concurrent install — read once, call unlocked:
                    // the sink itself may submit-free operations that take
                    // time, and holding the lock would block the USB
                    // thread's submits).
                    Sink sink = sink_provider_ ? sink_provider_() : nullptr;
                    lock.unlock();
                    if (sink && !batch->empty()) {
                        sink(batch->data(), batch->data() + batch->size());
                    }
                    batch->clear();
                    lock.lock();
                    pool_.push_back(std::move(batch));
                }
                if (!run_.load() && queue_.empty()) return;
            }
        });
    }

    /// Drains the submitted batches (invoking the sink) and joins. No new
    /// submissions may arrive once stop() is entered — callers stop their
    /// producer thread first.
    void stop() {
        std::thread worker;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!run_.load() && !thread_.joinable()) return;
            run_.store(false);
            worker = std::move(thread_);
        }
        cv_.notify_all();
        if (worker.joinable()) worker.join();
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        pool_.clear();
        sink_provider_ = nullptr;
    }

    /// A slot for the producer to decode into — recycled from the pool
    /// when the worker has spare capacity, freshly allocated otherwise.
    [[nodiscard]] std::unique_ptr<Batch> acquire() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pool_.empty()) return std::make_unique<Batch>();
        auto batch = std::move(pool_.back());
        pool_.pop_back();
        return batch;
    }

    /// Hands a filled slot to the worker (FIFO). An empty batch is
    /// recycled directly without reaching the sink — matching both
    /// parsers' existing `!batch_.empty()` emit guard.
    void submit(std::unique_ptr<Batch> batch) {
        bool empty = batch->empty();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (empty) {
                pool_.push_back(std::move(batch));
                return;
            }
            queue_.push_back(std::move(batch));
        }
        if (!empty) cv_.notify_one();
    }

private:
    std::thread thread_;
    std::function<Sink()> sink_provider_;
    std::deque<std::unique_ptr<Batch>> queue_;
    std::vector<std::unique_ptr<Batch>> pool_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> run_{false};
};

} // namespace gui::davis

#endif // GUI_DAVIS_BATCH_WORKER_H
