// gui/calibration/calibration_event_tap.cpp

#include "calibration_event_tap.h"

#include <algorithm>
#include <utility>

#include "app/camera_controller.h"

namespace gui {

CalibrationEventTap::CalibrationEventTap(QObject* parent) : QObject(parent) {}

CalibrationEventTap::~CalibrationEventTap() {
    // Disconnect first so no new on_events_ready() calls start after this
    // returns. Then lock mutex_ to wait for any in-progress call (running on
    // the SDK thread via DirectConnection) to finish before buffer_ is
    // destroyed — prevents use-after-free.
    if (camera_) {
        disconnect(camera_, &CameraController::cd_events_ready,
                   this, &CalibrationEventTap::on_events_ready);
    }
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.clear();
    buffer_.shrink_to_fit();
}

void CalibrationEventTap::attach(CameraController* camera) {
    camera_ = camera;
    if (camera_) {
        // DirectConnection: on_events_ready() runs on the SDK streaming
        // thread (the emitter's thread), NOT the GUI thread. This avoids
        // posting batches to the GUI event queue, which would grow without
        // bound when the worker thread falls behind and the GUI thread stops
        // draining, OOM-killing the process. The slot is thread-safe
        // (mutex_ only, no GUI access).
        // UniqueConnection (audit §六-B1): the wizard re-attaches on every
        // set_camera() call; without it each wizard reopen would connect one
        // more copy of the slot and inject every event batch N times.
        connect(camera_, &CameraController::cd_events_ready,
                this, &CalibrationEventTap::on_events_ready,
                Qt::ConnectionType(Qt::DirectConnection | Qt::UniqueConnection));
    }
}

void CalibrationEventTap::on_events_ready(
    std::shared_ptr<std::vector<Metavision::EventCD>> events) {
    if (!events || events->empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.insert(buffer_.end(), events->begin(), events->end());
    // Cap: if the GUI thread was blocked (findChessboardCorners) and the
    // buffer grew past the limit, trim to the most recent kMaxBufferEvents.
    // Older events are from a previous 50 ms cycle and irrelevant for the
    // max-event 1 ms window pick. Build a tail copy and move-assign so the
    // old allocation is freed immediately (erase + shrink_to_fit would leave
    // the old capacity allocated until the next realloc).
    if (buffer_.size() > kMaxBufferEvents) {
        std::vector<Metavision::EventCD> trimmed(
            buffer_.end() - kMaxBufferEvents, buffer_.end());
        buffer_ = std::move(trimmed);
    }
}

std::size_t CalibrationEventTap::drain_and_pick_max_window(
    Metavision::timestamp window_us,
    Metavision::timestamp span_us,
    std::vector<Metavision::EventCD>& out) {
    out.clear();
    if (window_us <= 0 || span_us <= 0) return 0;

    std::vector<Metavision::EventCD> local;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) return 0;
        local.swap(buffer_);
    }

    // Events arrive SDK-sorted by timestamp. Instead of slicing
    // [t0, t0+span_us) into fixed window_us-wide sub-windows (which anchors
    // the grid to the buffer's first event and can permanently split the
    // 20 Hz flip burst across a boundary — audit §9.2-F), count events into
    // kBucketUs-wide fine buckets and take the maximum sliding sum of
    // window_us/kBucketUs consecutive buckets. This lets the optimal 1 ms
    // window start at any 100 µs offset, so a burst straddling any fixed
    // boundary is still captured whole. Complexity is O(N) + O(buckets).
    const Metavision::timestamp t0 = local.front().t;
    const Metavision::timestamp t_end = t0 + span_us;
    const int n_buckets = static_cast<int>(span_us / kBucketUs);
    const int win_buckets = std::max(1, static_cast<int>(window_us / kBucketUs));
    if (n_buckets <= 0) return 0;

    // Find the range of events within [t0, t_end).
    auto begin_it = local.begin();
    auto end_it = std::lower_bound(local.begin(), local.end(), t_end,
        [](const Metavision::EventCD& e, Metavision::timestamp t) {
            return e.t < t;
        });

    // Single pass: bucket the events in [t0, t_end).
    std::vector<std::size_t> buckets(static_cast<std::size_t>(n_buckets), 0);
    for (auto it = begin_it; it != end_it; ++it) {
        const Metavision::timestamp dt = it->t - t0;
        if (dt < 0) continue;
        const int b = static_cast<int>(dt / kBucketUs);
        if (b < 0 || b >= n_buckets) continue;
        ++buckets[static_cast<std::size_t>(b)];
    }

    // Sliding sum of win_buckets consecutive buckets; keep the max.
    std::size_t run = 0;
    for (int i = 0; i < std::min(win_buckets, n_buckets); ++i)
        run += buckets[static_cast<std::size_t>(i)];
    int best_start = 0;
    std::size_t best_count = run;
    for (int i = win_buckets; i < n_buckets; ++i) {
        run += buckets[static_cast<std::size_t>(i)];
        run -= buckets[static_cast<std::size_t>(i - win_buckets)];
        if (run > best_count) {
            best_count = run;
            best_start = i - win_buckets + 1;
        }
    }

    // Copy the best window's events into out.
    if (best_count > 0) {
        const Metavision::timestamp w_start = t0 + static_cast<Metavision::timestamp>(best_start) * kBucketUs;
        const Metavision::timestamp w_end = w_start + window_us;
        auto w_begin = std::lower_bound(local.begin(), local.end(), w_start,
            [](const Metavision::EventCD& e, Metavision::timestamp t) {
                return e.t < t;
            });
        auto w_end_it = std::lower_bound(local.begin(), local.end(), w_end,
            [](const Metavision::EventCD& e, Metavision::timestamp t) {
                return e.t < t;
            });
        out.assign(w_begin, w_end_it);
    }

    // Retain events at or after t_end for the next call. MUST be under the
    // mutex: on_events_ready() (running on the SDK thread via
    // DirectConnection) may have appended new events to buffer_ while we were
    // processing local. The retained events have earlier timestamps than any
    // new arrivals, so [retained, new] preserves sort order.
    //
    // Build a merged vector (retained ++ buffer_) and swap it in, rather than
    // buffer_.insert(begin, ...) which is O(buffer_.size()) due to shifting.
    // The swap frees the old allocation immediately.
    {
        std::lock_guard<std::mutex> lk(mutex_);
        std::vector<Metavision::EventCD> merged;
        merged.reserve(static_cast<std::size_t>(local.end() - end_it) + buffer_.size());
        merged.insert(merged.end(), end_it, local.end());
        merged.insert(merged.end(), buffer_.begin(), buffer_.end());
        buffer_.swap(merged);
    }
    return out.size();
}

void CalibrationEventTap::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.clear();
}

} // namespace gui