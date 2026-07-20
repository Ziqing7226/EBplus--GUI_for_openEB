// gui/app/file_frame_generator.cpp

#include "file_frame_generator.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "algo_bridge/filter_chain.h"

namespace gui {

FileFrameGenerator::FileFrameGenerator(QObject* parent) : QObject(parent) {
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, &FileFrameGenerator::on_timer);
}

FileFrameGenerator::~FileFrameGenerator() {
    timer_.stop();
}

void FileFrameGenerator::add_events(const Metavision::EventCD* begin,
                                    const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    bool just_truncated = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // OOM guard (audit §六-C2): never let the buffer grow past
        // kMaxBufferedEvents. A batch that would cross the cap is only
        // appended up to the cap; the rest is dropped and reported once.
        const std::size_t room = kMaxBufferedEvents - events_.size();
        const std::size_t n = static_cast<std::size_t>(end - begin);
        if (room > 0) {
            const Metavision::EventCD* append_end =
                begin + std::min(n, room);
            events_.insert(events_.end(), begin, append_end);
            // Duration = last buffered event timestamp. Updated atomically
            // so on_timer() (GUI thread) can read it without locking.
            const Metavision::timestamp last_t = (append_end - 1)->t;
            Metavision::timestamp cur =
                duration_us_.load(std::memory_order_relaxed);
            while (last_t > cur) {
                if (duration_us_.compare_exchange_weak(
                        cur, last_t, std::memory_order_relaxed)) {
                    break;
                }
            }
        }
        if (n > room && !truncated_) {
            truncated_ = true;
            just_truncated = true;
        }
    }
    // Emit outside the lock; Qt queues this to GUI-thread listeners.
    if (just_truncated) {
        emit buffer_truncated();
    }
}

void FileFrameGenerator::set_geometry(long width, long height) {
    if (width <= 0 || height <= 0) return;
    if (width_ == width && height_ == height && !frame_.empty()) return;
    width_ = width;
    height_ = height;
    frame_.create(static_cast<int>(height_), static_cast<int>(width_), CV_8UC3);
}

void FileFrameGenerator::set_fps(std::uint16_t fps) {
    if (fps == 0) fps = 1;
    fps_ = fps;
    if (timer_.isActive()) {
        timer_.setInterval(1000 / static_cast<int>(fps_));
    }
}

void FileFrameGenerator::set_accumulation_time_us(Metavision::timestamp us) {
    if (us < 1) us = 1;
    accumulation_us_ = us;
}

void FileFrameGenerator::set_color_palette(Metavision::ColorPalette palette) {
    palette_ = palette;
}

void FileFrameGenerator::set_duration_us(Metavision::timestamp us) {
    Metavision::timestamp cur = duration_us_.load(std::memory_order_relaxed);
    while (us > cur) {
        if (duration_us_.compare_exchange_weak(cur, us,
                                               std::memory_order_relaxed)) {
            break;
        }
    }
}

void FileFrameGenerator::play() {
    if (playing_) return;
    if (width_ <= 0 || height_ <= 0) return;
    // If at or past EOF, restart from the beginning. Only meaningful once
    // loading is complete — a cursor parked at the buffer top while the
    // file is still streaming is NOT EOF (audit §六-P2).
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    if (dur > 0 && cursor_us_ >= dur &&
        loading_complete_.load(std::memory_order_acquire)) {
        cursor_us_ = 0;
        // Same contract as seek()/looped(): stateful algorithms must reset
        // before events from the beginning of the file arrive (audit §六-P4).
        emit seeked(0);
    }
    playing_ = true;
    timer_.start(1000 / static_cast<int>(fps_));
}

void FileFrameGenerator::pause() {
    if (!playing_) return;
    timer_.stop();
    playing_ = false;
}

void FileFrameGenerator::seek(Metavision::timestamp t_us) {
    if (t_us < 0) t_us = 0;
    // Clamp to the known duration so a Step past EOF can't park the cursor
    // beyond the last event (the next Play would then restart from 0 as if
    // EOF had been reached — audit §六-P5).
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    if (dur > 0 && t_us > dur) t_us = dur;
    cursor_us_ = t_us;
    // Notify listeners so stateful algorithms can reset their temporal state
    // before the new (possibly earlier) events arrive. Without this, a
    // backward seek leaves algorithm timestamps ahead of the new events,
    // causing them to be ignored and the output to freeze — the same issue
    // as looped() but triggered by user-initiated cursor jumps.
    emit seeked(t_us);
    // Render immediately so the user sees the seeked frame.
    render_frame(cursor_us_, cursor_us_ + accumulation_us_);
    if (width_ > 0 && height_ > 0 && !frame_.empty()) {
        cv::Mat rgb;
        cv::cvtColor(frame_, rgb, cv::COLOR_BGR2RGB);
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step), QImage::Format_RGB888);
        QImage copy = img.copy();
        emit frame_ready(std::move(copy), cursor_us_);
    }
    emit position_changed(cursor_us_,
                          duration_us_.load(std::memory_order_relaxed));
}

Metavision::timestamp FileFrameGenerator::duration_us() const {
    return duration_us_.load(std::memory_order_relaxed);
}

std::size_t FileFrameGenerator::event_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_.size();
}

void FileFrameGenerator::clear() {
    timer_.stop();
    playing_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.clear();
        truncated_ = false;
    }
    duration_us_.store(0, std::memory_order_relaxed);
    // A new file is about to stream in: suspend EOF handling until the
    // loader signals completion (audit §六-P2).
    loading_complete_.store(false, std::memory_order_release);
    cursor_us_ = 0;
}

bool FileFrameGenerator::is_truncated() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return truncated_;
}

void FileFrameGenerator::on_timer() {
    if (width_ <= 0 || height_ <= 0) return;

    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);

    // EOF / buffer-wait check (audit §六-P2). duration_us_ is only the max
    // timestamp buffered SO FAR, so a cursor at/past it means one of two
    // things:
    //   - loading complete  → genuine EOF: stop (emit eof_reached) or,
    //     in loop mode, wrap to 0 (emit looped).
    //   - still loading     → the cursor merely caught up with the read
    //     progress: wait silently (no advance, no EOF, no wrap — wrapping
    //     only makes sense for a complete file) until more events are
    //     buffered or loading completes. playing_ stays true.
    if (dur > 0 && cursor_us_ >= dur) {
        if (!loading_complete_.load(std::memory_order_acquire)) {
            return;
        }
        if (loop_) {
            cursor_us_ = 0;
            emit looped();
        } else {
            timer_.stop();
            playing_ = false;
            emit eof_reached();
            return;
        }
    }

    const Metavision::timestamp start = cursor_us_;
    const Metavision::timestamp end = start + accumulation_us_;

    // Render events in [start, end) to frame_.
    render_frame(start, end);

    // Convert BGR → RGB and emit.
    if (!frame_.empty()) {
        cv::Mat rgb;
        cv::cvtColor(frame_, rgb, cv::COLOR_BGR2RGB);
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step), QImage::Format_RGB888);
        QImage copy = img.copy();
        emit frame_ready(std::move(copy), start);
    }

    cursor_us_ = end;
    emit position_changed(cursor_us_, dur);
}

void FileFrameGenerator::render_frame(Metavision::timestamp start_us,
                                      Metavision::timestamp end_us) {
    if (frame_.empty()) {
        if (width_ > 0 && height_ > 0) {
            frame_.create(static_cast<int>(height_),
                          static_cast<int>(width_), CV_8UC3);
        } else {
            return;
        }
    }

    // Use the Metavision color palette (same as CDFrameGenerator).
    const cv::Vec3b bg   = Metavision::get_bgr_color(palette_, Metavision::ColorType::Background);
    const cv::Vec3b on   = Metavision::get_bgr_color(palette_, Metavision::ColorType::Positive);
    const cv::Vec3b off  = Metavision::get_bgr_color(palette_, Metavision::ColorType::Negative);
    frame_.setTo(cv::Scalar(bg[0], bg[1], bg[2]));

    // Collect the RAW events in [start_us, end_us). These are used both for
    // rendering (after FilterChain transformation) and for feeding algorithm
    // instances (RAW, unfiltered — matching live mode where the algo CD
    // callback is separate from the CameraController's FilterChain callback).
    auto window_events = std::make_shared<std::vector<Metavision::EventCD>>();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!events_.empty()) {
            // Events are sorted by timestamp (SDK guarantee). Binary search
            // for the window boundaries.
            auto begin_it = std::lower_bound(
                events_.begin(), events_.end(), start_us,
                [](const Metavision::EventCD& e, Metavision::timestamp t) {
                    return e.t < t;
                });
            auto end_it = std::lower_bound(
                events_.begin(), events_.end(), end_us,
                [](const Metavision::EventCD& e, Metavision::timestamp t) {
                    return e.t < t;
                });
            window_events->assign(begin_it, end_it);
        }
    }

    // Apply FilterChain to the window events for BOTH display rendering and
    // algorithm feeding. This ensures flip/rotate/etc. take effect immediately
    // during file playback (events are buffered raw and filtered per-frame),
    // AND that algorithm output is also flipped — ReplaceStrategy replaces
    // the display frame with the algorithm's output, so if algorithms receive
    // raw (unflipped) events, the flip would be invisible when a Replace-mode
    // algorithm is running.
    //
    // NOTE: process() clears its output vector before filling it. We must NOT
    // pass *window_events as both input (begin/end) and output — out.clear()
    // would invalidate the input iterators (aliasing UB). Use a separate
    // vector and move the result back.
    const int h = static_cast<int>(height_);
    const int w = static_cast<int>(width_);
    if (filter_chain_ && filter_chain_->has_enabled()) {
        std::vector<Metavision::EventCD> filtered;
        filter_chain_->process(window_events->data(),
                               window_events->data() + window_events->size(),
                               filtered);
        *window_events = std::move(filtered);
    }
    for (const auto& ev : *window_events) {
        if (ev.x < 0 || ev.x >= w || ev.y < 0 || ev.y >= h) continue;
        frame_.ptr<cv::Vec3b>(static_cast<int>(ev.y))[ev.x] = ev.p ? on : off;
    }

    // Emit the (filtered) events in this window so algorithm instances can
    // process them synchronously with the displayed frame. Emitted before
    // frame_ready so results are ready when the frame is displayed.
    emit events_window_ready(window_events, start_us);
}

} // namespace gui
