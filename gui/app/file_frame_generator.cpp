// gui/app/file_frame_generator.cpp

#include "file_frame_generator.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "algo_bridge/filter_chain.h"
#include "frame_mode_renderer.h"

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
    display_preproc_.init(static_cast<int>(width), static_cast<int>(height));
    // Recompute the software ROI rect against the new sensor size.
    set_display_roi(roi_enabled_, roi_x_, roi_y_, roi_w_, roi_h_, roi_roni_);
}

void FileFrameGenerator::set_display_roi(bool enabled, int x, int y, int w, int h,
                                         bool roni) {
    roi_enabled_ = enabled;
    roi_roni_ = roni;
    roi_x_ = x; roi_y_ = y; roi_w_ = w; roi_h_ = h;
    // Compute the rect (auto-center on -1, clamp to sensor), mirroring
    // ProcessRegion::compute and CameraController::set_unified_roi.
    const int sw = width_ > 0 ? static_cast<int>(width_) : 1280;
    const int sh = height_ > 0 ? static_cast<int>(height_) : 720;
    const int rw = (w <= 0) ? sw : std::min(w, sw);
    const int rh = (h <= 0) ? sh : std::min(h, sh);
    const int rx = (x < 0) ? (sw - rw) / 2 : std::min(std::max(0, x), sw - rw);
    const int ry = (y < 0) ? (sh - rh) / 2 : std::min(std::max(0, y), sh - rh);
    roi_x0_ = rx; roi_y0_ = ry;
    roi_x1_ = std::min(rx + rw, sw);
    roi_y1_ = std::min(ry + rh, sh);
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
    // Display filter temporal state must not span a backward time jump
    // (Phase 2.5) — stale timestamp surfaces would suppress events.
    display_preproc_.reset_filter();
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
    //     progress: wait silently (no advance, no EOF, NO WRAP) until more
    //     events are buffered or loading completes. playing_ stays true.
    //
    // The wait applies to loop mode too (§12.2-A revisited): wrapping while
    // loading replays the buffered prefix over and over — with a tiny
    // accumulation window the cursor outruns the loader within the first
    // milliseconds and the first window's frame is re-emitted repeatedly
    // (user report: "开头反复闪烁同一个累积帧"). The edcfbf3 concern (loop
    // never wraps if loading_complete_ is never set) is handled by the
    // loader's reliable completion signal (camera_controller.cpp:268);
    // a genuinely wedged loader stalls playback instead of flashing —
    // the lesser evil.
    if (dur > 0 && cursor_us_ >= dur) {
        if (!loading_complete_.load(std::memory_order_acquire)) {
            return;
        }
        if (loop_) {
            cursor_us_ = 0;
            // Display filter temporal state must not span the loop wrap
            // (Phase 2.5) — event time jumps back to the start of the file.
            display_preproc_.reset_filter();
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

    // Software ROI (Phase 2.6): drop events outside the rect so the rendered
    // frame AND the algorithm feed (events_window_ready) both see the same
    // ROI-limited stream — identical semantics to a live source with the
    // hardware ROI enabled. RONI mode (Phase 2.6 debug D-5) inverts the
    // predicate: keep events OUTSIDE the rect (hardware drops inside).
    if (roi_enabled_) {
        auto& evs = *window_events;
        std::size_t kept = 0;
        for (std::size_t i = 0; i < evs.size(); ++i) {
            const auto& ev = evs[i];
            const bool inside = ev.x >= roi_x0_ && ev.x < roi_x1_ &&
                                ev.y >= roi_y0_ && ev.y < roi_y1_;
            if (inside != roi_roni_) {
                evs[kept++] = ev;
            }
        }
        evs.resize(kept);
    }

    // Apply FilterChain to the window events for BOTH display rendering and
    // algorithm feeding. This ensures flip/etc. take effect immediately
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
    // Display-path preprocessing (Phase 2.5): apply the Preprocessing panel's
    // noise filter to the RENDERED pixels only. The events emitted to
    // algorithm instances below are intentionally NOT noise-filtered here —
    // each algorithm owns its Preprocessor stage with the same config.
    const std::vector<Metavision::EventCD>* draw_events = window_events.get();
    std::vector<Metavision::EventCD> display_filtered;
    if (display_preproc_.active() && !window_events->empty()) {
        auto [p, m] = display_preproc_.apply(
            reinterpret_cast<const gui_algo::Event*>(window_events->data()),
            window_events->size());
        display_filtered.assign(
            reinterpret_cast<const Metavision::EventCD*>(p),
            reinterpret_cast<const Metavision::EventCD*>(p) + m);
        draw_events = &display_filtered;
    }
    // Non-integration frame modes: feed the window events to the shared
    // frame-mode renderer and use its generated frame for display. Otherwise
    // keep the palette event render (Integration mode).
    if (frame_mode_renderer_ && frame_mode_renderer_->active()) {
        frame_mode_renderer_->add_events(draw_events->data(),
                                         draw_events->data() + draw_events->size());
        cv::Mat mode_frame = frame_mode_renderer_->generate();
        if (!mode_frame.empty() && mode_frame.size() == frame_.size()) {
            frame_ = mode_frame;
        } else {
            frame_.setTo(cv::Scalar(bg[0], bg[1], bg[2]));
        }
    } else {
        for (const auto& ev : *draw_events) {
            if (ev.x < 0 || ev.x >= w || ev.y < 0 || ev.y >= h) continue;
            frame_.ptr<cv::Vec3b>(static_cast<int>(ev.y))[ev.x] = ev.p ? on : off;
        }
    }

    // Emit the (filtered) events in this window so algorithm instances can
    // process them synchronously with the displayed frame. Emitted before
    // frame_ready so results are ready when the frame is displayed.
    emit events_window_ready(window_events, start_us);
}

} // namespace gui
