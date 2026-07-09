// gui/app/file_frame_generator.cpp

#include "file_frame_generator.h"

#include <algorithm>

#include <opencv2/imgproc.hpp>

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
    std::lock_guard<std::mutex> lock(mutex_);
    events_.insert(events_.end(), begin, end);
    // Duration = last event timestamp. Updated atomically so on_timer()
    // (GUI thread) can read it without locking.
    const Metavision::timestamp last_t = (end - 1)->t;
    Metavision::timestamp cur = duration_us_.load(std::memory_order_relaxed);
    while (last_t > cur) {
        if (duration_us_.compare_exchange_weak(cur, last_t,
                                               std::memory_order_relaxed)) {
            break;
        }
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
    // If at or past EOF, restart from the beginning.
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);
    if (dur > 0 && cursor_us_ >= dur) {
        cursor_us_ = 0;
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
    cursor_us_ = t_us;
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

bool FileFrameGenerator::has_events() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !events_.empty();
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
    }
    duration_us_.store(0, std::memory_order_relaxed);
    cursor_us_ = 0;
}

void FileFrameGenerator::on_timer() {
    if (width_ <= 0 || height_ <= 0) return;

    const Metavision::timestamp start = cursor_us_;
    const Metavision::timestamp end = start + accumulation_us_;
    const Metavision::timestamp dur = duration_us_.load(std::memory_order_relaxed);

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

    // EOF check: cursor has passed the last event.
    if (dur > 0 && cursor_us_ >= dur) {
        if (loop_) {
            cursor_us_ = 0;
        } else {
            timer_.stop();
            playing_ = false;
            emit eof_reached();
        }
    }
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

    // Collect the events in [start_us, end_us) for both rendering and for
    // feeding algorithm instances synchronously with this frame.
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

            // Render using palette colors.
            const int h = static_cast<int>(height_);
            const int w = static_cast<int>(width_);
            for (auto it = begin_it; it != end_it; ++it) {
                if (it->x >= w || it->y >= h) continue;
                frame_.at<cv::Vec3b>(static_cast<int>(it->y),
                                     static_cast<int>(it->x)) = it->p ? on : off;
            }
            window_events->assign(begin_it, end_it);
        }
    }

    // Emit the events in this window so algorithm instances can process
    // them synchronously with the displayed frame. Emitted before
    // frame_ready so results are ready when the frame is displayed.
    emit events_window_ready(window_events, start_us);
}

} // namespace gui
