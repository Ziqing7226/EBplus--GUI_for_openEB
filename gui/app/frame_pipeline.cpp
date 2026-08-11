// gui/app/frame_pipeline.cpp

#include "frame_pipeline.h"

#include <opencv2/imgproc.hpp>

namespace gui {

FramePipeline::FramePipeline(QObject* parent) : QObject(parent) {
    // Forward FileFrameGenerator signals to FramePipeline signals.
    connect(&file_generator_, &FileFrameGenerator::frame_ready,
            this, &FramePipeline::frame_ready);
    connect(&file_generator_, &FileFrameGenerator::position_changed,
            this, &FramePipeline::file_position_changed);
    connect(&file_generator_, &FileFrameGenerator::eof_reached,
            this, &FramePipeline::file_eof_reached);
    connect(&file_generator_, &FileFrameGenerator::looped,
            this, &FramePipeline::file_looped);
    connect(&file_generator_, &FileFrameGenerator::seeked,
            this, &FramePipeline::file_seeked);
    connect(&file_generator_, &FileFrameGenerator::events_window_ready,
            this, &FramePipeline::events_window_ready);
    connect(&file_generator_, &FileFrameGenerator::buffer_truncated,
            this, &FramePipeline::file_buffer_truncated);

    // File-path frame-mode rendering: render_frame feeds the shared renderer.
    file_generator_.set_frame_mode_renderer(&frame_mode_renderer_);
    // Stateful frame modes (TimeDecay / EventsIntegration) must not span a
    // backward time jump: reset on seek and loop.
    connect(this, &FramePipeline::file_seeked, this,
            [this](Metavision::timestamp) { frame_mode_renderer_.reset(); });
    connect(this, &FramePipeline::file_looped, this,
            [this] { frame_mode_renderer_.reset(); });

    // Live-mode tick for non-integration frame modes: generate the current
    // mode's frame at fps from the accumulated events.
    frame_mode_timer_ = new QTimer(this);
    frame_mode_timer_->setTimerType(Qt::PreciseTimer);
    connect(frame_mode_timer_, &QTimer::timeout,
            this, &FramePipeline::on_frame_mode_tick);
}

FramePipeline::~FramePipeline() {
    stop();
}

std::uint16_t FramePipeline::clamp_fps(std::uint16_t fps) const {
    if (fps < 1) fps = 1;
    if (fps > fps_limit_) fps = fps_limit_;
    return fps;
}

bool FramePipeline::start(long width, long height,
                          std::uint16_t fps,
                          Metavision::timestamp accumulation_time_us) {
    if (is_running()) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        return false;
    }
    width_  = width;
    height_ = height;
    fps_    = clamp_fps(fps);
    accumulation_us_ = accumulation_time_us;
    file_mode_ = false;
    {
        std::lock_guard<std::mutex> lk(display_preproc_mutex_);
        display_preproc_.init(static_cast<int>(width_), static_cast<int>(height_));
        // Source restart: temporal state of the display filter must not
        // carry over (timestamps may jump backward).
        display_preproc_.reset_filter();
    }
    generator_ = std::make_unique<gui_algo::FrameGenerator>(width_, height_);
    recreate_window();
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.set_mode(frame_mode_);
    frame_mode_renderer_.reset();
    update_frame_mode_timer();
    return window_id_ >= 0;
}

bool FramePipeline::start_file(long width, long height,
                               std::uint16_t fps,
                               Metavision::timestamp accumulation_time_us) {
    if (is_running()) {
        return false;
    }
    if (width <= 0 || height <= 0) {
        return false;
    }
    width_  = width;
    height_ = height;
    fps_    = clamp_fps(fps);
    accumulation_us_ = accumulation_time_us;
    file_mode_ = true;
    file_generator_.clear();
    file_generator_.set_geometry(width_, height_);
    file_generator_.set_fps(fps_);
    file_generator_.set_accumulation_time_us(accumulation_us_);
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.set_mode(frame_mode_);
    frame_mode_renderer_.reset();
    return true;
}

void FramePipeline::recreate_window() {
    if (!generator_) {
        return;
    }
    if (window_id_ >= 0) {
        generator_->remove_window(window_id_);
        window_id_ = -1;
    }
    gui_algo::FrameGenerator::WindowParams params;
    params.fps = fps_;
    params.accumulation_time_us = accumulation_us_;

    // The callback runs on CDFrameGenerator's internal thread. We must deep
    // copy the cv::Mat before emitting, since the SDK reuses it.
    window_id_ = generator_->add_window(
        "main", params,
        [this](Metavision::timestamp ts, cv::Mat& frame) {
            // Non-integration frame modes emit via the mode tick instead.
            if (frame_mode_ != FrameMode::Integration) {
                return;
            }
            if (frame.empty()) {
                return;
            }
            try {
                cv::Mat rgb;
                if (frame.channels() == 1) {
                    cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
                } else {
                    cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
                }
                QImage img(rgb.data, rgb.cols, rgb.rows,
                           static_cast<int>(rgb.step), QImage::Format_RGB888);
                QImage copy = img.copy(); // deep copy (rgb is local)
                emit frame_ready(std::move(copy), ts);
            } catch (const std::exception&) {
                // Swallow — the SDK thread must not propagate exceptions
            } catch (...) {}
        });
}

void FramePipeline::stop() {
    if (frame_mode_timer_) frame_mode_timer_->stop();
    if (file_mode_) {
        file_generator_.pause();
        file_generator_.clear();
        file_mode_ = false;
    }
    if (generator_) {
        generator_->remove_window(window_id_);
        generator_.reset();
    }
    window_id_ = -1;
    frame_mode_renderer_.reset();
}

void FramePipeline::add_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    if (file_mode_) {
        file_generator_.add_events(begin, end);
    } else if (generator_) {
        // Display-path preprocessing (Phase 2.5): apply the Preprocessing
        // panel's stages to the DISPLAY stream. gui_algo::Event and
        // Metavision::EventCD are layout-compatible (static_assert in
        // algo/common/event.h), so the reinterpret_cast is safe.
        std::lock_guard<std::mutex> lk(display_preproc_mutex_);
        const Metavision::EventCD* out_b = begin;
        const Metavision::EventCD* out_e = end;
        if (display_preproc_.active()) {
            const auto n = static_cast<std::size_t>(end - begin);
            auto [p, m] = display_preproc_.apply(
                reinterpret_cast<const gui_algo::Event*>(begin), n);
            out_b = reinterpret_cast<const Metavision::EventCD*>(p);
            out_e = reinterpret_cast<const Metavision::EventCD*>(p) + m;
        }
        // Processed-stream recording (Phase 2.5 step 5): the listener gets
        // the same span the display sees (raw when all stages are off, so
        // the recording stays continuous across preproc toggles).
        if (processed_listener_) processed_listener_(out_b, out_e);
        // Non-integration frame modes feed the frame-mode renderer (their
        // tick emits frame_ready); the CDFrameGenerator path is bypassed.
        if (frame_mode_ != FrameMode::Integration) {
            frame_mode_renderer_.add_events(out_b, out_e);
            if (out_e > out_b) {
                last_ev_ts_.store((out_e - 1)->t, std::memory_order_relaxed);
            }
        } else {
            generator_->add_events(out_b, out_e);
        }
    }
}

void FramePipeline::set_display_preproc_param(const std::string& key,
                                              const std::string& value) {
    {
        std::lock_guard<std::mutex> lk(display_preproc_mutex_);
        display_preproc_.set_param(key, value);
    }
    file_generator_.set_display_preproc_param(key, value);
}

void FramePipeline::reset_display_preproc_filter() {
    {
        std::lock_guard<std::mutex> lk(display_preproc_mutex_);
        display_preproc_.reset_filter();
    }
    file_generator_.reset_display_preproc_filter();
}

void FramePipeline::set_accumulation_time_us(Metavision::timestamp us) {
    if (us == accumulation_us_) return;
    accumulation_us_ = us;
    if (file_mode_) {
        file_generator_.set_accumulation_time_us(us);
    } else if (generator_) {
        generator_->set_accumulation_time_us(us);
    }
    emit accumulation_time_changed(us);
}

void FramePipeline::set_fps(std::uint16_t fps) {
    fps = clamp_fps(fps);
    if (fps == fps_) return;
    fps_ = fps;
    if (file_mode_) {
        file_generator_.set_fps(fps);
    } else if (generator_) {
        recreate_window();
    }
    update_frame_mode_timer();
    emit fps_changed(fps_);
}

void FramePipeline::set_fps_limit(std::uint16_t limit) {
    if (limit < 1) limit = 1;
    if (limit == fps_limit_) return;
    fps_limit_ = limit;
    emit fps_limit_changed(fps_limit_);
    if (fps_ > fps_limit_) {
        set_fps(fps_limit_);
    }
}

void FramePipeline::set_color_palette(Metavision::ColorPalette palette) {
    palette_ = palette;
    frame_mode_renderer_.set_palette(palette);
    if (generator_) {
        generator_->set_color_palette(palette);
    }
    // File mode: forward to FileFrameGenerator so render_frame() uses the
    // palette selected in DisplayPanel (matches CDFrameGenerator behavior).
    file_generator_.set_color_palette(palette);
}

void FramePipeline::set_frame_mode(FrameMode mode) {
    if (frame_mode_ == mode) return;
    frame_mode_ = mode;
    frame_mode_renderer_.set_mode(mode);
    frame_mode_renderer_.set_geometry(static_cast<int>(width_),
                                      static_cast<int>(height_));
    frame_mode_renderer_.reset();
    update_frame_mode_timer();
    emit frame_mode_changed(mode);
}

void FramePipeline::set_frame_decay_time_us(Metavision::timestamp us) {
    frame_mode_renderer_.set_decay_time_us(us);
}

void FramePipeline::update_frame_mode_timer() {
    if (!frame_mode_timer_) return;
    frame_mode_timer_->stop();
    // Live mode + non-integration frame modes: generate at fps on a timer.
    if (!file_mode_ && frame_mode_ != FrameMode::Integration && width_ > 0) {
        frame_mode_timer_->start(std::max(1, 1000 / static_cast<int>(fps_)));
    }
}

void FramePipeline::on_frame_mode_tick() {
    cv::Mat frame = frame_mode_renderer_.generate();
    if (frame.empty() || frame.channels() == 0) {
        return;
    }
    try {
        cv::Mat rgb;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
        } else {
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
        }
        QImage img(rgb.data, rgb.cols, rgb.rows,
                   static_cast<int>(rgb.step), QImage::Format_RGB888);
        emit frame_ready(img.copy(), last_ev_ts_.load(std::memory_order_relaxed));
    } catch (const std::exception&) {
        // Swallow — the tick must not propagate exceptions
    } catch (...) {}
}

void FramePipeline::set_file_filter_chain(FilterChain* fc) {
    file_generator_.set_filter_chain(fc);
}

// --- File playback control ---

void FramePipeline::play_file() {
    if (file_mode_) file_generator_.play();
}

void FramePipeline::pause_file() {
    if (file_mode_) file_generator_.pause();
}

void FramePipeline::seek_file(Metavision::timestamp t_us) {
    if (file_mode_) file_generator_.seek(t_us);
}

void FramePipeline::set_file_loop(bool on) {
    if (file_mode_) file_generator_.set_loop(on);
}

void FramePipeline::set_file_duration_us(Metavision::timestamp us) {
    if (file_mode_) file_generator_.set_duration_us(us);
}

void FramePipeline::set_file_loading_complete(bool complete) {
    if (file_mode_) file_generator_.set_loading_complete(complete);
}

Metavision::timestamp FramePipeline::file_position_us() const {
    if (file_mode_) return file_generator_.position_us();
    return 0;
}

Metavision::timestamp FramePipeline::file_duration_us() const {
    if (file_mode_) return file_generator_.duration_us();
    return 0;
}

} // namespace gui
