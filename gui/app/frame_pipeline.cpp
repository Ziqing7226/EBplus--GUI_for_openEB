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
    generator_ = std::make_unique<gui_algo::FrameGenerator>(width_, height_);
    recreate_window();
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
}

void FramePipeline::add_events(const Metavision::EventCD* begin,
                               const Metavision::EventCD* end) {
    if (file_mode_) {
        file_generator_.add_events(begin, end);
    } else if (generator_) {
        generator_->add_events(begin, end);
    }
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
    if (generator_) {
        generator_->set_color_palette(palette);
    }
    // File mode: forward to FileFrameGenerator so render_frame() uses the
    // palette selected in DisplayPanel (matches CDFrameGenerator behavior).
    file_generator_.set_color_palette(palette);
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

Metavision::timestamp FramePipeline::file_position_us() const {
    if (file_mode_) return file_generator_.position_us();
    return 0;
}

Metavision::timestamp FramePipeline::file_duration_us() const {
    if (file_mode_) return file_generator_.duration_us();
    return 0;
}

bool FramePipeline::file_is_playing() const {
    if (file_mode_) return file_generator_.is_playing();
    return false;
}

} // namespace gui
