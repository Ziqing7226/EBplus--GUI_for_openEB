// gui/recorder/playback_controller.cpp

#include "playback_controller.h"

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/offline_streaming_control.h>

#include "app/camera_controller.h"
#include "app/frame_pipeline.h"

namespace gui {

PlaybackController::PlaybackController(QObject* parent) : QObject(parent) {}

void PlaybackController::set_camera(CameraController* controller) {
    if (controller_ == controller) return;
    if (controller_) {
        // Disconnect FramePipeline signals (not covered by disconnecting
        // the controller itself, since FramePipeline is a separate object).
        if (auto* fp = controller_->frame_pipeline()) {
            disconnect(fp, nullptr, this, nullptr);
        }
        disconnect(controller_, nullptr, this, nullptr);
    }
    // Reset stale playback state from the previous controller.
    path_.clear();
    duration_us_ = 0;
    at_eof_ = false;
    if (playing_) {
        playing_ = false;
        emit state_changed(false);
    }
    controller_ = controller;
    if (controller_) {
        // In file mode, the SDK camera stops after buffering all events
        // (real_time_playback=false delivers them in ~10ms). This is NOT
        // playback stopping — the FileFrameGenerator continues playing from
        // its buffer. Only update state for live camera stops.
        connect(controller_, &CameraController::stopped, this, [this]() {
            if (controller_ && controller_->is_file_source()) return;
            // Live camera stopped:
            if (playing_) {
                playing_ = false;
                emit state_changed(false);
            }
        });

        // Listen to FramePipeline signals for file playback.
        if (auto* fp = controller_->frame_pipeline()) {
            connect(fp, &FramePipeline::fps_changed,
                    this, &PlaybackController::on_pipeline_param_changed);
            connect(fp, &FramePipeline::accumulation_time_changed,
                    this, &PlaybackController::on_pipeline_param_changed);
            connect(fp, &FramePipeline::file_position_changed,
                    this, &PlaybackController::on_file_position_changed);
            connect(fp, &FramePipeline::file_eof_reached,
                    this, &PlaybackController::on_file_eof);
        }
    }
}

bool PlaybackController::available() const {
    return controller_ && controller_->is_connected() && controller_->is_file_source();
}

Metavision::timestamp PlaybackController::time_window_us() const {
    if (!controller_ || !controller_->frame_pipeline()) return 33333;
    return controller_->frame_pipeline()->accumulation_time_us();
}

std::uint16_t PlaybackController::frame_rate() const {
    if (!controller_ || !controller_->frame_pipeline()) return 30;
    return controller_->frame_pipeline()->fps();
}

double PlaybackController::multiplier() const {
    if (!controller_ || !controller_->frame_pipeline()) return 1.0;
    auto* fp = controller_->frame_pipeline();
    return static_cast<double>(fp->fps()) *
           static_cast<double>(fp->accumulation_time_us()) / 1.0e6;
}

bool PlaybackController::open_file(const QString& path) {
    if (!controller_) {
        emit error(tr("No camera controller bound."));
        return false;
    }
    path_ = path;
    // connect_file opens with real_time_playback=false: all events are read
    // as fast as possible and buffered in the FileFrameGenerator. Playback
    // rate is controlled by the FileFrameGenerator's QTimer, not by SDK
    // delivery speed. This enables slow-motion and fast-forward.
    if (!controller_->connect_file(path.toStdString())) {
        return false;
    }
    // Start the camera so events flow into the FileFrameGenerator buffer.
    // With real_time_playback=false, all events arrive in ~10ms regardless
    // of file duration.
    if (!controller_->start()) {
        return false;
    }
    // Query duration from OSC (ready after start). The FileFrameGenerator
    // also updates duration from the last event timestamp, so this is a
    // secondary source — whichever is larger wins.
    duration_us_ = query_duration();
    if (auto* fp = controller_->frame_pipeline()) {
        fp->set_file_duration_us(duration_us_);
        fp->set_file_loop(loop_);
    }
    at_eof_ = false;
    // Auto-start playback.
    play();
    emit opened(duration_us_);
    emit multiplier_changed(multiplier());
    return true;
}

void PlaybackController::play() {
    if (!available() || playing_) {
        return;
    }
    auto* fp = controller_ ? controller_->frame_pipeline() : nullptr;
    if (!fp) return;
    // If at EOF, restart from the beginning. The FileFrameGenerator also
    // does this internally, but we reset at_eof_ for UI state tracking.
    if (at_eof_) {
        at_eof_ = false;
        fp->seek_file(0);
    }
    fp->play_file();
    playing_ = true;
    emit state_changed(true);
}

void PlaybackController::pause() {
    if (!playing_) {
        return;
    }
    auto* fp = controller_ ? controller_->frame_pipeline() : nullptr;
    if (fp) {
        fp->pause_file();
    }
    playing_ = false;
    at_eof_ = false;  // user paused, not EOF
    emit state_changed(false);
}

void PlaybackController::toggle_play_pause() {
    if (playing_) {
        pause();
    } else {
        play();
    }
}

bool PlaybackController::seek(Metavision::timestamp t_us) {
    if (!available()) {
        return false;
    }
    auto* fp = controller_ ? controller_->frame_pipeline() : nullptr;
    if (!fp) return false;
    // Seek is O(1): just set the cursor and render immediately. No OSC
    // seek, no file reopen — works even after EOF.
    fp->seek_file(t_us);
    // If we were at EOF, seeking clears that state.
    if (at_eof_) {
        at_eof_ = false;
    }
    return true;
}

void PlaybackController::set_loop(bool on) {
    loop_ = on;
    if (auto* fp = controller_ ? controller_->frame_pipeline() : nullptr) {
        fp->set_file_loop(on);
    }
    emit loop_changed(on);
}

void PlaybackController::set_time_window_us(Metavision::timestamp us) {
    if (us < 1) us = 1;
    if (!controller_ || !controller_->frame_pipeline()) return;
    controller_->frame_pipeline()->set_accumulation_time_us(us);
}

void PlaybackController::set_frame_rate(std::uint16_t fps) {
    if (fps == 0) fps = 1;
    if (!controller_ || !controller_->frame_pipeline()) return;
    controller_->frame_pipeline()->set_fps(fps);
}

void PlaybackController::set_multiplier(double m) {
    if (m <= 0.0) m = 0.000001;
    if (!controller_ || !controller_->frame_pipeline()) return;
    auto* fp = controller_->frame_pipeline();
    // fps is locked: derive accumulation from the requested multiplier.
    // accumulation = multiplier * 1e6 / fps
    Metavision::timestamp tw = static_cast<Metavision::timestamp>(
        m * 1.0e6 / static_cast<double>(fp->fps()) + 0.5);
    if (tw < 1) tw = 1;
    fp->set_accumulation_time_us(tw);
}

void PlaybackController::on_pipeline_param_changed() {
    emit multiplier_changed(multiplier());
}

void PlaybackController::on_file_position_changed(Metavision::timestamp pos,
                                                   Metavision::timestamp dur) {
    if (dur > duration_us_) {
        duration_us_ = dur;
    }
    emit position_changed(pos, duration_us_);
}

void PlaybackController::on_file_eof() {
    playing_ = false;
    at_eof_ = true;
    emit state_changed(false);
}

Metavision::timestamp PlaybackController::duration_us() const {
    if (auto* fp = controller_ ? controller_->frame_pipeline() : nullptr) {
        const Metavision::timestamp fd = fp->file_duration_us();
        if (fd > duration_us_) return fd;
    }
    return duration_us_;
}

Metavision::timestamp PlaybackController::position_us() const {
    if (auto* fp = controller_ ? controller_->frame_pipeline() : nullptr) {
        return fp->file_position_us();
    }
    return 0;
}

Metavision::timestamp PlaybackController::query_duration() const {
    if (!available()) {
        return 0;
    }
    auto* cam = controller_->camera_handle();
    if (!cam) {
        return 0;
    }
    try {
        auto& osc = cam->offline_streaming_control();
        if (!osc.is_ready()) {
            return 0;
        }
        return osc.get_duration();
    } catch (...) {
        return 0;
    }
}

} // namespace gui
