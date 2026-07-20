// gui/recorder/recorder_controller.cpp

#include "recorder_controller.h"

#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/sdk/stream/camera.h>

#include "app/camera_controller.h"

namespace gui {

RecorderController::RecorderController(QObject* parent) : QObject(parent) {
    connect(&timer_, &QTimer::timeout, this, [this]() {
        emit elapsed(std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time_));
    });
    flush_timer_.setInterval(20);
    connect(&flush_timer_, &QTimer::timeout, this, [this]() {
        if (!recording_ || !controller_) return;
        auto* cam = controller_->camera_handle();
        if (!cam) return;
        try {
            if (auto* stream = cam->get_device().get_facility<Metavision::I_EventsStream>()) {
                stream->get_latest_raw_data();
            }
        } catch (...) {}
    });
}

RecorderController::~RecorderController() {
    stop();
}

bool RecorderController::start(CameraController* controller, const QString& path) {
    if (recording_ || !controller || path.isEmpty()) {
        return false;
    }
    // Recording is only meaningful for live cameras: a file-playback source
    // has no underlying hardware stream and log_raw_data would silently
    // produce an empty / corrupt file.
    if (controller->is_file_source()) {
        emit error(tr("Recording is only available for live cameras."));
        return false;
    }
    auto* camera = controller->camera_handle();
    if (!camera) {
        emit error(tr("No camera connected."));
        return false;
    }
    // Recording while the camera is not streaming produces an empty file
    // (audit §六-C5) — require an active stream.
    if (!controller->is_running()) {
        emit error(tr("Camera is not streaming. Start streaming before recording."));
        return false;
    }
    Metavision::I_EventsStream* stream = nullptr;
    try {
        stream = camera->get_device().get_facility<Metavision::I_EventsStream>();
        if (!stream) {
            emit error(tr("Recording is not supported by this device."));
            return false;
        }
        if (!stream->log_raw_data(path.toStdString())) {
            emit error(tr("Failed to open recording file:\n%1").arg(path));
            return false;
        }
    } catch (const std::exception& e) {
        emit error(QString::fromUtf8(e.what()));
        return false;
    }
    controller_ = controller;
    path_ = path;
    recording_ = true;
    start_time_ = std::chrono::steady_clock::now();
    timer_.start(1000);
    // Per the I_EventsStream contract: "The writing of each buffer of event
    // will have to be triggered by calls to get_latest_raw_data." Drive that
    // from a high-frequency timer so events actually land in the file.
    flush_timer_.start(20);
    emit recording_started(path);
    return true;
}

void RecorderController::stop() {
    if (!recording_) {
        return;
    }
    recording_ = false;
    timer_.stop();
    flush_timer_.stop();
    // Explicitly stop logging so buffers are flushed and the file is closed
    // cleanly. The SDK does NOT do this implicitly on stream teardown.
    if (controller_) {
        if (auto* cam = controller_->camera_handle()) {
            if (auto* stream = cam->get_device().get_facility<Metavision::I_EventsStream>()) {
                // Drain the final hardware buffer before closing the log.
                // stop_log_raw_data only flushes already-pulled data to
                // disk; events that arrived between the last flush_timer
                // tick (20 ms) and now would otherwise be lost.
                try { stream->get_latest_raw_data(); } catch (...) {}
                // stop_log_raw_data must run even if get_latest_raw_data
                // threw, otherwise the RAW file is left without a clean
                // footer and the most recent events are lost.
                try { stream->stop_log_raw_data(); } catch (...) {}
            }
        }
    }
    QString p = path_;
    path_.clear();
    controller_ = nullptr;
    emit recording_stopped(p);
}

} // namespace gui
