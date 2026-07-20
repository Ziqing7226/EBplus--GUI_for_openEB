// gui/exporter/exporter_controller.cpp

#include "exporter_controller.h"

#include <QFile>
#include <QMetaObject>

#include <algorithm>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/core/utils/colors.h>
#include <metavision/sdk/core/utils/cv_video_recorder.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/stream/hdf5_event_file_writer.h>

#include <opencv2/imgproc.hpp>

namespace gui {

ExporterController::ExporterController(QObject* parent) : QObject(parent) {}

ExporterController::~ExporterController() {
    cancel();
    if (worker_.joinable()) worker_.join();
}

bool ExporterController::start(const ExportParams& params) {
    if (running_) return false;
    if (worker_.joinable()) worker_.join();  // reap previous finished worker
    cancel_ = false;
    running_ = true;
    worker_ = std::thread([this, params]() {
        try {
            if (params.format == ExportParams::Format::HDF5) {
                run_hdf5(params);
            } else {
                run_avi(params);
            }
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
                emit failed(msg);
            }, Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                emit failed(tr("Export failed with an unknown error."));
            }, Qt::QueuedConnection);
        }
        running_ = false;
    });
    return true;
}

void ExporterController::cancel() {
    cancel_ = true;
}

void ExporterController::run_hdf5(const ExportParams& p) {
    Metavision::Camera cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false); // as fast as possible
        cam = Metavision::Camera::from_file(p.source_path.toStdString(), hints);
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // Query total duration once for progress reporting. The OSC is only
    // available for file sources, which is always the case for export.
    Metavision::timestamp dur_us = 0;
    try {
        auto& osc = cam.offline_streaming_control();
        if (osc.is_ready()) dur_us = osc.get_duration();
    } catch (...) {}

    // callback_error captures the first exception message from the writer
    // callback (e.g. missing HDF5 ECF compression plugin). It is written
    // before cancel_ is set (release), so the polling loop observes it after
    // reading cancel_ (acquire). Without this, a genuine failure would be
    // silently reported as "Export cancelled" — hiding the real cause.
    std::string callback_error;
    Metavision::HDF5EventFileWriter writer(p.output_path.toStdString());
    std::atomic<Metavision::timestamp> last_ts{0};
    auto id = cam.cd().add_callback(
        [&writer, &last_ts, &callback_error, this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            try {
                if (cancel_) return;
                writer.add_events(b, e);
                if (b != e) last_ts.store((e - 1)->t, std::memory_order_relaxed);
            } catch (const std::exception& ex) {
                callback_error = ex.what();
                cancel_.store(true, std::memory_order_release);
            } catch (...) {
                callback_error = "Unknown error in HDF5 writer";
                cancel_.store(true, std::memory_order_release);
            }
        });
    cam.start();
    // Spin until EOF or cancel. Emit incremental progress so the progress bar
    // does not sit at 0% for the entire export (only jumping to 100% at the end).
    while (!cancel_) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!cam.is_running()) break;
            if (dur_us > 0) {
                const double r = std::min(1.0, static_cast<double>(
                    last_ts.load(std::memory_order_relaxed)) / dur_us);
                QMetaObject::invokeMethod(this, [this, r]() { emit progress(r); },
                                          Qt::QueuedConnection);
            }
        } catch (...) {
            break;
        }
    }
    try { cam.stop(); } catch (...) {}
    cam.cd().remove_callback(id);
    // writer.close() may throw if the ECF compression plugin is missing or
    // the file system is full. Capture the error so the user sees the cause
    // rather than a generic abort.
    try {
        writer.close();
    } catch (const std::exception& ex) {
        callback_error = ex.what();
        cancel_.store(true, std::memory_order_release);
    } catch (...) {
        callback_error = "Unknown error closing HDF5 file";
        cancel_.store(true, std::memory_order_release);
    }

    // Distinguish cancel from completion: a cancelled export must not emit
    // completed (the output file is partial/truncated). Delete the partial
    // file so the user can't mistake it for a valid recording (audit §六-E4).
    if (cancel_.load(std::memory_order_acquire)) {
        QFile::remove(p.output_path);
        QMetaObject::invokeMethod(this, [this, msg = callback_error]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

void ExporterController::run_avi(const ExportParams& p) {
    Metavision::Camera cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        cam = Metavision::Camera::from_file(p.source_path.toStdString(), hints);
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // Query total duration once for progress reporting.
    Metavision::timestamp dur_us = 0;
    try {
        auto& osc = cam.offline_streaming_control();
        if (osc.is_ready()) dur_us = osc.get_duration();
    } catch (...) {}

    // Geometry may be unavailable for certain file formats (e.g. DAT without
    // embedded geometry metadata). Querying it outside a try/catch would
    // crash via std::terminate on an uncaught exception. Likewise, w/h == 0
    // would make cv::VideoWriter segfault on cv::Size(0,0).
    int w = 0, h = 0;
    try {
        const auto& g = cam.geometry();
        w = g.get_width();
        h = g.get_height();
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, msg = tr("Source file has no geometry: %1")
                                                              .arg(QString::fromUtf8(e.what()))]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }
    if (w <= 0 || h <= 0) {
        QMetaObject::invokeMethod(this, [this, w, h]() {
            emit failed(tr("Source file has invalid geometry (%1x%2). Cannot export.")
                            .arg(w).arg(h));
        }, Qt::QueuedConnection);
        return;
    }

    const int fourcc = (p.quality >= 50) ? cv::VideoWriter::fourcc('H', '2', '6', '4')
                                         : cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    Metavision::CvVideoRecorder recorder(p.output_path.toStdString(), fourcc,
                                         static_cast<uint32_t>(p.fps), cv::Size(w, h), p.color);
    if (!recorder.start()) {
        QMetaObject::invokeMethod(this, [this]() {
            emit failed(tr("Failed to open AVI writer. Check the output path and codec."));
        }, Qt::QueuedConnection);
        return;
    }

    // process_all_frames=true: with the default (false) the generator only
    // encodes the LAST frame of each buffered batch — intermediate frames are
    // skipped (video duration compressed) and stop() aborts with buffered
    // events still unprocessed (video tail missing). See audit §六-E1.
    //
    // Caveat: with process_all_frames=true the generator's internal
    // events_back_ queue grows whenever encoding is slower than reading
    // (real_time_playback=false reads as fast as possible), so exporting
    // very long files can grow memory significantly until a feed-throttling
    // mechanism is added (out of scope for this fix round).
    auto gen = std::make_unique<Metavision::CDFrameGenerator>(
        w, h, /*process_all_frames=*/true);
    gen->set_color_palette(p.color ? Metavision::ColorPalette::Dark : Metavision::ColorPalette::Gray);
    gen->set_display_accumulation_time_us(
        static_cast<Metavision::timestamp>(p.accumulation_us));

    // callback_error captures the first exception message from any callback.
    // It is written before cancel_ is set (release), so the polling loop
    // observes it after reading cancel_ (acquire). This lets us distinguish
    // a real error from a user-initiated cancel and show the actual cause.
    std::string callback_error;
    std::atomic<bool> done{false};
    // start() takes (fps, callback). The callback receives (timestamp, frame).
    // It returns false if the generator thread failed to start.
    //
    // Color handling: CDFrameGenerator defaults to single-channel (grayscale)
    // output regardless of the palette; set_color_palette only selects the
    // lookup table, not the channel count. CvVideoRecorder was constructed
    // with p.color as its `colored` flag, so we must hand it a frame whose
    // channel count matches: 3-channel BGR when p.color, 1-channel gray
    // otherwise. cv::VideoWriter with isColor=false fed a 3-channel image
    // (or vice versa) produces corrupted output or an OpenCV assertion.
    if (!gen->start(static_cast<std::uint16_t>(p.fps),
                    [&recorder, &done, &callback_error, this, color = p.color](Metavision::timestamp, cv::Mat& frame) {
                        try {
                            if (cancel_ || done) return;
                            if (frame.empty()) return;
                            cv::Mat out;
                            if (color) {
                                if (frame.channels() == 1) {
                                    cv::cvtColor(frame, out, cv::COLOR_GRAY2BGR);
                                } else {
                                    out = frame;
                                }
                            } else {
                                if (frame.channels() == 3) {
                                    cv::cvtColor(frame, out, cv::COLOR_BGR2GRAY);
                                } else {
                                    out = frame;
                                }
                            }
                            recorder.write(out);
                        } catch (const std::exception& e) {
                            callback_error = e.what();
                            cancel_.store(true, std::memory_order_release);
                        } catch (...) {
                            callback_error = "Unknown error in frame writer";
                            cancel_.store(true, std::memory_order_release);
                        }
                    })) {
        QMetaObject::invokeMethod(this, [this]() {
            emit failed(tr("Failed to start CD frame generator."));
        }, Qt::QueuedConnection);
        recorder.stop();
        return;
    }

    std::atomic<Metavision::timestamp> last_ts{0};
    auto id = cam.cd().add_callback(
        [&gen, &last_ts, &callback_error, this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            try {
                gen->add_events(b, e);
                if (b != e) last_ts.store((e - 1)->t, std::memory_order_relaxed);
            } catch (const std::exception& e2) {
                callback_error = e2.what();
                cancel_.store(true, std::memory_order_release);
            } catch (...) {
                callback_error = "Unknown error in event callback";
                cancel_.store(true, std::memory_order_release);
            }
        });

    cam.start();
    while (!cancel_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!cam.is_running()) break;
        if (dur_us > 0) {
            const double r = std::min(1.0, static_cast<double>(
                last_ts.load(std::memory_order_relaxed)) / dur_us);
            QMetaObject::invokeMethod(this, [this, r]() { emit progress(r); },
                                      Qt::QueuedConnection);
        }
    }
    done = true;
    try { cam.stop(); } catch (...) {}
    cam.cd().remove_callback(id);
    gen->stop();
    recorder.stop();

    if (cancel_.load(std::memory_order_acquire)) {
        // Partial/truncated AVI — delete it (audit §六-E4).
        QFile::remove(p.output_path);
        QMetaObject::invokeMethod(this, [this, msg = callback_error]() {
            emit failed(msg.empty() ? tr("Export cancelled.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, out = p.output_path]() {
        emit progress(1.0);
        emit completed(out);
    }, Qt::QueuedConnection);
}

} // namespace gui
