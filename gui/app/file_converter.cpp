// gui/app/file_converter.cpp

#include "file_converter.h"

#include <atomic>
#include <chrono>
#include <thread>

#include <QFile>
#include <QMetaObject>
#include <QTextStream>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/camera_exception.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/stream/hdf5_event_file_writer.h>
#include <metavision/sdk/stream/offline_streaming_control.h>
#include <metavision/sdk/stream/raw_evt2_event_file_writer.h>

namespace gui {

FileConverter::FileConverter(QObject* parent) : QObject(parent) {}

FileConverter::~FileConverter() {
    cancel();
    if (worker_.joinable()) worker_.join();
}

void FileConverter::cancel() { cancel_ = true; }

void FileConverter::convert(const QString& src, const QString& dst, Format fmt) {
    if (running_) return;
    if (worker_.joinable()) worker_.join();  // reap previous finished worker
    cancel_ = false;
    running_ = true;
    worker_ = std::thread([this, src, dst, fmt]() {
        // Wrap the body so an exception (HDF5 writer throw, CameraException
        // from cam.start(), add_events ordering throw, etc.) does not escape
        // the thread and call std::terminate. Mirrors ExporterController.
        try {
            run_convert(src, dst, fmt);
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
                emit failed(msg);
            }, Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                emit failed(tr("Conversion failed with an unknown error."));
            }, Qt::QueuedConnection);
        }
        running_ = false;
    });
}

void FileConverter::cut(const QString& src, const QString& dst,
                        Metavision::timestamp start_us, Metavision::timestamp end_us) {
    if (running_) return;
    if (worker_.joinable()) worker_.join();  // reap previous finished worker
    cancel_ = false;
    running_ = true;
    worker_ = std::thread([this, src, dst, start_us, end_us]() {
        try {
            run_cut(src, dst, start_us, end_us);
        } catch (const std::exception& e) {
            QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
                emit failed(msg);
            }, Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(this, [this]() {
                emit failed(tr("Cut failed with an unknown error."));
            }, Qt::QueuedConnection);
        }
        running_ = false;
    });
}

FileInfo FileConverter::info(const QString& src) const {
    FileInfo fi;
    fi.path = src;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        Metavision::Camera cam = Metavision::Camera::from_file(src.toStdString(), hints);
        const auto& g = cam.geometry();
        fi.width = g.get_width();
        fi.height = g.get_height();
        const auto& cfg = cam.get_camera_configuration();
        fi.integrator = QString::fromStdString(cfg.integrator);
        fi.serial = QString::fromStdString(cfg.serial_number);
        fi.plugin = QString::fromStdString(cfg.plugin_name);
        fi.encoding = QString::fromStdString(cfg.data_encoding_format);
        try {
            auto& osc = cam.offline_streaming_control();
            if (osc.is_ready()) {
                fi.duration_us = osc.get_duration();
            }
        } catch (const Metavision::CameraException&) {
            // Live cameras and unsupported files have no duration.
        }
    } catch (const Metavision::CameraException&) {
        // leave defaults
    }
    return fi;
}

void FileConverter::run_convert(const QString& src, const QString& dst, Format fmt) {
    Metavision::Camera cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        cam = Metavision::Camera::from_file(src.toStdString(), hints);
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // Note: cam.geometry() is not needed here — the HDF5 writer extracts
    // geometry from camera metadata internally, and CSV events carry their
    // own coordinates. Avoiding the call eliminates a segfault when the
    // source file format does not provide a geometry facility.

    std::unique_ptr<Metavision::HDF5EventFileWriter> hdf5;
    std::unique_ptr<QFile> csvf;
    std::unique_ptr<QTextStream> csvs;
    if (fmt == Format::HDF5) {
        hdf5 = std::make_unique<Metavision::HDF5EventFileWriter>(dst.toStdString());
    } else {
        csvf = std::make_unique<QFile>(dst);
        if (!csvf->open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMetaObject::invokeMethod(this, [this]() {
                emit failed(tr("Cannot open CSV output file."));
            }, Qt::QueuedConnection);
            return;
        }
        csvs = std::make_unique<QTextStream>(csvf.get());
        *csvs << "t,x,y,p\n";
    }

    // callback_error_msg captures the first exception message (e.g. missing
    // HDF5 ECF compression plugin). It is written before the callback_error
    // flag (release), so the polling loop observes it after reading the flag
    // (acquire).
    std::string callback_error_msg;
    std::atomic<bool> callback_error{false};
    auto id = cam.cd().add_callback(
        [&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (cancel_) return;
            try {
                if (fmt == Format::HDF5) {
                    hdf5->add_events(b, e);
                } else {
                    for (auto it = b; it != e; ++it) {
                        *csvs << it->t << ',' << it->x << ',' << it->y << ',' << it->p << '\n';
                    }
                }
            } catch (const std::exception& ex) {
                callback_error_msg = ex.what();
                callback_error.store(true, std::memory_order_release);
            } catch (...) {
                callback_error_msg = "Unknown error in conversion callback";
                callback_error.store(true, std::memory_order_release);
            }
        });
    // Query total duration so the polling loop can report progress. Files
    // without OfflineStreamingControl (e.g. live streams — though convert is
    // offline-only) simply leave duration at 0 and no progress is emitted.
    Metavision::timestamp duration_us = 0;
    try {
        auto& osc = cam.offline_streaming_control();
        if (osc.is_ready()) duration_us = osc.get_duration();
    } catch (...) {}

    cam.start();
    while (!cancel_) {
        if (callback_error.load(std::memory_order_acquire)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!cam.is_running()) break;
        // Report progress based on the last decoded timestamp. Emitting from
        // a std::thread is safe: AutoConnection queues the call to the
        // FileConverter's thread (the main thread).
        if (duration_us > 0) {
            try {
                const auto last = cam.get_last_timestamp();
                if (last > 0) {
                    double r = static_cast<double>(last)
                               / static_cast<double>(duration_us);
                    if (r < 0) r = 0; else if (r > 1.0) r = 1.0;
                    emit progress(r);
                }
            } catch (...) {}
        }
    }
    try { cam.stop(); } catch (...) {}
    cam.cd().remove_callback(id);
    // hdf5->close() may throw if the ECF compression plugin is missing.
    if (hdf5) {
        try {
            hdf5->close();
        } catch (const std::exception& ex) {
            callback_error_msg = ex.what();
            callback_error.store(true, std::memory_order_release);
        } catch (...) {
            callback_error_msg = "Unknown error closing HDF5 file";
            callback_error.store(true, std::memory_order_release);
        }
    }
    if (csvs) csvs->flush();
    if (csvf) csvf->close();

    if (callback_error.load(std::memory_order_acquire)) {
        QMetaObject::invokeMethod(this, [this, msg = callback_error_msg]() {
            emit failed(msg.empty() ? tr("Conversion failed: error in streaming callback.")
                                    : QString::fromUtf8(msg.c_str()));
        }, Qt::QueuedConnection);
        return;
    }

    // Distinguish cancel from completion: a cancelled run must not report
    // success (the output file is partial). Delete the partial file so the
    // user can't mistake it for a valid recording (audit §六-E4).
    if (cancel_) {
        QFile::remove(dst);
        QMetaObject::invokeMethod(this, [this]() {
            emit failed(tr("Conversion cancelled."));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, dst]() {
        emit progress(1.0);
        emit completed(dst);
    }, Qt::QueuedConnection);
}

void FileConverter::run_cut(const QString& src, const QString& dst,
                            Metavision::timestamp start_us, Metavision::timestamp end_us) {
    Metavision::Camera cam;
    try {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        cam = Metavision::Camera::from_file(src.toStdString(), hints);
    } catch (const Metavision::CameraException& e) {
        QMetaObject::invokeMethod(this, [this, msg = QString::fromUtf8(e.what())]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }

    // RAWEvt2EventFileWriter needs the sensor geometry. Wrap in try/catch
    // since some file formats may not expose a geometry facility (causing
    // an uncaught exception → std::terminate → crash).
    int w = 0, h = 0;
    try {
        const auto& g = cam.geometry();
        w = g.get_width();
        h = g.get_height();
    } catch (const std::exception& e) {
        QMetaObject::invokeMethod(this, [this, msg = tr("Source has no geometry: %1")
                                                          .arg(QString::fromUtf8(e.what()))]() {
            emit failed(msg);
        }, Qt::QueuedConnection);
        return;
    }
    Metavision::RAWEvt2EventFileWriter writer(w, h, dst.toStdString());
    bool seeked = false;
    try {
        if (start_us > 0) {
            auto& osc = cam.offline_streaming_control();
            if (osc.is_ready()) {
                // osc.seek() returns bool: true if the seek succeeded. Only
                // trust the seek when it returns true; otherwise fall back to
                // the lower-bound filter in the callback to drop early events.
                seeked = osc.seek(start_us);
            }
        }
    } catch (const Metavision::CameraException&) {
        // File doesn't support seeking — proceed from the start and rely on
        // the lower-bound filter below to drop events before start_us.
    }

    // Signal to the polling loop that we've already written the last event
    // we want, so it can stop early instead of consuming the whole file.
    std::atomic<bool> reached_end{false};
    std::atomic<bool> callback_error{false};
    auto id = cam.cd().add_callback(
        [&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (cancel_) return;
            try {
                // Lower bound: drop events before start_us when seek was not
                // available (or not supported by this file).
                auto it_begin = b;
                if (!seeked && start_us > 0) {
                    while (it_begin != e && it_begin->t < start_us) ++it_begin;
                    if (it_begin == e) return;
                }
                // Upper bound: stop writing once we pass end_us.
                if (end_us > 0 && it_begin != e && (e - 1)->t > end_us) {
                    auto it = it_begin;
                    while (it != e && it->t <= end_us) ++it;
                    writer.add_events(it_begin, it);
                    reached_end.store(true, std::memory_order_release);
                } else {
                    writer.add_events(it_begin, e);
                }
            } catch (const std::exception& ex) {
                // Store error for the polling loop to pick up
                callback_error.store(true, std::memory_order_release);
            } catch (...) {}
        });
    // Progress denominator: the time span we're actually keeping.
    const Metavision::timestamp span_us =
        (end_us > start_us) ? (end_us - start_us) : 0;

    cam.start();
    while (!cancel_) {
        if (callback_error.load(std::memory_order_acquire)) {
            try { cam.stop(); } catch (...) {}
            break;
        }
        if (reached_end.load(std::memory_order_acquire)) {
            try { cam.stop(); } catch (...) {}
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!cam.is_running()) break;
        // Report progress relative to the [start_us, end_us] window.
        if (span_us > 0) {
            try {
                const auto last = cam.get_last_timestamp();
                if (last > start_us) {
                    double r = static_cast<double>(last - start_us)
                               / static_cast<double>(span_us);
                    if (r < 0) r = 0; else if (r > 1.0) r = 1.0;
                    emit progress(r);
                }
            } catch (...) {}
        }
    }
    try { cam.stop(); } catch (...) {}
    cam.cd().remove_callback(id);
    writer.close();

    if (callback_error.load(std::memory_order_acquire)) {
        QMetaObject::invokeMethod(this, [this]() {
            emit failed(tr("Cut failed: error in streaming callback."));
        }, Qt::QueuedConnection);
        return;
    }

    if (cancel_) {
        // Partial RAW cut — delete it (audit §六-E4).
        QFile::remove(dst);
        QMetaObject::invokeMethod(this, [this]() {
            emit failed(tr("Cut cancelled."));
        }, Qt::QueuedConnection);
        return;
    }
    QMetaObject::invokeMethod(this, [this, dst]() {
        emit progress(1.0);
        emit completed(dst);
    }, Qt::QueuedConnection);
}

} // namespace gui
