// gui/recorder/recorder_controller.h — real-time RAW recording (design §3.3.1).
//
// Wraps I_EventsStream::log_raw_data() to record the live camera stream to a
// RAW file. Recording is only available for live cameras (not file playback).
// A QTimer reports the elapsed recording time once per second.

#ifndef GUI_RECORDER_RECORDER_CONTROLLER_H
#define GUI_RECORDER_RECORDER_CONTROLLER_H

#include <QObject>
#include <QPointer>
#include <QString>
#include <QTimer>
#include <chrono>

namespace gui {

class CameraController;

class RecorderController : public QObject {
    Q_OBJECT
public:
    explicit RecorderController(QObject* parent = nullptr);
    ~RecorderController();

    /// @brief Starts recording the live camera stream to @p path (RAW format).
    /// @return true on success.
    bool start(CameraController* controller, const QString& path);
    void stop();

    bool is_recording() const { return recording_; }

signals:
    void recording_started(const QString& path);
    void recording_stopped(const QString& path);
    void elapsed(std::chrono::seconds s);
    void error(const QString& msg);

private:
    bool recording_{false};
    QString path_;
    QTimer timer_;           ///< Emits elapsed() once per second.
    QTimer flush_timer_;     ///< Calls I_EventsStream::get_latest_raw_data() to flush buffers.
    QPointer<CameraController> controller_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace gui

#endif // GUI_RECORDER_RECORDER_CONTROLLER_H
