// gui/calibration/calibration_worker.h — off-GUI-thread calibration work
// (Phase 4).
//
// Owns an IntrinsicCalibration and runs circle-grid detection (+, in 4-3,
// cv::calibrateCamera) on a dedicated QThread so the GUI stays responsive
// while OpenCV blocks for tens of milliseconds per frame. The wizard submits
// rendered binary frames via the submit_frame signal; the worker emits
// frame_accepted / frame_rejected / capture_complete back to the GUI thread.
//
// Phase 4 uses only the AsymmetricCircles pattern, so the pattern type is
// fixed here (the configure() slot takes geometry only) — this also avoids
// having to register the CalibrationPattern enum for queued connections.

#ifndef GUI_CALIBRATION_CALIBRATION_WORKER_H
#define GUI_CALIBRATION_CALIBRATION_WORKER_H

#include <cstddef>
#include <QImage>
#include <QObject>
#include <memory>

#include <opencv2/core.hpp>

#include "algo/calibration/intrinsic.h"

class QString;

namespace gui {

class CalibrationWorker : public QObject {
    Q_OBJECT
public:
    explicit CalibrationWorker(QObject* parent = nullptr);
    ~CalibrationWorker();

public slots:
    /// @brief Configures the board geometry + target frame count. Runs on the
    /// worker thread (queued from the GUI). IntrinsicCalibration clears
    /// accumulated observations when the geometry changes.
    void configure(int cols, int rows, double square_size_mm, int target_frames);

    /// @brief Discards all accumulated observations.
    void reset();

    /// @brief Detects the asymmetric circle grid in @p frame and, if it passes
    /// the coverage + duplicate-pose checks, accepts it as an observation.
    /// Emits frame_accepted or frame_rejected; emits capture_complete once
    /// target_frames observations are accepted.
    void process_frame(const cv::Mat& frame);

    /// @brief Runs cv::calibrateCamera on the accumulated observations. Must
    /// be called only after capture_complete. Emits calibration_done.
    void run_calibration();

    /// @brief Writes the last calibration result to @p path as OpenCV YAML,
    /// creating the parent directory if needed (auto-mkdir). Emits
    /// export_done.
    void export_to(const QString& path);

signals:
    void frame_accepted(QImage annotated, std::size_t accepted, std::size_t target);
    void frame_rejected(QString reason);
    void capture_complete(std::size_t accepted);
    void calibration_done(bool ok, double rms, int frames_used, QString error);
    void export_done(bool ok, QString message);

private:
    std::unique_ptr<gui_algo::IntrinsicCalibration> intrinsic_;
    gui_algo::IntrinsicResult last_result_;
    std::size_t target_{15};
};

} // namespace gui

#endif // GUI_CALIBRATION_CALIBRATION_WORKER_H
