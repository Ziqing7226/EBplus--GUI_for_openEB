// gui/calibration/calibration_worker.h — off-GUI-thread calibration work.
//
// Owns an IntrinsicCalibration and runs blinking-chessboard detection (+, on
// run_calibration, the two-pass cv::calibrateCamera) on a dedicated QThread so
// the GUI stays responsive while OpenCV blocks. The wizard submits the
// accumulated ON/OFF polarity masks (BlinkCapture) via the process_capture
// signal; the worker builds the binary blink frame with its blink parameters,
// detects the chessboard and — on success — runs the coverage + duplicate-pose
// checks and accumulates the observation directly (no capture-review dialog),
// emitting frame_accepted / frame_rejected back to the GUI.
//
// The pattern is fixed to the Blinking Chessboard (9×6 inner corners); the
// configure() slot takes square size + target frames only.

#ifndef GUI_CALIBRATION_CALIBRATION_WORKER_H
#define GUI_CALIBRATION_CALIBRATION_WORKER_H

#include <cstddef>
#include <QImage>
#include <QObject>
#include <QPointF>
#include <QString>
#include <QVector>
#include <memory>

#include <opencv2/core.hpp>

#include "algo/calibration/blinking_detect.h"
#include "algo/calibration/intrinsic.h"

namespace gui {

/// @brief Per-polarity event counts of one capture (CV_32S), sent to the
/// worker so the blink frame can be built with the current blink parameters.
/// Counts (not 0/1 masks) are used because the screen's backlight PWM gives
/// every on-screen pixel both polarities; the board's black squares stand out
/// only by their 2-3x higher event count.
struct BlinkCapture {
    cv::Mat on_cnt;   ///< Per-pixel ON event count
    cv::Mat off_cnt;  ///< Per-pixel OFF event count
};

class CalibrationWorker : public QObject {
    Q_OBJECT
public:
    explicit CalibrationWorker(QObject* parent = nullptr);
    ~CalibrationWorker();

public slots:
    /// @brief Configures the board square size + target frame count. Runs on
    /// the worker thread (queued from the GUI). The grid is fixed at 9×6;
    /// IntrinsicCalibration clears accumulated observations when the scale changes.
    void configure(double square_size_mm, int target_frames);

    /// @brief Discards all accumulated observations.
    void reset();

    /// @brief Builds the blink frame from the capture's polarity masks, runs
    /// chessboard detection, and on success commits the observation directly
    /// (coverage + duplicate-pose checks gate it). Emits frame_accepted /
    /// frame_rejected; capture_complete once the target is reached.
    void process_capture(const BlinkCapture& capture);

    /// @brief Removes the most recently accepted observation. Used by the
    /// wizard's "Delete this capture" button. Emits frame_deleted with the
    /// remaining observation count. One delete per accept: the wizard gates this.
    void delete_last_capture();

    /// @brief Runs the two-pass calibration on the accumulated observations.
    /// Must be called only after capture_complete. Emits calibration_done.
    void run_calibration();

    /// @brief Writes the last calibration result to @p path as OpenCV YAML,
    /// creating the parent directory if needed (auto-mkdir). Emits
    /// export_done.
    void export_to(const QString& path);

signals:
    /// @param points  the accepted view's corners (sensor px) — the wizard
    /// stacks them for the coverage overlay (convex hull over all views).
    void frame_accepted(QImage annotated, std::size_t accepted, std::size_t target,
                        QVector<QPointF> points);
    void frame_rejected(QString reason);
    /// @brief Emitted after delete_last_capture() with the remaining
    /// observation count.
    void frame_deleted(std::size_t remaining);
    void capture_complete(std::size_t accepted);
    void calibration_done(bool ok, double rms, int frames_used, int removed_frames, QString error);
    void export_done(bool ok, QString message);

private:
    std::unique_ptr<gui_algo::IntrinsicCalibration> intrinsic_;
    gui_algo::IntrinsicResult last_result_;
    std::size_t target_{15};
    /// Current blink-frame parameters (fixed defaults; permissive polarity
    /// ratios so the chessboard fill — not the gate — decides detection).
    gui_algo::BlinkParams blink_params_;
    /// Last detection result (kept for accept).
    gui_algo::DetectionResult last_detect_;
};

} // namespace gui

Q_DECLARE_METATYPE(gui::BlinkCapture)

#endif // GUI_CALIBRATION_CALIBRATION_WORKER_H
