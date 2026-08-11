// gui/calibration/calibration_worker.h — off-GUI-thread calibration work
// (Zhou's Method).
//
// Owns an IntrinsicCalibration and runs screw-head grid detection (+, in 4-3,
// cv::calibrateCamera) on a dedicated QThread so the GUI stays responsive
// while OpenCV blocks for tens of milliseconds per frame. The wizard submits
// rendered three-valued colour frames via the review_frame_requested signal;
// the worker detects and emits review_ready (with the detected cross/ring
// features) so the wizard can show its capture-review dialog. If the user
// accepts, accept_review() commits the observation (coverage + duplicate-pose
// checks gate it, as before); the worker emits frame_accepted /
// frame_rejected / capture_complete back to the GUI. Re-detect with relaxed
// ring parameters runs on the stored frame.
//
// The pattern is fixed to ScrewHeadGrid (asymmetric 6×5); the configure() slot
// takes scale/target/dot_gap only — this also avoids registering the
// CalibrationPattern enum for queued connections.

#ifndef GUI_CALIBRATION_CALIBRATION_WORKER_H
#define GUI_CALIBRATION_CALIBRATION_WORKER_H

#include <cstddef>
#include <QImage>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>
#include <memory>

#include <opencv2/core.hpp>

#include "algo/calibration/intrinsic.h"

namespace gui {

/// @brief Result of a capture-review detection, shown by the wizard's review
/// dialog. Carries the RAW captured frame plus every detected cross/ring
/// feature so the dialog can draw blue crosses and red circles and let the
/// user re-detect with relaxed parameters before committing.
struct CaptureReview {
    bool found{false};
    QImage frame;                 ///< Raw captured frame (RGB), no annotation
    QVector<QPointF> crosses;     ///< Detected cross centres (blue markers)
    QVector<QPointF> rings;       ///< Detected ring centres (red circles)
    QVector<float> ring_radii;    ///< Ring radii in px (parallel to rings)
    QString reason;               ///< Explanation when found == false
};

class CalibrationWorker : public QObject {
    Q_OBJECT
public:
    explicit CalibrationWorker(QObject* parent = nullptr);
    ~CalibrationWorker();

public slots:
    /// @brief Configures the board scale + target frame count + dot gap. Runs on
    /// the worker thread (queued from the GUI). The grid is fixed at 6×5;
    /// IntrinsicCalibration clears accumulated observations when the scale changes.
    void configure(double square_size_mm, int target_frames, int dot_gap);

    /// @brief Discards all accumulated observations.
    void reset();

    /// @brief Runs screw-head detection on @p frame with the current ring
    /// parameters and emits review_ready (first detection for a capture). The
    /// frame is stored so the user can re-detect with different ring
    /// parameters before accepting.
    void review_frame(const cv::Mat& frame);

    /// @brief Updates the ring/circle detection parameters and re-runs
    /// detection on the stored frame. Emits review_ready again — the wizard's
    /// capture-review dialog uses this when the user tweaks a parameter and
    /// clicks Re-detect.
    void re_detect(double cover_frac, int min_pixels, int search_margin);

    /// @brief Commits the last review result as a calibration observation.
    /// The coverage + duplicate-pose checks that previously ran inside
    /// process_frame() now gate the user-accepted result. Emits
    /// frame_accepted / frame_rejected; capture_complete once the target is
    /// reached.
    void accept_review();

    /// @brief Removes the most recently accepted observation, undoing the last
    /// accept() from accept_review(). Used by the wizard's "Delete this
    /// capture" button so the user can discard a bad frame after inspecting the
    /// annotated preview — that frame is then excluded from the final intrinsic
    /// calculation. Emits frame_deleted with the remaining observation count.
    /// One delete per accept: the wizard gates this so a second consecutive
    /// delete cannot remove the prior frame.
    void delete_last_capture();

    /// @brief Runs cv::calibrateCamera on the accumulated observations. Must
    /// be called only after capture_complete. Emits calibration_done.
    void run_calibration();

    /// @brief Writes the last calibration result to @p path as OpenCV YAML,
    /// creating the parent directory if needed (auto-mkdir). Emits
    /// export_done.
    void export_to(const QString& path);

signals:
    /// @brief A capture-review detection result (first detect or a re-detect).
    void review_ready(const CaptureReview& review);
    void frame_accepted(QImage annotated, std::size_t accepted, std::size_t target);
    void frame_rejected(QString reason);
    /// @brief Emitted after delete_last_capture() with the remaining
    /// observation count. The wizard uses this to roll back the progress bar
    /// and preview to match the worker's authoritative state.
    void frame_deleted(std::size_t remaining);
    void capture_complete(std::size_t accepted);
    void calibration_done(bool ok, double rms, int frames_used, QString error);
    void export_done(bool ok, QString message);

private:
    /// @brief Runs detect_only on the stored review frame with the current ring
    /// params and emits review_ready. Shared by review_frame() and re_detect().
    void run_review_detect();

    std::unique_ptr<gui_algo::IntrinsicCalibration> intrinsic_;
    gui_algo::IntrinsicResult last_result_;
    std::size_t target_{15};
    /// Last frame submitted for review (kept for re-detect with new params).
    cv::Mat review_frame_;
    /// Current ring/circle detection parameters (relaxable via re_detect).
    gui_algo::RingParams ring_params_;
    /// Last detection result (kept for accept_review()).
    gui_algo::DetectionResult last_detect_;
};

} // namespace gui

Q_DECLARE_METATYPE(gui::CaptureReview)

#endif // GUI_CALIBRATION_CALIBRATION_WORKER_H
