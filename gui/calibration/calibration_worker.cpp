// gui/calibration/calibration_worker.cpp — see header.

#include "calibration_worker.h"

#include <algorithm>

#include <QDir>
#include <QFileInfo>
#include <QString>

#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

namespace gui {

namespace {
// A capture is rejected if the detected grid's bounding box covers less than
// this fraction of the frame area — a much smaller grid cannot condition
// cv::calibrateCamera well ("coverage insufficient"). 4% still admits a
// 300×210 px board on a 1280×720 sensor (~30 px squares), which is the
// practical floor for reliable quad linking on blink frames.
constexpr double kMinCoverageRatio = 0.04;
// Two poses whose detected points differ by less than this mean Euclidean
// distance (px) are treated as the same pose (duplicate).
constexpr double kDuplicateThresholdPx = 10.0;
} // namespace

CalibrationWorker::CalibrationWorker(QObject* parent)
    : QObject(parent),
      intrinsic_(std::make_unique<gui_algo::IntrinsicCalibration>()) {
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::Chessboard,
                            9, 6, 20.0f);
    // Permissive polarity ratios: a real blink frame carries plenty of
    // single-polarity noise (screen backlight, edges); the board fill — not
    // the gate — decides.
    blink_params_.ratio_on = 1.0;
    blink_params_.ratio_off = 1.0;
}

CalibrationWorker::~CalibrationWorker() = default;

void CalibrationWorker::configure(double square_size_mm, int target_frames) {
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::Chessboard,
                            9, 6, static_cast<float>(square_size_mm));
    target_ = static_cast<std::size_t>(std::max(1, target_frames));
}

void CalibrationWorker::reset() {
    intrinsic_->reset();
}

void CalibrationWorker::delete_last_capture() {
    intrinsic_->remove_last_frame();
    emit frame_deleted(intrinsic_->frame_count());
}

void CalibrationWorker::process_capture(const BlinkCapture& capture) {
    if (capture.on_cnt.empty() || capture.off_cnt.empty()) {
        emit frame_rejected(tr("Empty capture frame."));
        return;
    }

    const gui_algo::BlinkFrame bf = gui_algo::build_blink_frame_from_counts(
        capture.on_cnt, capture.off_cnt, blink_params_);
    if (!bf.valid) {
        emit frame_rejected(tr("Not enough blinking pixels (board area: %1, "
                               "min %2) — ensure the chessboard fills the view "
                               "and is steady.").arg(bf.both)
                                .arg(blink_params_.min_blink_pixels));
        return;
    }

    last_detect_ = intrinsic_->detect_only(bf.frame, true);
    if (!last_detect_.found || last_detect_.points.empty()) {
        emit frame_rejected(tr("Chessboard not detected — ensure the blinking "
                               "chessboard fills a large part of the view and "
                               "is in focus."));
        return;
    }

    // Coverage: detected grid bbox vs frame area.
    {
        float xmin = last_detect_.points[0].x, xmax = xmin;
        float ymin = last_detect_.points[0].y, ymax = ymin;
        for (const auto& p : last_detect_.points) {
            xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        }
        const double bbox_area =
            static_cast<double>(xmax - xmin) * static_cast<double>(ymax - ymin);
        const double frame_area =
            static_cast<double>(bf.frame.cols) * static_cast<double>(bf.frame.rows);
        if (frame_area > 0.0 && bbox_area / frame_area < kMinCoverageRatio) {
            emit frame_rejected(tr("Coverage too low — move the camera closer."));
            return;
        }
    }

    if (intrinsic_->is_duplicate_pose(last_detect_.points, kDuplicateThresholdPx)) {
        emit frame_rejected(tr("Duplicate pose — move the camera to a new angle."));
        return;
    }

    intrinsic_->accept(last_detect_.points);
    const std::size_t got = intrinsic_->frame_count();

    // The accepted corners (sensor px) ride along for the wizard's coverage
    // overlay (convex hull over all accepted views).
    QVector<QPointF> accepted_points;
    accepted_points.reserve(static_cast<qsizetype>(last_detect_.points.size()));
    for (const auto& p : last_detect_.points) {
        accepted_points.append(QPointF(p.x, p.y));
    }

    // Annotated BGR Mat → QImage (deep copy via .copy(), safe to pass across
    // threads to the GUI thread).
    QImage annotated;
    if (!last_detect_.image.empty()) {
        cv::Mat rgb;
        cv::cvtColor(last_detect_.image, rgb, cv::COLOR_BGR2RGB);
        annotated = QImage(rgb.data, rgb.cols, rgb.rows,
                           static_cast<int>(rgb.step),
                           QImage::Format_RGB888).copy();
    }
    emit frame_accepted(annotated, got, target_, accepted_points);
    if (got >= target_) {
        emit capture_complete(got);
    }
}

void CalibrationWorker::run_calibration() {
    // Two-pass cv::calibrateCamera (bundle adjustment) runs on the worker
    // thread so the GUI stays responsive. The result is cached for export_to().
    last_result_ = intrinsic_->run();
    emit calibration_done(last_result_.ok, last_result_.rms,
                          static_cast<int>(last_result_.frames_used),
                          static_cast<int>(last_result_.removed_frames),
                          QString::fromStdString(last_result_.error));
}

void CalibrationWorker::export_to(const QString& path) {
    if (path.isEmpty()) {
        emit export_done(false, tr("Empty path."));
        return;
    }
    if (!last_result_.ok) {
        emit export_done(false, tr("No successful calibration to export."));
        return;
    }
    // Auto-mkdir the parent directory so export to a fresh path
    // (e.g. ~/Documents/EBplus/calibration/intrinsic.yml) does not fail.
    const QFileInfo fi(path);
    const QDir dir = fi.dir();
    if (!dir.exists() && !dir.mkpath(".")) {
        emit export_done(false, tr("Could not create directory %1.").arg(dir.path()));
        return;
    }
    try {
        cv::FileStorage fs(path.toStdString(), cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            emit export_done(false, tr("Could not open %1 for writing.").arg(path));
            return;
        }
        fs << "image_width"  << intrinsic_->image_size().width;
        fs << "image_height" << intrinsic_->image_size().height;
        fs << "camera_matrix"           << last_result_.K;
        fs << "distortion_coefficients" << last_result_.dist_coeffs;
        fs << "rms" << last_result_.rms;
        fs << "kept_frames" << static_cast<int>(last_result_.kept_frames);
        fs << "removed_frames" << static_cast<int>(last_result_.removed_frames);
        fs << "per_view_rms_reprojection_errors" << last_result_.per_view_rms;
        fs.release();
        emit export_done(true, path);
    } catch (const std::exception& e) {
        emit export_done(false, QString::fromUtf8(e.what()));
    }
}

} // namespace gui
