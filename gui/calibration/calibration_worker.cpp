// gui/calibration/calibration_worker.cpp — see header (Phase 4-2).

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
// this fraction of the frame area — the camera is too far for the grid to
// condition cv::calibrateCamera well ("coverage insufficient", Phase 4).
constexpr double kMinCoverageRatio = 0.10;
// Two poses whose detected points differ by less than this mean Euclidean
// distance (px) are treated as the same pose (duplicate, Phase 4).
constexpr double kDuplicateThresholdPx = 10.0;
} // namespace

CalibrationWorker::CalibrationWorker(QObject* parent)
    : QObject(parent),
      intrinsic_(std::make_unique<gui_algo::IntrinsicCalibration>()) {
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::AsymmetricCircles,
                            4, 11, 5.0f);
}

CalibrationWorker::~CalibrationWorker() = default;

void CalibrationWorker::configure(int cols, int rows,
                                  double square_size_mm, int target_frames) {
    intrinsic_->set_pattern(gui_algo::CalibrationPattern::AsymmetricCircles,
                            cols, rows, static_cast<float>(square_size_mm));
    target_ = static_cast<std::size_t>(std::max(1, target_frames));
}

void CalibrationWorker::reset() {
    intrinsic_->reset();
}

void CalibrationWorker::process_frame(const cv::Mat& frame) {
    if (frame.empty()) {
        emit frame_rejected(tr("Empty capture frame."));
        return;
    }

    auto res = intrinsic_->detect_only(frame, true);

    if (!res.found) {
        emit frame_rejected(tr("Circle grid not detected — re-aim and try again."));
        return;
    }

    // Coverage: detected grid bbox vs frame area.
    if (!res.points.empty()) {
        float xmin = res.points[0].x, xmax = xmin;
        float ymin = res.points[0].y, ymax = ymin;
        for (const auto& p : res.points) {
            xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
            ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
        }
        const double bbox_area =
            static_cast<double>(xmax - xmin) * static_cast<double>(ymax - ymin);
        const double frame_area =
            static_cast<double>(frame.cols) * static_cast<double>(frame.rows);
        if (frame_area > 0.0 && bbox_area / frame_area < kMinCoverageRatio) {
            emit frame_rejected(tr("Coverage too low — move the camera closer."));
            return;
        }
    }

    if (intrinsic_->is_duplicate_pose(res.points, kDuplicateThresholdPx)) {
        emit frame_rejected(tr("Duplicate pose — move the camera to a new angle."));
        return;
    }

    intrinsic_->accept(res.points);
    const std::size_t got = intrinsic_->frame_count();

    // Annotated BGR Mat → QImage (deep copy via .copy(), safe to pass across
    // threads to the GUI thread).
    QImage annotated;
    if (!res.image.empty()) {
        cv::Mat rgb;
        cv::cvtColor(res.image, rgb, cv::COLOR_BGR2RGB);
        annotated = QImage(rgb.data, rgb.cols, rgb.rows,
                           static_cast<int>(rgb.step),
                           QImage::Format_RGB888).copy();
    }
    emit frame_accepted(annotated, got, target_);
    if (got >= target_) {
        emit capture_complete(got);
    }
}

void CalibrationWorker::run_calibration() {
    // cv::calibrateCamera (bundle adjustment) runs on the worker thread so the
    // GUI stays responsive. The result is cached for export_to().
    last_result_ = intrinsic_->run();
    emit calibration_done(last_result_.ok, last_result_.rms,
                          static_cast<int>(last_result_.frames_used),
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
    // Phase 4: auto-mkdir the parent directory so export to a fresh path
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
        fs.release();
        emit export_done(true, path);
    } catch (const std::exception& e) {
        emit export_done(false, QString::fromUtf8(e.what()));
    }
}

} // namespace gui
