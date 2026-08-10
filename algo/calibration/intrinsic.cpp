// algo/calibration/intrinsic.cpp

#include "intrinsic.h"

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/core/persistence.hpp>

#include "screwhead_detect.h"

namespace gui_algo {

IntrinsicCalibration::IntrinsicCalibration() = default;
IntrinsicCalibration::~IntrinsicCalibration() = default;

void IntrinsicCalibration::set_pattern(CalibrationPattern pattern,
                                       int cols, int rows,
                                       float square_size_mm) {
    // For chessboard, (cols, rows) IS the OpenCV inner-corner count — the same
    // convention as cv::findChessboardCorners' patternSize, the wizard's UI
    // ("Inner corners") and ChessboardDisplay (which draws (cols+1)×(rows+1)
    // squares). (Previously this subtracted 1 here, so a 9×6 board on screen
    // was searched as 8×5 and detection always failed.)
    // For circle grids, the count is the number of circles per row/column.
    cv::Size new_bs = cv::Size(std::max(cols, 1), std::max(rows, 1));
    // The wizard refreshes set_pattern on every capture/run so the user can
    // change pattern/dims/square mid-session. Each frame's object_points_ is
    // frozen at capture time via make_object_grid(), so mixing different
    // board geometries would feed cv::calibrateCamera point sets of
    // inconsistent size/coordinate-system and either throw or return a
    // wrong result. Clear accumulated observations when the geometry changes.
    if (pattern_ != pattern || board_size_ != new_bs || square_size_mm_ != square_size_mm) {
        image_points_.clear();
        object_points_.clear();
        image_size_ = cv::Size(0, 0);
    }
    pattern_ = pattern;
    board_size_ = new_bs;
    square_size_mm_ = square_size_mm;
}

void IntrinsicCalibration::set_dot_gap(int dot_gap) {
    dot_gap_ = std::clamp(dot_gap, 1, 3);
}

std::vector<cv::Point3f> IntrinsicCalibration::make_object_grid() const {
    std::vector<cv::Point3f> pts;
    pts.reserve(static_cast<std::size_t>(board_size_.width) *
                static_cast<std::size_t>(board_size_.height));
    if (pattern_ == CalibrationPattern::ScrewHeadGrid) {
        // Asymmetric grid: positions (2c+(r&1), r)·d, where d is the half-cell.
        // The user measures the same-row (or same-column) adjacent marker
        // distance = 2d, so d = square_size_mm / 2. No √2: same-row and
        // same-column neighbours are both 2d; the diagonal (d·√2) is closer but
        // is NOT what the user measures.
        const float d = square_size_mm_ / 2.0f;
        for (int r = 0; r < board_size_.height; ++r) {
            for (int c = 0; c < board_size_.width; ++c) {
                pts.emplace_back((2 * c + (r & 1)) * d, r * d, 0.0f);
            }
        }
    } else {
        for (int r = 0; r < board_size_.height; ++r) {
            for (int c = 0; c < board_size_.width; ++c) {
                pts.emplace_back(c * square_size_mm_, r * square_size_mm_, 0.0f);
            }
        }
    }
    return pts;
}

DetectionResult IntrinsicCalibration::detect_only(const cv::Mat& frame, bool annotate) {
    DetectionResult result;
    if (frame.empty()) {
        result.found = false;
        return result;
    }
    if (image_size_.area() == 0) {
        image_size_ = frame.size();
    }

    // ScrewHeadGrid takes the three-valued colour frame directly (polarity is
    // encoded in colour); the OpenCV patterns below need grayscale.
    if (pattern_ == CalibrationPattern::ScrewHeadGrid) {
        return detect_screwheads(frame, board_size_.width, board_size_.height,
                                 dot_gap_, annotate);
    }

    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<cv::Point2f> corners;
    bool found = false;

    switch (pattern_) {
        case CalibrationPattern::Chessboard:
            found = cv::findChessboardCorners(gray, board_size_, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found) {
                cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
                                      30, 0.01));
            }
            break;
        case CalibrationPattern::CircleGrid:
            found = cv::findCirclesGrid(gray, board_size_, corners);
            break;
        case CalibrationPattern::ScrewHeadGrid:
            break;  // handled above
    }

    result.found = found;
    result.points = corners;

    if (annotate) {
        if (frame.channels() == 1) {
            cv::cvtColor(frame, result.image, cv::COLOR_GRAY2BGR);
        } else {
            result.image = frame.clone();
        }
        cv::drawChessboardCorners(result.image, board_size_, corners, found);
    }
    // NOTE: no accumulation here — the caller decides via accept().
    return result;
}

void IntrinsicCalibration::accept(const std::vector<cv::Point2f>& points) {
    image_points_.push_back(points);
    object_points_.push_back(make_object_grid());
}

DetectionResult IntrinsicCalibration::add_frame(const cv::Mat& frame, bool annotate) {
    DetectionResult result = detect_only(frame, annotate);
    if (result.found) {
        accept(result.points);
    }
    return result;
}

bool IntrinsicCalibration::is_duplicate_pose(
    const std::vector<cv::Point2f>& points, double threshold_px) const {
    if (points.empty()) return false;
    for (const auto& stored : image_points_) {
        if (stored.size() != points.size()) continue;
        double sum = 0.0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const double dx = stored[i].x - points[i].x;
            const double dy = stored[i].y - points[i].y;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        const double mean = sum / static_cast<double>(points.size());
        if (mean < threshold_px) return true;
    }
    return false;
}

IntrinsicResult IntrinsicCalibration::run() {
    IntrinsicResult result;
    if (image_points_.size() < 3) {
        result.error = "Need at least 3 valid frames; got " +
                       std::to_string(image_points_.size());
        return result;
    }
    if (image_size_.area() == 0) {
        result.error = "Image size not yet known";
        return result;
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    try {
        double rms = cv::calibrateCamera(object_points_, image_points_,
            image_size_, K, dist, rvecs, tvecs);
        result.rms = rms;
        result.K = K;
        result.dist_coeffs = dist;
        result.rvecs = std::move(rvecs);
        result.tvecs = std::move(tvecs);
        result.frames_used = image_points_.size();
        // Set ok only after every potentially-throwing operation succeeded
        // (§四-低11): previously ok was set before the LUT precompute, so a
        // throwing step left ok==true together with an error message.
        result.ok = true;
    } catch (const cv::Exception& e) {
        result.error = e.what();
    } catch (const std::exception& e) {
        result.error = e.what();
    }
    return result;
}

void IntrinsicCalibration::reset() {
    image_points_.clear();
    object_points_.clear();
    image_size_ = cv::Size(0, 0);
}

bool load_intrinsics_yml(const std::string& path,
                         cv::Mat& K, cv::Mat& dist_coeffs,
                         cv::Size& image_size) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    cv::Mat k_in, dist_in;
    int w = 0, h = 0;
    fs["image_width"]  >> w;
    fs["image_height"] >> h;
    fs["camera_matrix"]           >> k_in;
    fs["distortion_coefficients"] >> dist_in;
    fs.release();
    if (k_in.empty() || dist_in.empty() || w <= 0 || h <= 0) return false;
    // Normalise to the canonical types used by IntrinsicCalibration /
    // cv::undistortPoints (CV_64F for K, CV_64F 1xN for dist).
    if (k_in.type() != CV_64F) {
        cv::Mat tmp;
        k_in.convertTo(tmp, CV_64F);
        k_in = tmp;
    }
    if (dist_in.type() != CV_64F) {
        cv::Mat tmp;
        dist_in.convertTo(tmp, CV_64F);
        dist_in = tmp;
    }
    if (k_in.rows != 3 || k_in.cols != 3) return false;
    K = k_in;
    dist_coeffs = dist_in;
    image_size = cv::Size(w, h);
    return true;
}

} // namespace gui_algo
