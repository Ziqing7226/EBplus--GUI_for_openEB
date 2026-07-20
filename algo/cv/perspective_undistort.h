// algo/cv/perspective_undistort.h — Perspective undistortion via LUT.
//
// Implements design §4.3.20 (jAER SingleCameraCalibration LUT undistortion).
// Given camera intrinsics (K) and distortion coefficients, remaps event
// coordinates into the undistorted image plane. A lookup table (LUT) is
// precomputed once via cv::undistortPoints for O(1) runtime remapping; the LUT
// may be disabled to fall back to per-event undistortPoints.
//
// Precondition (design §4.5.1): intrinsic calibration must be completed and
// supplied via set_calibration() before precompute_lut()/process(). Header-only.

#ifndef GUI_ALGO_CV_PERSPECTIVE_UNDISTORT_H
#define GUI_ALGO_CV_PERSPECTIVE_UNDISTORT_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>
// OpenCV 5.0 moved cv::undistortPoints from calib3d to the new geometry
// module. The calib3d.hpp shim shipped by some 5.x packagings (e.g. Homebrew)
// does not include geometry, so undistortPoints goes missing. Include
// geometry.hpp directly on v5+; fall back to calib3d.hpp on v4.x.
#if CV_MAJOR_VERSION >= 5
#include <opencv2/geometry.hpp>
#else
#include <opencv2/calib3d.hpp>
#endif

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Perspective undistortion of event coordinates via a precomputed LUT.
class PerspectiveUndistort {
public:
    PerspectiveUndistort(bool use_lut = true,
                         bool undistort = true,
                         bool rectify = false)
        : use_lut_(use_lut),
          undistort_(undistort),
          rectify_(rectify),
          R_(cv::Mat::eye(3, 3, CV_64F)) {}

    /// @brief Supplies the intrinsic camera matrix and distortion coefficients.
    /// @param K 3x3 camera matrix (CV_64F).
    /// @param dist_coeffs distortion vector (e.g. 1x5 [k1,k2,p1,p2,k3]).
    void set_calibration(const cv::Mat& K, const cv::Mat& dist_coeffs) {
        K_ = K.clone();
        dist_coeffs_ = dist_coeffs.clone();
    }

    /// @brief Sets the image size and, when use_lut is on, precomputes the LUT.
    /// Must be called after set_calibration() when use_lut is enabled.
    void precompute_lut(int width, int height) {
        width_ = width;
        height_ = height;
        if (!use_lut_) return;
        const std::size_t n = static_cast<std::size_t>(width) * height;
        lut_x_.assign(n, 0.0f);
        lut_y_.assign(n, 0.0f);
        std::vector<cv::Point2f> src;
        src.reserve(n);
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                src.emplace_back(static_cast<float>(x), static_cast<float>(y));
            }
        }
        std::vector<cv::Point2f> dst;
        cv::undistortPoints(src, dst, K_, dist_coeffs_,
                            rectify_ ? R_ : cv::Mat(), K_);
        for (std::size_t i = 0; i < dst.size(); ++i) {
            lut_x_[i] = dst[i].x;
            lut_y_[i] = dst[i].y;
        }
    }

    /// @brief Remaps event coordinates in-place into the undistorted plane.
    /// Out-of-bounds results are clamped to the image extent. When zoom != 1.0,
    /// coordinates are scaled around the image center after undistortion.
    void process(MutableEventPacket& packet) {
        if (!undistort_) return;
        if (K_.empty() || dist_coeffs_.empty()) return;
        if (use_lut_ && lut_x_.empty()) return;
        if (width_ <= 0 || height_ <= 0) return;
        const float xmax = static_cast<float>(width_ - 1);
        const float ymax = static_cast<float>(height_ - 1);
        const float cx = xmax * 0.5f;
        const float cy = ymax * 0.5f;
        for (Event& e : packet) {
            if (e.x >= width_ || e.y >= height_) continue;
            float ox = 0.0f;
            float oy = 0.0f;
            if (use_lut_) {
                const std::size_t idx =
                    static_cast<std::size_t>(e.y) * width_ + e.x;
                ox = lut_x_[idx];
                oy = lut_y_[idx];
            } else {
                std::vector<cv::Point2f> src{
                    cv::Point2f(static_cast<float>(e.x), static_cast<float>(e.y))};
                std::vector<cv::Point2f> dst;
                cv::undistortPoints(src, dst, K_, dist_coeffs_,
                                    rectify_ ? R_ : cv::Mat(), K_);
                ox = dst[0].x;
                oy = dst[0].y;
            }
            // Apply zoom around image center.
            if (zoom_ != 1.0f) {
                ox = cx + (ox - cx) * zoom_;
                oy = cy + (oy - cy) * zoom_;
            }
            if (ox < 0.0f) ox = 0.0f;
            else if (ox > xmax) ox = xmax;
            if (oy < 0.0f) oy = 0.0f;
            else if (oy > ymax) oy = ymax;
            e.x = static_cast<std::uint16_t>(ox);
            e.y = static_cast<std::uint16_t>(oy);
        }
    }

    // Parameter accessors ---------------------------------------------------
    bool use_lut() const { return use_lut_; }
    bool undistort() const { return undistort_; }
    bool rectify() const { return rectify_; }
    void set_use_lut(bool v) { use_lut_ = v; }
    void set_undistort(bool v) { undistort_ = v; }
    void set_rectify(bool v) { rectify_ = v; }
    void set_zoom(float z) { zoom_ = (z > 0.01f) ? z : 1.0f; }
    float zoom() const { return zoom_; }

    void reset() {
        lut_x_.clear();
        lut_y_.clear();
        K_.release();
        dist_coeffs_.release();
        width_ = 0;
        height_ = 0;
        zoom_ = 1.0f;
    }

private:
    bool use_lut_;
    bool undistort_;
    bool rectify_;
    float zoom_{1.0f};
    cv::Mat K_;
    cv::Mat dist_coeffs_;
    cv::Mat R_;   ///< Rectification rotation (identity by default).
    int width_{0};
    int height_{0};
    std::vector<float> lut_x_;
    std::vector<float> lut_y_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_PERSPECTIVE_UNDISTORT_H
