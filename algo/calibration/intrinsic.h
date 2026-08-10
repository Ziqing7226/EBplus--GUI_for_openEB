// algo/calibration/intrinsic.h — intrinsic camera calibration (Zhang method).
//
// Phase 9 (design §4.5.1): detects chessboard / circle-grid corners in
// accumulated event frames, accumulates multi-pose observations, and runs
// cv::calibrateCamera to produce K, distCoeffs and an RMS reprojection error.
//
// The algorithm is frame-driven: the GUI wizard feeds accumulated grayscale
// frames via add_frame(). Detection runs synchronously on the caller's thread
// (cheap relative to calibration itself). Once enough poses are collected,
// run() performs the bundle adjustment.

#ifndef GUI_ALGO_CALIBRATION_INTRINSIC_H
#define GUI_ALGO_CALIBRATION_INTRINSIC_H

#include <cstddef>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace gui_algo {

/// @brief Pattern type for calibration board detection.
enum class CalibrationPattern {
    Chessboard,     ///< Standard chessboard (inner corners)
    CircleGrid,     ///< Regular circle grid
    ScrewHeadGrid   ///< Zhou's screw-head grid (dashed cross + solid ring),
                    ///< asymmetric 6×5 layout, polarity-verified detection
};

/// @brief Result of a single frame's corner detection.
struct DetectionResult {
    bool found{false};
    cv::Mat image;                 ///< Annotated copy (for wizard preview)
    std::vector<cv::Point2f> points; ///< Detected corner coordinates
};

/// @brief Result of the full intrinsic calibration.
struct IntrinsicResult {
    bool ok{false};
    double rms{0.0};               ///< Reprojection RMS (pixels)
    cv::Mat K;                     ///< 3x3 camera matrix
    cv::Mat dist_coeffs;           ///< 1x5 distortion [k1,k2,p1,p2,k3]
    std::vector<cv::Mat> rvecs;    ///< Per-frame rotation vectors
    std::vector<cv::Mat> tvecs;    ///< Per-frame translation vectors
    std::size_t frames_used{0};
    std::string error;             ///< Empty when ok==true
};

/// @brief Intrinsic calibration accumulator.
class IntrinsicCalibration {
public:
    IntrinsicCalibration();
    ~IntrinsicCalibration();

    /// @brief Configures the board geometry. Must be called before add_frame().
    void set_pattern(CalibrationPattern pattern,
                     int cols, int rows,
                     float square_size_mm);

    /// @brief Sets the dashed-cross dot-gap parameter (1/2/3) used by
    /// ScrewHeadGrid detection (dilate radius = dot_gap+1). No-op for other
    /// patterns. Stored only; does not affect object-point geometry.
    void set_dot_gap(int dot_gap);

    /// @brief Attempts to detect the calibration pattern in @p frame and, on
    /// success, accumulates the observation. Convenience wrapper around
    /// detect_only() + accept() for callers that want detect-and-keep in one
    /// call (preserves the original Phase 9 contract).
    /// @param frame Grayscale or BGR accumulated event frame.
    /// @param annotate If true, @p result.image contains a drawn preview.
    /// @return DetectionResult with found state and corner points.
    DetectionResult add_frame(const cv::Mat& frame, bool annotate = true);

    /// @brief Runs pattern detection only — does NOT accumulate. Lets the
    /// caller reject a frame (duplicate pose, insufficient coverage, …)
    /// before committing it via accept(). Records image_size_ on first call.
    DetectionResult detect_only(const cv::Mat& frame, bool annotate = true);

    /// @brief Commits an already-detected point set as a calibration
    /// observation. @p points must come from a successful detect_only() on a
    /// frame of the configured geometry.
    void accept(const std::vector<cv::Point2f>& points);

    /// @brief Returns true if @p points match (within @p threshold_px mean
    /// Euclidean distance) any already-accepted observation. Used by the
    /// Phase 4 wizard to reject duplicate-pose captures. Element-wise
    /// comparison assumes findCirclesGrid returns points in a consistent
    /// order for a given board pose.
    bool is_duplicate_pose(const std::vector<cv::Point2f>& points,
                           double threshold_px) const;

    /// @brief Runs cv::calibrateCamera on all collected observations.
    IntrinsicResult run();

    /// @brief Discards all collected observations.
    void reset();

    std::size_t frame_count() const { return image_points_.size(); }
    cv::Size image_size() const { return image_size_; }

    /// @brief The object-point grid for the currently configured geometry
    /// (read-only). Exposed so the wizard/tests can verify the configured
    /// board matches the displayed pattern.
    std::vector<cv::Point3f> object_grid() const { return make_object_grid(); }

private:
    CalibrationPattern pattern_{CalibrationPattern::Chessboard};
    cv::Size board_size_{0, 0};    ///< Inner-corner count for chessboard (== OpenCV patternSize), (cols, rows) for circles
    float square_size_mm_{1.0f};
    int dot_gap_{2};               ///< ScrewHeadGrid dashed-cross dot gap (1/2/3)
    cv::Size image_size_{0, 0};

    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<std::vector<cv::Point3f>> object_points_;

    std::vector<cv::Point3f> make_object_grid() const;
};

/// @brief Loads intrinsic calibration (K, distCoeffs, image_size) from a YAML
///        file written by CalibrationWizard::on_intrinsic_save() or any
///        OpenCV-compatible YAML with the same keys:
///          image_width, image_height, camera_matrix, distortion_coefficients
///        (rms optional). Used by the calibration wizard (round-trip) and by
///        the Preprocessor undistort stage (consumes the wizard's output).
/// @return true on success; on failure leaves outputs unchanged.
bool load_intrinsics_yml(const std::string& path,
                         cv::Mat& K, cv::Mat& dist_coeffs,
                         cv::Size& image_size);

} // namespace gui_algo

#endif // GUI_ALGO_CALIBRATION_INTRINSIC_H
