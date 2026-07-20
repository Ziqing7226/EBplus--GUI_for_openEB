// algo/analytics/event_to_video.h — Event -> grayscale image reconstruction.
//
// Design §4.4.2. Reconstructs grayscale frames from pure event streams via
// three paths: BardowVariational (TV-L1 variational with joint optical-flow
// and intensity estimation via Chambolle-Pock primal-dual solver),
// InteractingMaps (six-map interconnection: I/G/V/F/C/R with rotation
// estimation), and E2VID (DL, default — full pipeline ported from rpg_e2vid
// with ONNX Runtime backend and heuristic fallback). Inspired by the
// referenced papers (Bardow et al. 2016; Cook et al. 2011) and the rpg_e2vid
// project. Header-only.

#ifndef GUI_ALGO_ANALYTICS_EVENT_TO_VIDEO_H
#define GUI_ALGO_ANALYTICS_EVENT_TO_VIDEO_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/analytics/e2vid/e2vid_inference.h"
#include "algo/analytics/e2vid/intensity_rescaler.h"
#include "algo/analytics/e2vid/unsharp_mask.h"

namespace gui_algo {

/// @brief Event-based grayscale image reconstruction (3 modes).
class EventToVideo {
public:
    enum class Mode {
        BardowVariational,  ///< TV-L1 variational (Chambolle-Pock), default.
        InteractingMaps,    ///< Six-map interconnection (Cook 2011).
        E2VID,              ///< Neural-network inference (ONNX Runtime or fallback).
    };

    /// @brief Constructs the reconstructor.
    /// @param width,height Sensor dimensions (or ROI dimensions).
    /// @param mode Reconstruction mode.
    /// @param output_fps Output frame rate in Hz, [1, 120].
    EventToVideo(int width, int height,
                 Mode mode = Mode::BardowVariational,
                 int output_fps = 30)
        : width_(width), height_(height), mode_(mode),
          output_fps_(clamp_fps(output_fps)),
          log_intensity_(static_cast<std::size_t>(width) * height, 0.0),
          e2vid_(width, height) {}

    /// @brief Accumulates events.
    /// For BardowVariational / InteractingMaps: buffers timestamped events
    /// into a ring buffer. At each get_frame() call, events within the
    /// sliding time window [t - window_ms, t] are summed (each contributing
    /// +/- theta) to form the data term f (Bardow) / V (Cook). Events older
    /// than the window are discarded — no exponential decay, matching the
    /// sliding-window formulation in both reference papers.
    /// For E2VID: buffers events for the next voxel grid + inference call.
    void process(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0) return;
        if (mode_ == Mode::E2VID) {
            // Buffer events for E2VID voxel grid inference (E2VID handles
            // its own downsample internally).
            e2vid_event_buffer_.insert(e2vid_event_buffer_.end(),
                                       events, events + n);
            if (events[n - 1].t > current_t_) current_t_ = events[n - 1].t;
        } else {
            // Non-E2VID: buffer events at effective resolution for the
            // sliding-window data-term rebuild in get_frame().
            const int W = eff_w(), H = eff_h();
            event_window_.reserve(event_window_.size() + n);
            for (std::size_t i = 0; i < n; ++i) {
                const Event& e = events[i];
                if (e.t > current_t_) current_t_ = e.t;
                int ex = e.x, ey = e.y;
                if (downsample_) {
                    // 1/4 downsample: keep only even-coordinate events,
                    // halve coordinates (128x128 ROI -> 64x64).
                    if ((e.x & 1u) != 0 || (e.y & 1u) != 0) continue;
                    ex = e.x >> 1;
                    ey = e.y >> 1;
                }
                if (ex < 0 || ex >= W || ey < 0 || ey >= H) continue;
                event_window_.push_back(WindowedEvent{
                    static_cast<std::uint16_t>(ex),
                    static_cast<std::uint16_t>(ey),
                    static_cast<std::int8_t>(e.is_on() ? 1 : -1),
                    e.t});
            }
        }
    }

    /// @brief Reconstructs and returns the current grayscale frame (CV_8UC1).
    cv::Mat get_frame() {
        cv::Mat frame(height_, width_, CV_8UC1, cv::Scalar(0));
        if (width_ <= 0 || height_ <= 0) return frame;
        // Rebuild log_intensity_ from the sliding time window: sum all
        // events within [current_t - window_ms, current_t]. Events older
        // than the window are pruned from the ring buffer. This matches
        // the sliding-window formulation in both Bardow (2016) and Cook
        // (2011) — no exponential decay, old events are fully discarded.
        rebuild_log_intensity();
        // NOTE: the old decay_tau_ms parameter was removed (§四-M7): the
        // decay was applied right after a full window rebuild, so it never
        // accumulated across frames — it only globally dimmed the current
        // frame by the inter-frame interval, a no-op semantically.
        switch (mode_) {
            case Mode::BardowVariational:
                frame = reconstruct_bardow();
                break;
            case Mode::InteractingMaps:
                frame = reconstruct_interacting();
                break;
            case Mode::E2VID:
                frame = reconstruct_e2vid();
                break;
        }
        // Upsample non-E2VID downsampled output back to sensor/ROI size.
        if (mode_ != Mode::E2VID && downsample_ &&
            frame.rows != height_ && !frame.empty()) {
            cv::resize(frame, frame, cv::Size(width_, height_),
                       0, 0, cv::INTER_NEAREST);
        }
        return frame;
    }

    // Mode setters --------------------------------------------------------
    void set_mode(Mode m) {
        if (mode_ != m) {
            mode_ = m;
            // Mode change invalidates state buffers (different code path).
            reset_state();
        }
    }
    Mode mode() const { return mode_; }

    void set_output_fps(int fps) { output_fps_ = clamp_fps(fps); }
    int output_fps() const { return output_fps_; }

    // BardowVariational parameters ---------------------------------------
    void set_window_ms(float ms) { window_ms_ = clamp_window(ms); }
    float window_ms() const { return window_ms_; }

    void set_delta_t_ms(float ms) { delta_t_ms_ = clamp_delta_t(ms); }
    float delta_t_ms() const { return delta_t_ms_; }

    void set_theta(float t) { theta_ = clamp_theta(t); }
    float theta() const { return theta_; }

    void set_lambda1(float v) { lambda1_ = v; }
    void set_lambda2(float v) { lambda2_ = v; }
    void set_lambda3(float v) { lambda3_ = v; }
    void set_lambda4(float v) { lambda4_ = v; }
    void set_lambda5(float v) { lambda5_ = v; }
    void set_lambda6(float v) { lambda6_ = v; }

    void set_num_iterations(int n) { num_iterations_ = clamp_iter(n, 10, 500); }
    int num_iterations() const { return num_iterations_; }

    // InteractingMaps parameters -----------------------------------------
    void set_relaxation_step(float s) {
        relaxation_step_ = (s > 0.0f && s < 1.0f) ? s : 0.1f;
    }
    float relaxation_step() const { return relaxation_step_; }

    void set_im_iterations(int n) { im_iterations_ = clamp_iter(n, 10, 1000); }
    int im_iterations() const { return im_iterations_; }

    /// @brief Sets the camera field-of-view (degrees) used to build the
    /// calibration map C for InteractingMaps. Default 60°.
    void set_fov_deg(float f) {
        fov_deg_ = (f < 10.0f) ? 10.0f : (f > 170.0f ? 170.0f : f);
        im_calib_dirty_ = true;  // force rebuild of C on next reconstruct
    }
    float fov_deg() const { return fov_deg_; }

    // Downsample (non-E2VID modes) ---------------------------------------
    /// @brief Enables/disables 1/4 downsample for Bardow/InteractingMaps.
    /// When on, only even-coordinate events are processed at half resolution,
    /// and the output is upsampled back (INTER_NEAREST) for display.
    void set_downsample(bool v) {
        if (downsample_ != v) {
            downsample_ = v;
            reset_state();  // buffer dimensions changed
        }
    }
    bool downsample() const { return downsample_; }

    // E2VID parameters ----------------------------------------------------
    void set_model_path(const std::string& path) {
        model_path_ = path;
        e2vid_.load_model(path);
    }
    const std::string& model_path() const { return model_path_; }
    bool e2vid_model_loaded() const { return e2vid_.is_model_loaded(); }

    void set_e2vid_num_bins(int b) { e2vid_.set_num_bins(b); }
    int e2vid_num_bins() const { return e2vid_.num_bins(); }

    void set_e2vid_auto_hdr(bool v) { intensity_rescaler_.set_auto_hdr(v); }
    bool e2vid_auto_hdr() const { return intensity_rescaler_.auto_hdr(); }

    void set_e2vid_downsample(bool v) { e2vid_.set_downsample(v); }
    bool e2vid_downsample() const { return e2vid_.downsample(); }

    void set_unsharp_amount(float v) { unsharp_mask_.set_amount(v); }
    float unsharp_amount() const { return unsharp_mask_.amount(); }

    void set_unsharp_sigma(float v) { unsharp_mask_.set_sigma(v); }
    float unsharp_sigma() const { return unsharp_mask_.sigma(); }

    void set_bilateral_sigma(float v) { bilateral_filter_.set_sigma(v); }
    float bilateral_sigma() const { return bilateral_filter_.sigma(); }

    void set_e2vid_hot_pixel_mask(const std::vector<std::uint8_t>& mask) {
        e2vid_.set_hot_pixel_mask(mask);
    }
    void clear_e2vid_hot_pixel_mask() { e2vid_.clear_hot_pixel_mask(); }

    /// @brief Returns the minimum interval between output frames in us.
    Metavision::timestamp frame_interval_us() const {
        return static_cast<Metavision::timestamp>(1.0e6 / output_fps_);
    }

    /// @brief Resets the reconstruction state (log-intensity + E2VID pipeline).
    void reset() {
        reset_state();
        e2vid_.reset();
        e2vid_event_buffer_.clear();
        intensity_rescaler_.reset();
        current_t_ = 0;
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static int clamp_fps(int fps) {
        if (fps < 1) return 1;
        if (fps > 120) return 120;
        return fps;
    }
    static float clamp_window(float ms) {
        if (ms < 10.0f) return 10.0f;
        if (ms > 500.0f) return 500.0f;
        return ms;
    }
    static float clamp_delta_t(float ms) {
        if (ms < 1.0f) return 1.0f;
        if (ms > 50.0f) return 50.0f;
        return ms;
    }
    static float clamp_theta(float t) {
        if (t < 0.05f) return 0.05f;
        if (t > 0.5f) return 0.5f;
        return t;
    }
    static int clamp_iter(int n, int lo, int hi) {
        if (n < lo) return lo;
        if (n > hi) return hi;
        return n;
    }

    /// @brief Effective reconstruction width (halved when downsample is on).
    int eff_w() const { return downsample_ ? (width_ > 0 ? (width_ + 1) / 2 : 0) : width_; }
    /// @brief Effective reconstruction height (halved when downsample is on).
    int eff_h() const { return downsample_ ? (height_ > 0 ? (height_ + 1) / 2 : 0) : height_; }

    /// @brief Rebuilds log_intensity_ from events in the sliding window
    /// [current_t - window_ms, current_t]. Events older than the window
    /// are pruned from event_window_. Each event contributes +/- theta.
    void rebuild_log_intensity() {
        const int W = eff_w(), H = eff_h();
        const std::size_t N = static_cast<std::size_t>(W) * H;
        if (log_intensity_.size() != N) {
            log_intensity_.assign(N, 0.0);
        } else {
            std::fill(log_intensity_.begin(), log_intensity_.end(), 0.0);
        }
        if (event_window_.empty()) return;
        const Metavision::timestamp window_us =
            static_cast<Metavision::timestamp>(window_ms_ * 1000.0f);
        const Metavision::timestamp t_min = current_t_ - window_us;
        // Find first event within the window (events are time-ordered).
        std::size_t first_valid = event_window_.size();
        for (std::size_t i = 0; i < event_window_.size(); ++i) {
            if (event_window_[i].t >= t_min) { first_valid = i; break; }
        }
        // Sum events within the window into log_intensity_.
        for (std::size_t i = first_valid; i < event_window_.size(); ++i) {
            const WindowedEvent& e = event_window_[i];
            log_intensity_[static_cast<std::size_t>(e.y) * W + e.x]
                += (e.p > 0 ? theta_ : -theta_);
        }
        // Prune old events to bound memory.
        if (first_valid > 0) {
            event_window_.erase(event_window_.begin(),
                                event_window_.begin() + first_valid);
        }
    }

    /// @brief Resets non-E2VID reconstruction state.
    void reset_state() {
        std::fill(log_intensity_.begin(), log_intensity_.end(), 0.0);
        event_window_.clear();
        L_.clear(); L_prev_.clear(); L_prior_.clear(); L_tp_.clear();
        ux_.clear(); uy_.clear(); ux_prev_.clear(); uy_prev_.clear();
        px_L_.clear(); py_L_.clear();
        px_ux_.clear(); py_ux_.clear();
        px_uy_.clear(); py_uy_.clear();
        I_map_.clear(); Gx_.clear(); Gy_.clear();
        Fx_.clear(); Fy_.clear();
        Cx_.clear(); Cy_.clear(); Cz_.clear();
        R_[0] = R_[1] = R_[2] = 0.0;
        im_first_frame_ = true;
        im_calib_dirty_ = true;
    }

    /// @brief Converts a log-intensity buffer to a normalized CV_8UC1 frame.
    cv::Mat to_gray(const std::vector<double>& u, int w, int h) const {
        cv::Mat frame(h, w, CV_8UC1, cv::Scalar(0));
        double lo = 0.0, hi = 0.0;
        bool first = true;
        for (const double v : u) {
            if (first) { lo = v; hi = v; first = false; }
            else { if (v < lo) lo = v; if (v > hi) hi = v; }
        }
        const double range = hi - lo;
        for (int y = 0; y < h; ++y) {
            std::uint8_t* row = frame.ptr<std::uint8_t>(y);
            for (int x = 0; x < w; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * w + x;
                double g = range > 1e-9
                               ? (u[idx] - lo) / range
                               : 0.5;
                if (g < 0.0) g = 0.0;
                if (g > 1.0) g = 1.0;
                row[x] = static_cast<std::uint8_t>(g * 255.0 + 0.5);
            }
        }
        return frame;
    }

    /// @brief Chambolle projection TV denoising (scalar field).
    /// Solves: min_u ||u - g||^2/2 + lambda * TV(u)
    /// via Chambolle's semi-implicit projection algorithm.
    /// Dual variables px,py are maintained across calls for warm-start.
    void chambolle_tv(const std::vector<double>& g, double lambda,
                      int iters, int w, int h,
                      std::vector<double>& u,
                      std::vector<double>& px,
                      std::vector<double>& py) const {
        const std::size_t N = static_cast<std::size_t>(w) * h;
        if (u.size() != N) u.assign(N, 0.0);
        if (px.size() != N) px.assign(N, 0.0);
        if (py.size() != N) py.assign(N, 0.0);
        if (lambda <= 1e-9) { u = g; return; }
        const double tau = 1.0 / 16.0;  // <= 1/8 (Lipschitz constant of grad)
        const double inv_lambda = 1.0 / lambda;
        if (chambolle_phi_.size() != N) chambolle_phi_.resize(N);
        auto& phi = chambolle_phi_;
        for (int iter = 0; iter < iters; ++iter) {
            // phi = div(p) - g/lambda.
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * w + x;
                    const double div_p =
                        px[idx] - (x > 0 ? px[idx - 1] : 0.0) +
                        py[idx] - (y > 0 ? py[idx - w] : 0.0);
                    phi[idx] = div_p - g[idx] * inv_lambda;
                }
            }
            // p = (p + tau * grad(phi)) / (1 + tau * |grad(phi)|).
            for (int y = 0; y < h; ++y) {
                for (int x = 0; x < w; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * w + x;
                    const double gx = (x + 1 < w) ? phi[idx + 1] - phi[idx] : 0.0;
                    const double gy = (y + 1 < h) ? phi[idx + w] - phi[idx] : 0.0;
                    const double denom = 1.0 + tau * std::sqrt(gx * gx + gy * gy);
                    px[idx] = (px[idx] + tau * gx) / denom;
                    py[idx] = (py[idx] + tau * gy) / denom;
                }
            }
        }
        // u = g - lambda * div(p).
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * w + x;
                const double div_p =
                    px[idx] - (x > 0 ? px[idx - 1] : 0.0) +
                    py[idx] - (y > 0 ? py[idx - w] : 0.0);
                u[idx] = g[idx] - lambda * div_p;
            }
        }
    }

    /// @brief Solves a 3x3 linear system M·x = b via Cramer's rule.
    static void solve_3x3(const double M[9], const double b[3], double x[3]) {
        const double det = M[0]*(M[4]*M[8]-M[5]*M[7])
                         - M[1]*(M[3]*M[8]-M[5]*M[6])
                         + M[2]*(M[3]*M[7]-M[4]*M[6]);
        if (std::abs(det) < 1e-12) { x[0]=x[1]=x[2]=0.0; return; }
        const double inv = 1.0 / det;
        x[0] = (b[0]*(M[4]*M[8]-M[5]*M[7]) - M[1]*(b[1]*M[8]-M[5]*b[2])
               + M[2]*(b[1]*M[7]-M[4]*b[2])) * inv;
        x[1] = (M[0]*(b[1]*M[8]-M[5]*b[2]) - b[0]*(M[3]*M[8]-M[5]*M[6])
               + M[2]*(M[3]*b[2]-b[1]*M[6])) * inv;
        x[2] = (M[0]*(M[4]*b[2]-b[1]*M[7]) - M[1]*(M[3]*b[2]-b[1]*M[6])
               + b[0]*(M[3]*M[7]-M[4]*M[6])) * inv;
    }

    // =====================================================================
    // BardowVariational: joint optical-flow and intensity estimation.
    //
    // Full reproduction of Bardow et al. 2016 CVPR (Eq. 3), adapted to the
    // real-time 2D framework by approximating the spatio-temporal volume
    // (M x N x K) with a single-time-step sliding window (current frame vs
    // previous frame). All six regularization terms (lambda1..6) and the
    // joint estimation of velocity field u and log-intensity L are preserved:
    //   lambda1: TV(u)           — spatial smoothness of optical flow.
    //   lambda2: ||u - u_prev||  — temporal smoothness of flow.
    //   lambda3: TV(L)           — spatial smoothness of intensity.
    //   lambda4: |<grad L, dt*u> + (L - L_prev)| — optical-flow constraint
    //           (brightness constancy, first-order Taylor approximation).
    //   lambda5: h_theta(L - L_tp) — no-event dead-zone constraint.
    //   lambda6: ||L - L_prior||^2 — prior image retention.
    //
    // Optimization uses sequential proximal steps (Gauss-Seidel style):
    //   1. TV-denoise event data f with lambda3 (data-driven, produces
    //      visible structure — this is the core that prevents gray output).
    //   2. Estimate optical flow u from L (lambda1, lambda2, lambda4) with
    //      robustness: gradient-magnitude thresholding prevents division-
    //      by-near-zero blow-up in flat regions; flow magnitude is clamped.
    //   3. Apply flow-based temporal prediction (lambda4) — single blend.
    //   4. Apply no-event dead-zone soft threshold (lambda5).
    //   5. Apply prior image retention (lambda6) — single gentle blend.
    // Each correction is applied exactly once (not iterated) to prevent the
    // prior term from collapsing the estimate to a flat field.
    // =====================================================================
    cv::Mat reconstruct_bardow() {
        const int W = eff_w(), H = eff_h();
        const std::size_t N = static_cast<std::size_t>(W) * H;
        const std::vector<double>& f = log_intensity_;  // event data term
        // Lazy initialization of state buffers.
        if (L_.size() != N) {
            L_.assign(N, 0.0); L_prev_.assign(N, 0.0);
            L_prior_.assign(N, 0.0); L_tp_.assign(N, 0.0);
            ux_.assign(N, 0.0); uy_.assign(N, 0.0);
            ux_prev_.assign(N, 0.0); uy_prev_.assign(N, 0.0);
            px_L_.assign(N, 0.0); py_L_.assign(N, 0.0);
            px_ux_.assign(N, 0.0); py_ux_.assign(N, 0.0);
            px_uy_.assign(N, 0.0); py_uy_.assign(N, 0.0);
            im_first_frame_ = true;
        }
        const double dt = static_cast<double>(delta_t_ms_) * 1e-3;  // seconds
        // Warm-start L from event data on first frame.
        if (im_first_frame_) {
            L_ = f; L_prev_ = f; L_prior_ = f; L_tp_ = f;
            im_first_frame_ = false;
        }
        // --- Step 1: TV-denoise event data (lambda3) ---
        // L = argmin ||L - f||^2/2 + lambda3 * TV(L). Data-driven: produces
        // visible structure from the event-accumulated log-intensity.
        chambolle_tv(f, lambda3_, num_iterations_, W, H, L_, px_L_, py_L_);
        // --- Step 2: Joint optical flow estimation (lambda1, lambda2, lambda4) ---
        // Optical-flow constraint (Eq. 2): <grad L, u> = -L_t.
        // L_t = (L - L_prev) / dt. Minimum-norm solution (Horn-Schunck style):
        // u_target = -L_t / |grad L|^2 * grad L.
        {
            if (utx_.size() != N) { utx_.assign(N, 0.0); uty_.assign(N, 0.0); }
            auto& utx = utx_;
            auto& uty = uty_;
            const double inv_dt = 1.0 / std::max(dt, 1e-9);
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * W + x;
                    const double gx = (x + 1 < W) ? L_[idx + 1] - L_[idx] : 0.0;
                    const double gy = (y + 1 < H) ? L_[idx + W] - L_[idx] : 0.0;
                    const double gmag2 = gx * gx + gy * gy;
                    // Only estimate flow where gradient is significant —
                    // prevents division-by-near-zero blow-up in flat regions.
                    if (gmag2 > 1e-4) {
                        const double Lt = (L_[idx] - L_prev_[idx]) * inv_dt;
                        utx[idx] = -Lt * gx / gmag2;
                        uty[idx] = -Lt * gy / gmag2;
                        // Clamp flow magnitude to prevent divergence.
                        const double umag = std::sqrt(utx[idx] * utx[idx] +
                                                      uty[idx] * uty[idx]);
                        const double max_u = 5.0;
                        if (umag > max_u) {
                            const double s = max_u / umag;
                            utx[idx] *= s; uty[idx] *= s;
                        }
                    } else {
                        utx[idx] = 0.0; uty[idx] = 0.0;
                    }
                }
            }
            // Temporal smoothness (lambda2): blend with previous flow.
            if (lambda2_ > 1e-9) {
                const double w = lambda2_;
                for (std::size_t i = 0; i < N; ++i) {
                    utx[i] = (utx[i] + w * ux_prev_[i]) / (1.0 + w);
                    uty[i] = (uty[i] + w * uy_prev_[i]) / (1.0 + w);
                }
            }
            // Spatial TV smoothness (lambda1).
            chambolle_tv(utx, lambda1_, num_iterations_, W, H,
                         ux_, px_ux_, py_ux_);
            chambolle_tv(uty, lambda1_, num_iterations_, W, H,
                         uy_, px_uy_, py_uy_);
        }
        // --- Step 3: Flow-based temporal prediction (lambda4) ---
        // Optical-flow consistency: L should satisfy
        // L_t = -<grad L, u> * dt, i.e. L_flow = L_prev + dt * <grad L, u>.
        // Blend L toward L_flow (single application, not iterated).
        if (lambda4_ > 1e-9) {
            const double w = lambda4_;
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * W + x;
                    const double gx = (x + 1 < W) ? L_[idx + 1] - L_[idx] : 0.0;
                    const double gy = (y + 1 < H) ? L_[idx + W] - L_[idx] : 0.0;
                    const double L_flow = L_prev_[idx]
                        + dt * (ux_[idx] * gx + uy_[idx] * gy);
                    L_[idx] = (L_[idx] + w * L_flow) / (1.0 + w);
                }
            }
        }
        // --- Step 4: No-event dead-zone (lambda5) ---
        // Soft-threshold |L - L_tp| around theta (Eq. 4, 6). Within
        // [-theta, theta] no cost; beyond, L1 penalty.
        if (lambda5_ > 1e-9) {
            const double shift = static_cast<double>(lambda5_);
            for (std::size_t i = 0; i < N; ++i) {
                const double diff = L_[i] - L_tp_[i];
                const double thr = theta_ + shift;
                if (diff > thr)       L_[i] -= shift;
                else if (diff < -thr) L_[i] += shift;
            }
        }
        // --- Step 5: Prior image retention (lambda6) ---
        // Paper: λ6·||L(x,t1) − L̂(x)||² constrains L at the oldest
        // time-step in the spatio-temporal volume. In our single-step
        // framework, we apply the prior only at pixels WITHOUT new events
        // (|f|≈0), where the data term provides no fresh information.
        // At pixels with new events, the data term dominates. This matches
        // the paper's intent: the prior compensates information loss from
        // window sliding, which only affects pixels that lost their events.
        if (lambda6_ > 1e-9) {
            const double w = lambda6_;
            for (std::size_t i = 0; i < N; ++i) {
                if (std::abs(f[i]) < 1e-9) {
                    L_[i] = (L_[i] + w * L_prior_[i]) / (1.0 + w);
                }
            }
        }
        // Persist state for next frame's sliding window.
        L_prior_ = L_;   // prior image L-hat
        L_prev_ = L_;    // previous-frame L
        ux_prev_ = ux_;  // previous-frame flow
        uy_prev_ = uy_;
        L_tp_ = L_;      // intensity at last event (approx: current frame)
        return to_gray(L_, W, H);
    }

    // =====================================================================
    // InteractingMaps: six-map interconnection (Cook et al. 2011 IJCNN).
    //
    // Full reproduction of the interacting-map network with six maps:
    //   I: light intensity          ((W+1) x (H+1), scalar)
    //   G: spatial gradient         (W x H, 2D vector)
    //   V: temporal derivative      (W x H, scalar) — input from events
    //   F: optical flow             (W x H, 2D vector)
    //   C: camera calibration       (W x H, 3D vector) — precomputed constant
    //   R: camera rotation          (single 3D vector)
    //
    // Three relations drive relaxation updates:
    //   (i)   G = grad(I)              — gradient definition (Eq. 2, 6, 7-9)
    //   (ii)  -V = F . G               — optical-flow constraint (Eq. 1, 5)
    //   (iii) F = m32(R x C)           — 3D rotation geometry (Eq. 3, 11-13)
    //
    // Each relation pulls a map toward the candidate satisfying it, using a
    // relaxation step δ. R is updated via linear least squares (Eq. 10, 13).
    //
    // Stability fixes (minimal, standard practice — do not alter algorithm
    // behavior under normal conditions):
    //   - V clamped to [-1, 1] (saturate after ~4-5 events per pixel)
    //   - F estimation skipped where |G|² < 1e-6 (prevent div-by-zero)
    //   - F magnitude clamped to 5.0 (prevent blow-up)
    //   - R magnitude clamped to 0.5 rad (prevent rotation blow-up)
    //   - I clamped to [-1, 1] (prevent divergence)
    //   - Warm-start I from V on first frame (prevent all-zero deadlock)
    // =====================================================================
    cv::Mat reconstruct_interacting() {
        const int W = eff_w(), H = eff_h();
        const std::size_t N = static_cast<std::size_t>(W) * H;
        const std::size_t NI = static_cast<std::size_t>(W + 1) * (H + 1);
        const std::vector<double>& V = log_intensity_;  // input map V
        // Clamp V to [-1, 1] — events represent ±theta=0.22 each; clamping
        // at ±1.0 allows ~4-5 events per pixel before saturation.
        if (Vc_.size() != N) Vc_.resize(N);
        auto& Vc = Vc_;
        for (std::size_t i = 0; i < N; ++i) {
            double v = V[i];
            Vc[i] = (v > 1.0) ? 1.0 : (v < -1.0 ? -1.0 : v);
        }
        // Lazy initialization.
        if (I_map_.size() != NI) {
            I_map_.assign(NI, 0.0);
            Gx_.assign(N, 0.0); Gy_.assign(N, 0.0);
            Fx_.assign(N, 0.0); Fy_.assign(N, 0.0);
            R_[0] = R_[1] = R_[2] = 0.0;
            im_calib_dirty_ = true;
            im_first_frame_ = true;
        }
        if (im_calib_dirty_ || Cx_.size() != N) {
            build_calibration_map();
            im_calib_dirty_ = false;
        }
        // Warm-start: initialize I from V on first frame. Without this,
        // I=G=F=0 and V cannot influence the system (F·G deadlock).
        if (im_first_frame_) {
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    I_map_[static_cast<std::size_t>(y) * (W + 1) + x] =
                        Vc[static_cast<std::size_t>(y) * W + x];
                }
            }
            im_first_frame_ = false;
        }
        const double delta = static_cast<double>(relaxation_step_);
        // Relaxation iterations (Cook et al. 2011, Section II).
        for (int iter = 0; iter < im_iterations_; ++iter) {
            // (i) G = grad(I) — update gradient toward I's gradient.
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * W + x;
                    const std::size_t iidx =
                        static_cast<std::size_t>(y) * (W + 1) + x;
                    const double gx = I_map_[iidx + 1] - I_map_[iidx];
                    const double gy = I_map_[iidx + W + 1] - I_map_[iidx];
                    Gx_[idx] = (1.0 - delta) * Gx_[idx] + delta * gx;
                    Gy_[idx] = (1.0 - delta) * Gy_[idx] + delta * gy;
                }
            }
            // (ii) -V = F·G → F = -V * G / |G|² — optical flow constraint.
            // Skip where |G|² is negligible (flat regions). Clamp |F|.
            for (std::size_t i = 0; i < N; ++i) {
                const double gmag2 = Gx_[i] * Gx_[i] + Gy_[i] * Gy_[i];
                if (gmag2 > 1e-6) {
                    double fx_c = -Vc[i] * Gx_[i] / gmag2;
                    double fy_c = -Vc[i] * Gy_[i] / gmag2;
                    const double fmag = std::sqrt(fx_c * fx_c + fy_c * fy_c);
                    constexpr double fmax = 5.0;
                    if (fmag > fmax) {
                        const double s = fmax / fmag;
                        fx_c *= s; fy_c *= s;
                    }
                    Fx_[i] = (1.0 - delta) * Fx_[i] + delta * fx_c;
                    Fy_[i] = (1.0 - delta) * Fy_[i] + delta * fy_c;
                }
            }
            // (iii) F = m32(R × C) — rotation geometry.
            // Step a: update R via linear least squares from F and C.
            //   F_x =  Cz*R1 - Cy*R2
            //   F_y = -Cz*R0 + Cx*R2
            // Jacobian A_i = [[0, Cz, -Cy], [-Cz, 0, Cx]].
            // Normal equations: M·R = b, M = Σ A_i^T A_i, b = Σ A_i^T F_i.
            {
                double M[9] = {0}, b[3] = {0};
                for (std::size_t i = 0; i < N; ++i) {
                    const double cx = Cx_[i], cy = Cy_[i], cz = Cz_[i];
                    const double fx = Fx_[i], fy = Fy_[i];
                    // A_i^T A_i (symmetric, upper triangle):
                    M[0] += cz * cz;                 // (0,0)
                    M[1] += 0.0;                     // (0,1)
                    M[2] += -cz * cx;                // (0,2)
                    M[4] += cz * cz;                 // (1,1)
                    M[5] += -cz * cy;                // (1,2)
                    M[8] += cy * cy + cx * cx;       // (2,2)
                    // A_i^T F_i:
                    b[0] += -cz * fy;
                    b[1] += cz * fx;
                    b[2] += -cy * fx + cx * fy;
                }
                M[3] = M[1]; M[6] = M[2]; M[7] = M[5];  // symmetric
                double R_new[3];
                solve_3x3(M, b, R_new);
                // Clamp R magnitude (prevents rotation blow-up).
                const double rmag = std::sqrt(R_new[0] * R_new[0] +
                                              R_new[1] * R_new[1] +
                                              R_new[2] * R_new[2]);
                constexpr double rmax = 0.5;  // radians
                if (rmag > rmax) {
                    const double s = rmax / rmag;
                    R_new[0] *= s; R_new[1] *= s; R_new[2] *= s;
                }
                R_[0] = (1.0 - delta) * R_[0] + delta * R_new[0];
                R_[1] = (1.0 - delta) * R_[1] + delta * R_new[1];
                R_[2] = (1.0 - delta) * R_[2] + delta * R_new[2];
            }
            // Step b: update F from R and C (relation iii candidate).
            for (std::size_t i = 0; i < N; ++i) {
                const double cx = Cx_[i], cy = Cy_[i], cz = Cz_[i];
                // m32(R × C) = [R1*Cz - R2*Cy, R2*Cx - R0*Cz]
                const double fx_c = R_[1] * cz - R_[2] * cy;
                const double fy_c = R_[2] * cx - R_[0] * cz;
                Fx_[i] = (1.0 - delta) * Fx_[i] + delta * fx_c;
                Fy_[i] = (1.0 - delta) * Fy_[i] + delta * fy_c;
            }
            // Update I: data fidelity (I → V) + gradient integration (Poisson).
            // The Poisson step makes ∇I consistent with G (relation i),
            // providing spatial smoothing. Data fidelity keeps I grounded
            // to the event-derived log-intensity V.
            for (int y = 0; y < H; ++y) {
                for (int x = 0; x < W; ++x) {
                    const std::size_t idx =
                        static_cast<std::size_t>(y) * W + x;
                    const std::size_t iidx =
                        static_cast<std::size_t>(y) * (W + 1) + x;
                    // Poisson target: I should have gradient G.
                    // ∇²I = ∇·G → I_target = avg(neighbors) - div(G)/4.
                    const double I_l = I_map_[iidx - (x > 0 ? 1 : 0)];
                    const double I_r = I_map_[iidx + (x + 1 < W ? 1 : 0)];
                    const double I_u = I_map_[iidx - (y > 0 ? W + 1 : 0)];
                    const double I_d = I_map_[iidx + (y + 1 < H ? W + 1 : 0)];
                    const double div_G =
                        (Gx_[idx] - (x > 0 ? Gx_[idx - 1] : 0.0)) +
                        (Gy_[idx] - (y > 0 ? Gy_[idx - static_cast<std::size_t>(W)] : 0.0));
                    const double poisson =
                        0.25 * (I_l + I_r + I_u + I_d) - 0.25 * div_G;
                    // Data fidelity: blend with V where events exist.
                    double target;
                    if (std::abs(Vc[idx]) > 1e-9) {
                        target = 0.5 * Vc[idx] + 0.5 * poisson;
                    } else {
                        target = poisson;
                    }
                    double I_new = (1.0 - delta) * I_map_[iidx] + delta * target;
                    // Clamp I to prevent divergence.
                    if (I_new > 1.0) I_new = 1.0;
                    if (I_new < -1.0) I_new = -1.0;
                    I_map_[iidx] = I_new;
                }
            }
        }
        // Extract I (W×H subgrid) to gray frame.
        if (I_out_.size() != N) I_out_.resize(N);
        auto& I_out = I_out_;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                I_out[static_cast<std::size_t>(y) * W + x] =
                    I_map_[static_cast<std::size_t>(y) * (W + 1) + x];
            }
        }
        return to_gray(I_out, W, H);
    }

    /// @brief Builds the camera calibration map C (3D unit direction per
    /// pixel) from the field-of-view, and precomputes the least-squares
    /// projection matrix M = sum(I - C_i C_i^T) used for rotation estimation.
    void build_calibration_map() {
        const int W = eff_w(), H = eff_h();
        const std::size_t N = static_cast<std::size_t>(W) * H;
        Cx_.assign(N, 0.0); Cy_.assign(N, 0.0); Cz_.assign(N, 0.0);
        const double fov = static_cast<double>(fov_deg_) * M_PI / 180.0;
        const double f = (W > H ? W : H) / 2.0 / std::tan(fov / 2.0);
        const double cx0 = (W - 1) / 2.0;
        const double cy0 = (H - 1) / 2.0;
        // Accumulate M = sum(I - C C^T).
        double M[9] = {0};
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const std::size_t idx =
                    static_cast<std::size_t>(y) * W + x;
                double dx = (x - cx0) / f;
                double dy = (y - cy0) / f;
                double dz = 1.0;
                const double norm = std::sqrt(dx * dx + dy * dy + dz * dz);
                dx /= norm; dy /= norm; dz /= norm;
                Cx_[idx] = dx; Cy_[idx] = dy; Cz_[idx] = dz;
                M[0] += 1.0 - dx * dx;
                M[1] += -dx * dy;
                M[2] += -dx * dz;
                M[3] += -dy * dx;
                M[4] += 1.0 - dy * dy;
                M[5] += -dy * dz;
                M[6] += -dz * dx;
                M[7] += -dz * dy;
                M[8] += 1.0 - dz * dz;
            }
        }
        for (int i = 0; i < 9; ++i) im_Mat_[i] = M[i];
    }

    /// @brief E2VID reconstruction: voxel grid -> neural network inference
    ///        (or heuristic fallback) -> unsharp mask -> intensity rescale ->
    ///        bilateral filter. Ported from rpg_e2vid image_reconstructor.py.
    cv::Mat reconstruct_e2vid() {
        if (e2vid_event_buffer_.empty()) {
            return cv::Mat::zeros(height_, width_, CV_8UC1);
        }
        cv::Mat raw = e2vid_.infer(e2vid_event_buffer_.data(),
                                   e2vid_event_buffer_.size());
        cv::Mat sharpened;
        if (raw.type() == CV_32FC1) {
            sharpened = unsharp_mask_(raw);
        } else {
            sharpened = raw;
        }
        cv::Mat rescaled;
        if (sharpened.type() == CV_32FC1) {
            rescaled = intensity_rescaler_(sharpened);
        } else {
            rescaled = sharpened;
        }
        cv::Mat cropped = e2vid_.crop_to_sensor(rescaled);
        cv::Mat filtered = bilateral_filter_(cropped);
        e2vid_event_buffer_.clear();
        return filtered;
    }

    int width_;
    int height_;
    Mode mode_;
    int output_fps_;

    // BardowVariational parameters.
    float window_ms_{50.0f};       ///< Sliding window duration (ms).
    float delta_t_ms_{15.0f};
    float theta_{0.22f};
    float lambda1_{0.02f};
    float lambda2_{0.05f};
    float lambda3_{0.02f};
    float lambda4_{0.2f};
    float lambda5_{0.1f};
    float lambda6_{1.0f};          ///< Prior retention (paper value). Applied
                                   ///< only at pixels without new events to
                                   ///< match the paper's intent of compensating
                                   ///< window-sliding information loss.
    int num_iterations_{100};

    // InteractingMaps parameters.
    float relaxation_step_{0.1f};
    int im_iterations_{50};
    float fov_deg_{60.0f};

    // Downsample (non-E2VID modes).
    bool downsample_{false};

    // Base reconstruction state (used by non-E2VID modes).
    /// Compact event record for the sliding-window ring buffer (effective
    /// resolution coordinates, after downsampling).
    struct WindowedEvent {
        std::uint16_t x, y;
        std::int8_t p;            ///< +1 (on) or -1 (off)
        Metavision::timestamp t;  ///< Timestamp in us
    };
    std::vector<WindowedEvent> event_window_;  ///< Time-ordered event buffer.
    std::vector<double> log_intensity_;
    Metavision::timestamp current_t_{0};

    // --- BardowVariational optimization state ---
    std::vector<double> L_;         ///< Current log-intensity estimate.
    std::vector<double> L_prev_;    ///< Previous-frame L (temporal term).
    std::vector<double> L_prior_;   ///< Prior image L-hat (lambda6).
    std::vector<double> L_tp_;      ///< Intensity at last event (dead-zone).
    std::vector<double> ux_, uy_;   ///< Current optical-flow field.
    std::vector<double> ux_prev_, uy_prev_;  ///< Previous-frame flow.
    // Chambolle TV dual variables (warm-started across iterations).
    std::vector<double> px_L_, py_L_;    ///< Duals for L TV (lambda3).
    std::vector<double> px_ux_, py_ux_;  ///< Duals for ux TV (lambda1).
    std::vector<double> px_uy_, py_uy_;  ///< Duals for uy TV (lambda1).
    bool im_first_frame_{true};

    // --- InteractingMaps state ---
    std::vector<double> I_map_;  ///< Intensity map ((W+1) x (H+1)).
    std::vector<double> Gx_, Gy_;  ///< Spatial gradient (W x H).
    std::vector<double> Fx_, Fy_;  ///< Optical flow (W x H).
    double R_[3] = {0.0, 0.0, 0.0};  ///< Global rotation vector.
    std::vector<double> Cx_, Cy_, Cz_;  ///< Calibration map C (W x H, 3D).
    double im_Mat_[9] = {0};  ///< Precomputed sum(I - C C^T) for R least-squares.
    bool im_calib_dirty_{true};

    // --- Reusable temporary buffers (avoid per-call allocation) ---
    mutable std::vector<double> chambolle_phi_;  ///< chambolle_tv scratch (const method)
    std::vector<double> utx_, uty_;              ///< reconstruct_bardow flow targets
    std::vector<double> Vc_, I_out_;             ///< reconstruct_interacting scratch

    // E2VID parameters.
    std::string model_path_;

    // E2VID pipeline components (ported from rpg_e2vid).
    E2VIDInference e2vid_;
    IntensityRescaler intensity_rescaler_;
    UnsharpMaskFilter unsharp_mask_;
    BilateralImageFilter bilateral_filter_;
    std::vector<Event> e2vid_event_buffer_;
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_EVENT_TO_VIDEO_H
