// gui/algo_bridge/backends/analytics_backends.cpp — frame producers + analyzers
// (design §3.4). Split from the former algo_backend.cpp monolith.
// Contains: EventToVideo, FlowStatistics, ISIAnalyzer.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algo/analytics/event_to_video.h"
#include "algo/analytics/flow_statistics.h"
#include "algo/analytics/isi_analyzer.h"
#include "algo/analytics/sensor_self_test.h"

using namespace gui::backend_detail;

namespace gui {

class EventToVideoBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    // Current param values (re-applied after ROI rebuild so the fresh
    // EventToVideo instance inherits every setting, including E2VID model
    // path and post-processing params — otherwise ROI changes would silently
    // drop the E2VID configuration and fall back to the heuristic path).
    gui_algo::EventToVideo::Mode mode_{gui_algo::EventToVideo::Mode::E2VID};
    int output_fps_{30};
    // BardowVariational params.
    float window_ms_{50.0F};
    float delta_t_ms_{15.0F};
    float theta_{0.22F};
    int num_iterations_{100};
    float lambda1_{0.02F}, lambda2_{0.05F}, lambda3_{0.02F};
    float lambda4_{0.2F}, lambda5_{0.1F}, lambda6_{1.0F};
    float decay_tau_ms_{0.0F};
    // InteractingMaps params.
    float relaxation_step_{0.1F};
    int im_iterations_{50};
    float fov_deg_{60.0F};
    // E2VID params.
    std::string model_path_;
    int e2vid_num_bins_{5};
    bool e2vid_auto_hdr_{false};
    float unsharp_amount_{0.3F};
    float unsharp_sigma_{1.0F};
    float bilateral_sigma_{0.0F};
    std::unique_ptr<gui_algo::EventToVideo> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    bool filtered_active_{false};  ///< true when ROI/preproc modified events (N4)
    Preprocessor preproc_;
public:
    EventToVideoBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        algo_ = std::make_unique<gui_algo::EventToVideo>(aw / f, ah / f, mode_, output_fps_);
        // BardowVariational
        algo_->set_window_ms(window_ms_);
        algo_->set_delta_t_ms(delta_t_ms_);
        algo_->set_theta(theta_);
        algo_->set_num_iterations(num_iterations_);
        algo_->set_lambda1(lambda1_);
        algo_->set_lambda2(lambda2_);
        algo_->set_lambda3(lambda3_);
        algo_->set_lambda4(lambda4_);
        algo_->set_lambda5(lambda5_);
        algo_->set_lambda6(lambda6_);
        algo_->set_decay_tau_ms(decay_tau_ms_);
        // InteractingMaps
        algo_->set_relaxation_step(relaxation_step_);
        algo_->set_im_iterations(im_iterations_);
        algo_->set_fov_deg(fov_deg_);
        // E2VID: load the ONNX model if a path is set. An empty path keeps
        // the heuristic fallback; a non-empty path triggers load_model which
        // may fail silently and also fall back (BUG-G9: comment corrected —
        // the model IS loaded here in rebuild(), not deferred).
        if (!model_path_.empty()) algo_->set_model_path(model_path_);
        algo_->set_e2vid_num_bins(e2vid_num_bins_);
        // Re-sync from the algo: when a model is loaded, set_num_bins ignores
        // the caller's value and uses model_num_bins_ (e2vid_inference.h).
        // Without this re-sync, e2vid_num_bins_ would stay at the stale
        // default/user value and get_param("num_bins") would return the
        // wrong value (BUG-N11).
        e2vid_num_bins_ = algo_->e2vid_num_bins();
        algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        // 1/4 downsample is handled by the shared Preprocessor (which halves
        // event coordinates before they reach the algo). The algo's internal
        // downsample flags must stay OFF to avoid double-halving.
        algo_->set_e2vid_downsample(false);
        algo_->set_downsample(false);
        algo_->set_unsharp_amount(unsharp_amount_);
        algo_->set_unsharp_sigma(unsharp_sigma_);
        algo_->set_bilateral_sigma(bilateral_sigma_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Snapshot ROI state BEFORE any param mutation so the rebuild-skip
        // logic below compares old vs. new against the true prior state.
        // Without this, roi_enabled toggling would read the NEW roi_.enabled
        // for both old_aw and new_aw, falsely skipping rebuild() and leaving
        // the algorithm with mismatched dimensions vs. incoming events (N2).
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) {
                mode_ = static_cast<gui_algo::EventToVideo::Mode>(m);
                if (algo_) algo_->set_mode(mode_);
            }
        } else if (k == "output_fps") {
            output_fps_ = to_i(v);
            if (algo_) algo_->set_output_fps(output_fps_);
        } else if (k == "window_ms") {
            window_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_window_ms(window_ms_);
        } else if (k == "delta_t_ms") {
            delta_t_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_delta_t_ms(delta_t_ms_);
        } else if (k == "theta") {
            theta_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_theta(theta_);
        } else if (k == "num_iterations") {
            num_iterations_ = to_i(v);
            if (algo_) algo_->set_num_iterations(num_iterations_);
        } else if (k == "lambda1") {
            lambda1_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda1(lambda1_);
        } else if (k == "lambda2") {
            lambda2_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda2(lambda2_);
        } else if (k == "lambda3") {
            lambda3_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda3(lambda3_);
        } else if (k == "lambda4") {
            lambda4_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda4(lambda4_);
        } else if (k == "lambda5") {
            lambda5_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda5(lambda5_);
        } else if (k == "lambda6") {
            lambda6_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_lambda6(lambda6_);
        } else if (k == "decay_tau_ms") {
            decay_tau_ms_ = static_cast<float>(to_d(v)); if (algo_) algo_->set_decay_tau_ms(decay_tau_ms_);
        } else if (k == "relaxation_step") {
            relaxation_step_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_relaxation_step(relaxation_step_);
        } else if (k == "im_iterations") {
            im_iterations_ = to_i(v);
            if (algo_) algo_->set_im_iterations(im_iterations_);
        } else if (k == "fov_deg") {
            fov_deg_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_fov_deg(fov_deg_);
        } else if (k == "model_path") {
            model_path_ = v;
            if (algo_) {
                algo_->set_model_path(model_path_);
                // num_bins is dictated by the loaded model (rpg_e2vid:
                // model.num_bins). Sync the persisted value so subsequent
                // ROI rebuilds keep the model's channel count.
                e2vid_num_bins_ = algo_->e2vid_num_bins();
            }
        } else if (k == "num_bins") {
            e2vid_num_bins_ = to_i(v);
            if (algo_) {
                algo_->set_e2vid_num_bins(e2vid_num_bins_);
                // Re-sync: the algo ignores the caller's value when a model
                // is loaded (model_num_bins_ takes precedence). Without this,
                // e2vid_num_bins_ would retain the stale user/cache value
                // and get_param("num_bins") would return the wrong value
                // (BUG-N11).
                e2vid_num_bins_ = algo_->e2vid_num_bins();
            }
        } else if (k == "auto_hdr") {
            e2vid_auto_hdr_ = to_b(v);
            if (algo_) algo_->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        } else if (k == "unsharp_amount") {
            unsharp_amount_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_amount(unsharp_amount_);
        } else if (k == "unsharp_sigma") {
            unsharp_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_unsharp_sigma(unsharp_sigma_);
        } else if (k == "bilateral_sigma") {
            bilateral_sigma_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_bilateral_sigma(bilateral_sigma_);
        } else if (k == "downsample") {
            // Backward compat: the old per-algo "downsample" key is forwarded
            // to the shared preproc stage (preproc_downsample). The per-algo
            // flag stays OFF to avoid double-halving (BUG-R2). Must rebuild
            // directly (not via need_rebuild + N2 dimension check) because
            // the preproc factor change affects algorithm dimensions (aw/f,
            // ah/f in rebuild()) regardless of ROI dimensions — the N2 check
            // only compares ROI dims and would falsely skip the rebuild.
            // Skip rebuild if the value didn't change (avoids unnecessary
            // ONNX reload on duplicate set_param calls).
            const std::string prev = preproc_.get_param("preproc_downsample");
            preproc_.set_param("preproc_downsample", v);
            if (prev != v) rebuild();
            return;
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); need_rebuild = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); need_rebuild = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); need_rebuild = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); need_rebuild = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); need_rebuild = true; }
        if (need_rebuild) {
            // Only rebuild (which reloads the ONNX model) when the effective
            // dimensions actually change. ROI position changes (x, y) don't
            // affect the algorithm dimensions — they only shift which events
            // are cropped — so skip the expensive rebuild (N2).
            // Use the snapshot taken at entry — roi_.enabled may have been
            // toggled by this very call, so reading it now would compare the
            // NEW state against itself and falsely skip rebuild().
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) {
                rebuild();
            }
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "mode") return from_i(static_cast<int>(mode_));
        if (k == "output_fps") return from_i(output_fps_);
        if (k == "window_ms") return from_d(window_ms_);
        if (k == "delta_t_ms") return from_d(delta_t_ms_);
        if (k == "theta") return from_d(theta_);
        if (k == "num_iterations") return from_i(num_iterations_);
        if (k == "lambda1") return from_d(lambda1_);
        if (k == "lambda2") return from_d(lambda2_);
        if (k == "lambda3") return from_d(lambda3_);
        if (k == "lambda4") return from_d(lambda4_);
        if (k == "lambda5") return from_d(lambda5_);
        if (k == "lambda6") return from_d(lambda6_);
        if (k == "decay_tau_ms") return from_d(decay_tau_ms_);
        if (k == "relaxation_step") return from_d(relaxation_step_);
        if (k == "im_iterations") return from_i(im_iterations_);
        if (k == "fov_deg") return from_d(fov_deg_);
        if (k == "model_path") return model_path_;
        if (k == "num_bins") return from_i(e2vid_num_bins_);
        if (k == "auto_hdr") return from_b(e2vid_auto_hdr_);
        if (k == "unsharp_amount") return from_d(unsharp_amount_);
        if (k == "unsharp_sigma") return from_d(unsharp_sigma_);
        if (k == "bilateral_sigma") return from_d(bilateral_sigma_);
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "roi_x") return from_i(roi_.x);
        if (k == "roi_y") return from_i(roi_.y);
        if (k == "roi_w") return from_i(roi_.w);
        if (k == "roi_h") return from_i(roi_.h);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
            filtered_active_ = true;
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
            filtered_active_ = true;
        } else {
            filtered_active_ = false;
        }
        algo_->process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        // Return the post-ROI/preproc events when filtering was applied;
        // otherwise return the raw passthrough (N4). gui_algo::Event and
        // Metavision::EventCD are layout-compatible (static_assert in
        // as_events), so reinterpret_cast is safe.
        if (filtered_active_) {
            const auto* cd = reinterpret_cast<const Metavision::EventCD*>(roi_events_.data());
            r.filtered_events.assign(cd, cd + roi_events_.size());
        } else {
            r.filtered_events = passthrough_;
        }
        r.has_frame = true;
        cv::Mat frame = algo_->get_frame();
        const int f = preproc_.factor();
        if (f > 1 && !frame.empty()) {
            const int aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int ah = roi_.enabled ? roi_.rh : sensor_h_;
            cv::resize(frame, frame, cv::Size(aw, ah), 0, 0, cv::INTER_NEAREST);
        }
        r.frame = frame.clone();
        r.status = "e2v: " + std::to_string(algo_->width()) + "x" +
                   std::to_string(algo_->height()) +
                   (roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
        filtered_active_ = false;
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w;
        sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_);
        rebuild();
    }
};

/// FlowStatistics backend — requires ground-truth flow samples (not available
/// in real-time). Counts events only (audit §三-9: the FlowStatistics member
/// was never fed events, so it was removed); reports a status; no frame is
/// produced. Supports ROI (design §5.6.6): when enabled, only ROI events are
/// counted.
class FlowStatisticsBackend final : public AlgoBackend {
    std::vector<Metavision::EventCD> passthrough_;
    RoiFilter roi_;
    std::vector<gui_algo::Event> roi_buf_;
    std::size_t total_events_{0};
public:
    FlowStatisticsBackend(int w, int h) {
        roi_.init(w, h);
    }
    void set_param(const std::string& k, const std::string& v) override {
        roi_.set_param(k, v);
    }
    std::string get_param(const std::string& k) const override {
        auto r = roi_.get_param(k); if (!r.empty()) return r;
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        auto [ev, n] = roi_.apply(as_events(passthrough_.data()),
                                   passthrough_.size(), roi_buf_);
        total_events_ += n;
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.status = "flow_stats: " + std::to_string(total_events_) +
                   " events (no GT — Passive mode)" +
                   std::string(roi_.region.enabled ? " (ROI)" : "");
        return r;
    }
    void reset() override { passthrough_.clear(); roi_buf_.clear(); total_events_ = 0; }
    void set_sensor_dimensions(int w, int h) override {
        // Event-count only — no sensor-sized algorithm state; just update
        // the ROI geometry (audit §五-D1).
        roi_.set_sensor_dimensions(w, h);
    }
};

/// ISIAnalyzer backend — renders ISI histogram as frame.
/// Complex algorithm (design §4.4.4): defaults to the center 128×128 ROI.
class ISIAnalyzerBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    ProcessRegion roi_;
    bool per_pixel_{false};
    float max_isi_ms_{100.0F};
    std::unique_ptr<gui_algo::ISIAnalyzer> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    std::vector<gui_algo::Event> roi_events_;
    Preprocessor preproc_;
public:
    ISIAnalyzerBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
    }
    void rebuild() {
        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
        preproc_.init(aw, ah);
        const int f = preproc_.factor();
        algo_ = std::make_unique<gui_algo::ISIAnalyzer>(aw / f, ah / f, 32, max_isi_ms_, per_pixel_);
    }
    void set_param(const std::string& k, const std::string& v) override {
        if (preproc_.set_param(k, v)) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Only rebuild when the effective dimensions actually change
        // (audit §五-D4): apply_global_roi fires 5 set_param calls, and a
        // rebuild discards the accumulated histogram each time.
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        bool roi_changed = false;
        if (k == "per_pixel") {
            per_pixel_ = to_b(v);
            need_rebuild = true;
        } else if (k == "max_isi_ms") {
            max_isi_ms_ = static_cast<float>(to_d(v));
            if (algo_) algo_->set_max_isi_ms(max_isi_ms_);
        } else if (k == "roi_enabled") { roi_.enabled = to_b(v); roi_changed = true; }
        else if (k == "roi_x") { roi_.x = to_i(v); roi_changed = true; }
        else if (k == "roi_y") { roi_.y = to_i(v); roi_changed = true; }
        else if (k == "roi_w") { roi_.w = to_i(v); roi_changed = true; }
        else if (k == "roi_h") { roi_.h = to_i(v); roi_changed = true; }
        if (need_rebuild) { roi_.compute(sensor_w_, sensor_h_); rebuild(); }
        else if (roi_changed) {
            const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
            const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
            roi_.compute(sensor_w_, sensor_h_);
            const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
            if (new_aw != old_aw || new_ah != old_ah) rebuild();
        }
    }
    std::string get_param(const std::string& k) const override {
        auto pp = preproc_.get_param(k); if (!pp.empty()) return pp;
        if (k == "roi_enabled") return from_b(roi_.enabled);
        if (k == "per_pixel") return from_b(per_pixel_);
        if (k == "max_isi_ms") return from_d(max_isi_ms_);
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        const auto* ev = as_events(passthrough_.data());
        std::size_t n = passthrough_.size();
        if (roi_.enabled && roi_.rw > 0 && roi_.rh > 0) {
            crop_to_roi(ev, n, roi_, &preproc_, roi_events_);
            ev = roi_events_.data();
            n = roi_events_.size();
        } else if (preproc_.active() && n > 0) {
            auto [p, m] = preproc_.apply(ev, n);
            roi_events_.assign(p, p + m);
            ev = roi_events_.data();
            n = m;
        }
        algo_->process(ev, n);
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        cv::Mat frame = algo_->render();
        const int f = preproc_.factor();
        if (f > 1 && !frame.empty()) {
            const int aw = roi_.enabled ? roi_.rw : sensor_w_;
            const int ah = roi_.enabled ? roi_.rh : sensor_h_;
            cv::resize(frame, frame, cv::Size(aw, ah), 0, 0, cv::INTER_NEAREST);
        }
        r.frame = frame.clone();
        r.status = "isi: histogram" + std::string(roi_.enabled ? " (ROI)" : " (full)");
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        roi_events_.clear();
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w; sensor_h_ = h;
        roi_.compute(sensor_w_, sensor_h_); rebuild();
    }
};

/// SensorSelfTest backend — per-pixel refractory-period heatmap + bad-pixel
/// detection (design §4.4.8). Processes the FULL sensor (no ROI, no
/// preprocessing) because the self-test must cover every pixel. Produces a
/// CV_8UC3 heatmap frame: red = never triggered (suspected bad pixel),
/// grayscale = exponential mapping of min inter-event interval (brighter =
/// shorter refractory period).
class SensorSelfTestBackend final : public AlgoBackend {
    int sensor_w_{0}, sensor_h_{0};
    std::unique_ptr<gui_algo::SensorSelfTest> algo_;
    std::vector<Metavision::EventCD> passthrough_;
    // The full report (with bad-pixel coordinates) is expensive to compute
    // (O(N log N) for the median/p90 sort), so it is cached and recomputed
    // every ~30 frames (~0.5s at 60fps). The per-frame AlgoWindow status
    // label uses a concise one-line summary (via count_pixels(), O(N) with
    // a trivial constant) instead of the full multi-line report.
    int frame_counter_{0};
    std::string cached_report_;
    // Flag set by the close handler via set_param("__final_report", "1")
    // so the next pull_result() returns the full report in r.status for
    // the close dialog. Cleared after one use.
    bool final_report_requested_{false};
public:
    explicit SensorSelfTestBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        rebuild();
    }
    void rebuild() {
        algo_ = std::make_unique<gui_algo::SensorSelfTest>(sensor_w_, sensor_h_);
        cached_report_.clear();
        frame_counter_ = 0;
        final_report_requested_ = false;
    }
    void set_param(const std::string& k, const std::string& v) override {
        // No user-tunable parameters — the self-test is fully automatic.
        // The close handler uses "__final_report" to request the full report.
        if (k == "__final_report" && v == "1") {
            final_report_requested_ = true;
        }
    }
    std::string get_param(const std::string& /*k*/) const override {
        return {};
    }
    void push_events(const Metavision::EventCD* b, const Metavision::EventCD* e) override {
        passthrough_.assign(b, e);
        algo_->process(as_events(passthrough_.data()), passthrough_.size());
    }
    AlgoResult pull_result() override {
        AlgoResult r;
        r.filtered_events = passthrough_;
        r.has_frame = true;
        r.frame = algo_->render().clone();
        // Close handler requested the full report — compute it fresh and
        // return it in r.status for the dialog. This is a one-shot flag.
        if (final_report_requested_) {
            final_report_requested_ = false;
            cached_report_ = algo_->report();
            r.status = cached_report_;
            return r;
        }
        // Recompute the cached report periodically for potential future use.
        if (cached_report_.empty() || frame_counter_ % 30 == 0) {
            cached_report_ = algo_->report();
        }
        ++frame_counter_;
        // Per-frame status: concise one-line summary (not the full report,
        // which would flood the AlgoWindow status label every frame).
        const auto c = algo_->count_pixels();
        r.status = "Triggered: " + std::to_string(c.triggered) + "/" +
                   std::to_string(c.total) + " | Measured: " +
                   std::to_string(c.measured) + " | Bad: " +
                   std::to_string(c.bad);
        return r;
    }
    void reset() override {
        if (algo_) algo_->reset();
        passthrough_.clear();
        cached_report_.clear();
        frame_counter_ = 0;
        final_report_requested_ = false;
    }
    void set_sensor_dimensions(int w, int h) override {
        sensor_w_ = w;
        sensor_h_ = h;
        rebuild();
    }
};

// --- Per-category factory (called by create_algo_backend in backend_factory.cpp)
std::unique_ptr<AlgoBackend> create_analytics_backend(const std::string& name,
                                          int width, int height) {
    if (name == "event_to_video")              return std::make_unique<EventToVideoBackend>(width, height);
    if (name == "flow_statistics")             return std::make_unique<FlowStatisticsBackend>(width, height);
    if (name == "isi_analyzer")                return std::make_unique<ISIAnalyzerBackend>(width, height);
    if (name == "sensor_self_test")            return std::make_unique<SensorSelfTestBackend>(width, height);
    return nullptr;
}

} // namespace gui
