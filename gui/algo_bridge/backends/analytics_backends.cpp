// gui/algo_bridge/backends/analytics_backends.cpp — frame producers + analyzers
// (design §3.4). Split from the former algo_backend.cpp monolith.
// Contains: EventToVideo, FlowStatistics, ISIAnalyzer.

#include "algo_bridge/algo_backend.h"
#include "algo_bridge/backends/backend_common.h"

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "algo/analytics/event_to_video.h"
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
    // decay_tau_ms was removed (audit §4-M7): the algo's decay semantics are
    // ineffective, so the param is no longer registered or forwarded.
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
    Preprocessor preproc_;

    // --- Worker-thread machinery (audit §五-C1) ---------------------------
    // get_frame() runs ONNX inference / 100+ Bardow iterations (tens to
    // hundreds of ms). It used to execute inside pull_result() on the GUI
    // thread while push_events() (SDK data thread) blocked on the same
    // AlgoInstance::mutex_ — freezing the UI and back-pressuring capture.
    // Now: push_events() only preprocesses and appends the batch to a
    // bounded queue; a dedicated worker thread feeds batches to the
    // algorithm and runs get_frame(); pull_result() just copies the latest
    // finished frame from a double buffer.
    //
    // Locks (no path ever holds two of them in reverse order):
    //  - algo_mutex_: every access to algo_ (worker process/get_frame;
    //    GUI-thread setters, reset, rebuild swap, release).
    //  - preproc_mutex_: preproc_/roi_/sensor dims and the push-side buffers
    //    (passthrough_/roi_events_). Held only for the duration of one
    //    batch's preprocessing / a geometry read — never during inference.
    //  - queue_mutex_ + queue_cv_: the bounded batch queue.
    //  - latest_mutex_: the double-buffered latest frame + status.
    // rebuild()/release_resources()/ensure_algo() are always invoked under
    // AlgoInstance::mutex_ (the bridge serializes set_param/push/pull/reset/
    // set_enabled/set_sensor_dimensions), so they cannot run concurrently
    // with each other; only the worker thread runs outside that umbrella.
    std::mutex algo_mutex_;
    std::mutex preproc_mutex_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::vector<gui_algo::Event>> queue_;
    /// Queue bound: when the worker falls behind (inference slower than the
    /// event rate), push drops the OLDEST batch instead of blocking, and
    /// counts the drop (surfaced in the status line).
    static constexpr std::size_t kMaxQueuedBatches = 8;
    std::atomic<std::uint64_t> dropped_batches_{0};
    std::mutex latest_mutex_;
    cv::Mat latest_frame_;       ///< Latest finished frame (worker-written).
    std::string latest_status_;  ///< Status composed together with the frame.
    // Output-fps throttle state (event time). Atomics so reset()/rebuild()
    // can re-arm them from the GUI thread while the worker is running.
    std::atomic<Metavision::timestamp> worker_last_frame_t_{0};
    std::atomic<bool> worker_have_frame_{false};

public:
    EventToVideoBackend(int w, int h) : sensor_w_(w), sensor_h_(h) {
        roi_.compute(sensor_w_, sensor_h_);
        preproc_.halve_coords_ = true;
        rebuild();
        start_worker();
    }
    /// Stops the worker BEFORE any member it touches is destroyed.
    ~EventToVideoBackend() override { stop_worker(); }
    void rebuild() {
        int aw, ah, f;
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
            aw = roi_.enabled ? roi_.rw : sensor_w_;
            ah = roi_.enabled ? roi_.rh : sensor_h_;
            preproc_.init(aw, ah);
            f = preproc_.factor();
        }
        // Build the new algorithm OFF algo_mutex_ (the ONNX model load in
        // set_model_path may take hundreds of ms); the worker keeps using
        // the old instance until the swap at the end.
        auto fresh = std::make_unique<gui_algo::EventToVideo>(aw / f, ah / f, mode_, output_fps_);
        // BardowVariational
        fresh->set_window_ms(window_ms_);
        fresh->set_delta_t_ms(delta_t_ms_);
        fresh->set_theta(theta_);
        fresh->set_num_iterations(num_iterations_);
        fresh->set_lambda1(lambda1_);
        fresh->set_lambda2(lambda2_);
        fresh->set_lambda3(lambda3_);
        fresh->set_lambda4(lambda4_);
        fresh->set_lambda5(lambda5_);
        fresh->set_lambda6(lambda6_);
        // InteractingMaps
        fresh->set_relaxation_step(relaxation_step_);
        fresh->set_im_iterations(im_iterations_);
        fresh->set_fov_deg(fov_deg_);
        // E2VID: load the ONNX model if a path is set. An empty path keeps
        // the heuristic fallback; a non-empty path triggers load_model which
        // may fail silently and also fall back (BUG-G9: comment corrected —
        // the model IS loaded here in rebuild(), not deferred).
        if (!model_path_.empty()) fresh->set_model_path(model_path_);
        fresh->set_e2vid_num_bins(e2vid_num_bins_);
        // Re-sync from the algo: when a model is loaded, set_num_bins ignores
        // the caller's value and uses model_num_bins_ (e2vid_inference.h).
        // Without this re-sync, e2vid_num_bins_ would stay at the stale
        // default/user value and get_param("num_bins") would return the
        // wrong value (BUG-N11).
        e2vid_num_bins_ = fresh->e2vid_num_bins();
        fresh->set_e2vid_auto_hdr(e2vid_auto_hdr_);
        // 1/4 downsample is handled by the shared Preprocessor (which halves
        // event coordinates before they reach the algo). The algo's internal
        // downsample flags must stay OFF to avoid double-halving.
        fresh->set_e2vid_downsample(false);
        fresh->set_downsample(false);
        fresh->set_unsharp_amount(unsharp_amount_);
        fresh->set_unsharp_sigma(unsharp_sigma_);
        fresh->set_bilateral_sigma(bilateral_sigma_);
        {
            std::lock_guard<std::mutex> lk(algo_mutex_);
            algo_ = std::move(fresh);
        }
        // Drop queued batches and the published frame of the previous
        // geometry/instance; also re-arm the output-fps throttle.
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(latest_mutex_);
            latest_frame_.release();
            latest_status_.clear();
        }
        worker_have_frame_.store(false, std::memory_order_relaxed);
    }
    void set_param(const std::string& k, const std::string& v) override {
        // algo_ is concurrently used by the worker thread — serialize every
        // direct algo mutation through algo_mutex_ (audit §五-C1). Branches
        // that call rebuild() must NOT hold it (rebuild takes it internally).
        auto with_algo = [this](const std::function<void(gui_algo::EventToVideo&)>& fn) {
            std::lock_guard<std::mutex> lk(algo_mutex_);
            if (algo_) fn(*algo_);
        };
        // preproc_/roi_ are concurrently read by push_events() (SDK thread)
        // and the worker — guard every mutation with preproc_mutex_.
        bool preproc_hit;
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
            preproc_hit = preproc_.set_param(k, v);
        }
        if (preproc_hit) {
            if (k == "preproc_downsample") rebuild();
            return;
        }
        // Snapshot ROI state BEFORE any param mutation so the rebuild-skip
        // logic below compares old vs. new against the true prior state.
        // Without this, roi_enabled toggling would read the NEW roi_.enabled
        // for both old_aw and new_aw, falsely skipping rebuild() and leaving
        // the algorithm with mismatched dimensions vs. incoming events (N2).
        // (Unlocked read: serialized with push_events via AlgoInstance::mutex_;
        // the worker only reads roi_ as well.)
        const bool prev_roi_enabled = roi_.enabled;
        const int prev_roi_rw = roi_.rw;
        const int prev_roi_rh = roi_.rh;
        bool need_rebuild = false;
        if (k == "mode") {
            int m = to_i(v);
            if (m >= 0 && m <= 2) {
                mode_ = static_cast<gui_algo::EventToVideo::Mode>(m);
                with_algo([this](gui_algo::EventToVideo& a) { a.set_mode(mode_); });
            }
        } else if (k == "output_fps") {
            output_fps_ = to_i(v);
            with_algo([this](gui_algo::EventToVideo& a) { a.set_output_fps(output_fps_); });
        } else if (k == "window_ms") {
            window_ms_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_window_ms(window_ms_); });
        } else if (k == "delta_t_ms") {
            delta_t_ms_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_delta_t_ms(delta_t_ms_); });
        } else if (k == "theta") {
            theta_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_theta(theta_); });
        } else if (k == "num_iterations") {
            num_iterations_ = to_i(v);
            with_algo([this](gui_algo::EventToVideo& a) { a.set_num_iterations(num_iterations_); });
        } else if (k == "lambda1") {
            lambda1_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda1(lambda1_); });
        } else if (k == "lambda2") {
            lambda2_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda2(lambda2_); });
        } else if (k == "lambda3") {
            lambda3_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda3(lambda3_); });
        } else if (k == "lambda4") {
            lambda4_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda4(lambda4_); });
        } else if (k == "lambda5") {
            lambda5_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda5(lambda5_); });
        } else if (k == "lambda6") {
            lambda6_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_lambda6(lambda6_); });
        } else if (k == "relaxation_step") {
            relaxation_step_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_relaxation_step(relaxation_step_); });
        } else if (k == "im_iterations") {
            im_iterations_ = to_i(v);
            with_algo([this](gui_algo::EventToVideo& a) { a.set_im_iterations(im_iterations_); });
        } else if (k == "fov_deg") {
            fov_deg_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_fov_deg(fov_deg_); });
        } else if (k == "model_path") {
            model_path_ = v;
            // Hot-swapping the model on the live algo would race in-flight
            // inference on the worker thread, so reload via rebuild() (same
            // cost — set_model_path IS the ONNX load). rebuild() re-syncs
            // e2vid_num_bins_ from the freshly loaded model, so the persisted
            // num_bins stays the model-determined value (BUG-G2/N11).
            rebuild();
        } else if (k == "num_bins") {
            e2vid_num_bins_ = to_i(v);
            std::lock_guard<std::mutex> lk(algo_mutex_);
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
            with_algo([this](gui_algo::EventToVideo& a) { a.set_e2vid_auto_hdr(e2vid_auto_hdr_); });
        } else if (k == "unsharp_amount") {
            unsharp_amount_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_unsharp_amount(unsharp_amount_); });
        } else if (k == "unsharp_sigma") {
            unsharp_sigma_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_unsharp_sigma(unsharp_sigma_); });
        } else if (k == "bilateral_sigma") {
            bilateral_sigma_ = static_cast<float>(to_d(v));
            with_algo([this](gui_algo::EventToVideo& a) { a.set_bilateral_sigma(bilateral_sigma_); });
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
            std::string prev;
            {
                std::lock_guard<std::mutex> lk(preproc_mutex_);
                prev = preproc_.get_param("preproc_downsample");
                preproc_.set_param("preproc_downsample", v);
            }
            if (prev != v) rebuild();
            return;
        } else if (k == "roi_enabled") {
            { std::lock_guard<std::mutex> lk(preproc_mutex_); roi_.enabled = to_b(v); }
            need_rebuild = true;
        } else if (k == "roi_x") {
            { std::lock_guard<std::mutex> lk(preproc_mutex_); roi_.x = to_i(v); }
            need_rebuild = true;
        } else if (k == "roi_y") {
            { std::lock_guard<std::mutex> lk(preproc_mutex_); roi_.y = to_i(v); }
            need_rebuild = true;
        } else if (k == "roi_w") {
            { std::lock_guard<std::mutex> lk(preproc_mutex_); roi_.w = to_i(v); }
            need_rebuild = true;
        } else if (k == "roi_h") {
            { std::lock_guard<std::mutex> lk(preproc_mutex_); roi_.h = to_i(v); }
            need_rebuild = true;
        }
        if (need_rebuild) {
            // Only rebuild (which reloads the ONNX model) when the effective
            // dimensions actually change. ROI position changes (x, y) don't
            // affect the algorithm dimensions — they only shift which events
            // are cropped — so skip the expensive rebuild (N2).
            // Use the snapshot taken at entry — roi_.enabled may have been
            // toggled by this very call, so reading it now would compare the
            // NEW state against itself and falsely skip rebuild().
            bool dims_changed;
            {
                std::lock_guard<std::mutex> lk(preproc_mutex_);
                const int old_aw = prev_roi_enabled ? prev_roi_rw : sensor_w_;
                const int old_ah = prev_roi_enabled ? prev_roi_rh : sensor_h_;
                roi_.compute(sensor_w_, sensor_h_);
                const int new_aw = roi_.enabled ? roi_.rw : sensor_w_;
                const int new_ah = roi_.enabled ? roi_.rh : sensor_h_;
                dims_changed = (new_aw != old_aw || new_ah != old_ah);
            }
            if (dims_changed) {
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
        if (k == "relaxation_step") return from_d(relaxation_step_);
        if (k == "im_iterations") return from_i(im_iterations_);
        if (k == "fov_deg") return from_d(fov_deg_);
        if (k == "model_path") return model_path_;
        if (k == "model_loaded") {
            // Pseudo-param (not registered) for the panel's one-shot error
            // hint (§5-H1): only meaningful in E2VID mode; empty = N/A.
            return (mode_ == gui_algo::EventToVideo::Mode::E2VID && algo_)
                       ? from_b(algo_->e2vid_model_loaded()) : std::string{};
        }
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
        ensure_algo();  // lazily rebuild after release_resources()
        // SDK data thread: preprocess ONLY, then enqueue — never touch algo_
        // here (audit §五-C1). The worker runs process()/get_frame().
        std::vector<gui_algo::Event> batch;
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
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
            if (n > 0) batch.assign(ev, ev + n);
        }
        if (batch.empty()) return;
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            // Bounded queue: a full queue means inference is slower than the
            // event rate — drop the OLDEST batch (keep the display current)
            // and count it, instead of blocking the SDK thread.
            if (queue_.size() >= kMaxQueuedBatches) {
                queue_.pop_front();
                ++dropped_batches_;
            }
            queue_.push_back(std::move(batch));
        }
        queue_cv_.notify_one();
    }
    AlgoResult pull_result() override {
        ensure_algo();  // lazily rebuild after release_resources()
        // GUI thread: copy the latest finished frame out of the double
        // buffer — pull NEVER triggers inference (audit §五-C1).
        AlgoResult r;
        {
            std::lock_guard<std::mutex> lk(latest_mutex_);
            if (!latest_frame_.empty()) {
                r.has_frame = true;
                r.frame = latest_frame_.clone();
                r.status = latest_status_;
            }
        }
        if (!r.has_frame) {
            // No finished frame yet (worker warming up, or just rebuilt).
            // StandaloneStrategy skips the frame update when has_frame is
            // false and still shows the status line.
            r.status = "e2v: waiting for first frame";
        }
        // filtered_events intentionally left empty: this is a frame producer
        // (Standalone display), and its post-ROI events use ROI-RELATIVE
        // coordinates — feeding them into the §五-G1 main-display
        // re-injection would splatter the ROI content at the frame origin.
        return r;
    }
    void reset() override {
        {
            std::lock_guard<std::mutex> lk(algo_mutex_);
            if (algo_) algo_->reset();
        }
        // Drop stale queued batches / published frame so no pre-reset data
        // is processed or shown; re-arm the output-fps throttle.
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(latest_mutex_);
            latest_frame_.release();
            latest_status_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
            passthrough_.clear();
            roi_events_.clear();
        }
        worker_have_frame_.store(false, std::memory_order_relaxed);
    }
    void set_sensor_dimensions(int w, int h) override {
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
            sensor_w_ = w;
            sensor_h_ = h;
            roi_.compute(sensor_w_, sensor_h_);
        }
        rebuild();
    }
    /// @brief Releases the ONNX session and all frame buffers while the
    /// instance is disabled (audit §5-E3). Stops the worker FIRST so it
    /// cannot touch algo_ during teardown; the algorithm (and the worker)
    /// is rebuilt/restarted lazily by ensure_algo() on the first push/pull
    /// after re-enable.
    void release_resources() override {
        stop_worker();
        {
            std::lock_guard<std::mutex> lk(algo_mutex_);
            algo_.reset();
        }
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(latest_mutex_);
            latest_frame_.release();
            latest_status_.clear();
        }
        {
            std::lock_guard<std::mutex> lk(preproc_mutex_);
            passthrough_.clear();
            roi_events_.clear();
        }
    }
private:
    /// Lazily rebuilds the algorithm (and restarts the worker) after
    /// release_resources() dropped them. Always called under
    /// AlgoInstance::mutex_ by the bridge, so it cannot race
    /// rebuild()/release_resources().
    void ensure_algo() {
        if (!algo_) rebuild();
        start_worker();
    }
    void start_worker() {
        if (worker_.joinable()) return;  // already running
        stop_.store(false, std::memory_order_relaxed);
        worker_ = std::thread([this] { worker_loop(); });
    }
    void stop_worker() {
        stop_.store(true, std::memory_order_relaxed);
        queue_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }
    /// Worker loop (audit §五-C1): pops queued batches, feeds them to the
    /// algorithm, and runs the heavy get_frame() at most once per
    /// output_fps interval. The throttle uses EVENT timestamps — consistent
    /// with the algorithm's event-time sliding window — so the cadence is
    /// correct for both live streams and (possibly non-realtime) file
    /// playback. The finished frame + status are published to the latest_
    /// double buffer for pull_result().
    void worker_loop() {
        for (;;) {
            std::vector<gui_algo::Event> batch;
            {
                std::unique_lock<std::mutex> lk(queue_mutex_);
                queue_cv_.wait(lk, [this] {
                    return stop_.load(std::memory_order_relaxed) || !queue_.empty();
                });
                if (stop_.load(std::memory_order_relaxed)) return;
                batch = std::move(queue_.front());
                queue_.pop_front();
            }
            cv::Mat frame;
            std::string status;
            {
                std::lock_guard<std::mutex> lk(algo_mutex_);
                if (!algo_) continue;  // released — ensure_algo() restarts us
                algo_->process(batch.data(), batch.size());
                const Metavision::timestamp t = batch.back().t;
                if (worker_have_frame_.load(std::memory_order_relaxed) &&
                    t - worker_last_frame_t_.load(std::memory_order_relaxed) <
                        algo_->frame_interval_us()) {
                    continue;  // events fed; not yet time for the next frame
                }
                worker_last_frame_t_.store(t, std::memory_order_relaxed);
                worker_have_frame_.store(true, std::memory_order_relaxed);
                frame = algo_->get_frame();
                // Upsample + status need the preproc/ROI geometry, which the
                // push thread mutates — read it under preproc_mutex_ (held
                // there only for one batch's preprocessing, so this never
                // stalls the worker on inference timescales).
                {
                    std::lock_guard<std::mutex> plk(preproc_mutex_);
                    const int f = preproc_.factor();
                    if (f > 1 && !frame.empty()) {
                        const int aw = roi_.enabled ? roi_.rw : sensor_w_;
                        const int ah = roi_.enabled ? roi_.rh : sensor_h_;
                        cv::resize(frame, frame, cv::Size(aw, ah), 0, 0, cv::INTER_NEAREST);
                    }
                    status = "e2v: " + std::to_string(algo_->width()) + "x" +
                             std::to_string(algo_->height()) +
                             (roi_.enabled ? " (ROI)" : " (full)");
                }
                // Surface the model state (audit §5-H1): a failed ONNX load
                // silently falls back to the heuristic path; users must be
                // able to tell which reconstruction they are looking at.
                if (algo_->mode() == gui_algo::EventToVideo::Mode::E2VID) {
                    status += algo_->e2vid_model_loaded() ? " model=loaded"
                                                          : " model=heuristic";
                }
            }
            const std::uint64_t drops = dropped_batches_.load(std::memory_order_relaxed);
            if (drops > 0) status += " dropped=" + std::to_string(drops);
            {
                std::lock_guard<std::mutex> lk(latest_mutex_);
                // frame is a freshly built Mat that is never mutated after
                // publishing, so pull_result() can safely clone() from it.
                latest_frame_ = frame;
                latest_status_ = std::move(status);
            }
        }
    }
};

/// FlowStatistics backend — requires ground-truth flow samples (not available
/// in real-time). Counts events only (audit §3-9: the FlowStatistics member
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
        // the ROI geometry (audit §5-D1).
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
        // (audit §5-D4): apply_global_roi fires 5 set_param calls, and a
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
