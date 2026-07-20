// algo/analytics/e2vid/e2vid_inference.h — E2VID neural-network inference.
//
// Design §4.4.2 (E2VID DL path). Wraps the E2VID / UNet-Recurrent model
// inference for event-based grayscale reconstruction. Ported from
// rpg_e2vid (image_reconstructor.py, model/unet.py).
//
// Architecture (rpg_e2vid):
//   - Input: 1 x num_bins x H x W event voxel grid (float32)
//   - Model: UNet or UNetRecurrent (ConvLSTM/ConvGRU) with skip connections,
//     4 encoders, 2 residual blocks, 4 decoders, sigmoid output
//   - Output: 1 x 1 x H x W grayscale image in [0, 1]
//
// Backends:
//   - ONNX Runtime (preferred): load exported .onnx model, run inference.
//     Conditionally compiled when ONNX Runtime is found via CMake.
//   - Heuristic fallback (always available): when no model is loaded,
//     reconstructs by summing voxel bins and applying sigmoid-like mapping.
//     This produces a crude but usable preview without the neural network.
//
// The CropParameters logic (padding to power-of-2 divisible sizes) is also
// implemented to match the original rpg_e2vid preprocessing. Header-only.

#ifndef GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H
#define GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "algo/common/event.h"
#include "algo/analytics/e2vid/event_voxel_grid.h"

// Conditional ONNX Runtime support.
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
#include <onnxruntime_cxx_api.h>
#endif

namespace gui_algo {

/// @brief Crop/padding parameters for UNet (matches rpg_e2vid CropParameters).
struct E2VIDCropParams {
    int width{0};
    int height{0};
    int crop_width{0};   ///< Padded width (divisible by 2^num_encoders).
    int crop_height{0};  ///< Padded height (divisible by 2^num_encoders).
    int pad_top{0};
    int pad_bottom{0};
    int pad_left{0};
    int pad_right{0};

    /// @brief Computes crop parameters for a given sensor size and UNet depth.
    static E2VIDCropParams compute(int width, int height, int num_encoders) {
        E2VIDCropParams p;
        p.width = width;
        p.height = height;
        const int factor = 1 << num_encoders;  // 2^num_encoders
        p.crop_width = optimal_crop_size(width, factor);
        p.crop_height = optimal_crop_size(height, factor);
        p.pad_top = (p.crop_height - height + 1) / 2;
        p.pad_bottom = (p.crop_height - height) / 2;
        p.pad_left = (p.crop_width - width + 1) / 2;
        p.pad_right = (p.crop_width - width) / 2;
        return p;
    }

    /// @brief Pads a CV_32FC1 image (HxW) to crop size using reflection.
    /// Uses BORDER_REFLECT_101 to match PyTorch's ReflectionPad2d semantics
    /// (edge sample is NOT repeated: gfedcb|abcdefgh|gfedcba).
    cv::Mat pad(const cv::Mat& img) const {
        cv::Mat padded;
        cv::copyMakeBorder(img, padded,
                           pad_top, pad_bottom, pad_left, pad_right,
                           cv::BORDER_REFLECT_101);
        return padded;
    }

    /// @brief Crops the center region back to the original sensor size.
    cv::Mat crop(const cv::Mat& img) const {
        const int cx = crop_width / 2;
        const int cy = crop_height / 2;
        const int x0 = cx - width / 2;
        const int y0 = cy - height / 2;
        return img(cv::Rect(x0, y0, width, height)).clone();
    }

private:
    static int optimal_crop_size(int max_size, int factor) {
        int crop = factor;
        while (crop < max_size) crop += factor;
        return crop;
    }
};

/// @brief E2VID model inference wrapper with ONNX Runtime and heuristic backend.
class E2VIDInference {
public:
    /// @brief Constructs the inference engine.
    /// @param width,height Sensor dimensions.
    /// @param num_bins Number of event tensor temporal bins (E2VID default: 5).
    /// @param num_encoders UNet encoder depth (default: 4).
    E2VIDInference(int width, int height, int num_bins = 5,
                   int num_encoders = 4)
        : width_(width), height_(height),
          num_bins_(clamp_bins(num_bins)),
          num_encoders_(num_encoders),
          crop_(E2VIDCropParams::compute(
              effective_dim(width), effective_dim(height), num_encoders)),
          full_crop_(E2VIDCropParams::compute(width, height, num_encoders)),
          voxel_grid_(effective_dim(width), effective_dim(height), num_bins_) {}

    /// @brief Loads an ONNX model from file.
    /// @return true if the model was loaded successfully.
    ///
    /// On success the number of input bins (num_bins_) is synchronised to the
    /// model's first-input channel dimension. This mirrors rpg_e2vid, where
    /// num_bins is a property of the model (config['num_bins'] / model.num_bins)
    /// rather than a free user parameter — see run_reconstruction.py:55 and
    /// model/model.py:14. Letting the user freely change num_bins after a model
    /// is loaded would mismatch the model's input channels and break inference.
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    bool load_model(const std::string& model_path) {
        try {
            env_ = std::make_unique<Ort::Env>(
                ORT_LOGGING_LEVEL_WARNING, "e2vid");
            Ort::SessionOptions session_opts;
            // Use all available CPU cores for ONNX inference (capped at 8 to
            // avoid oversubscription on high-core machines). The E2VID
            // UNetRecurrent model is compute-bound on Conv/MatMul ops, which
            // ONNX Runtime parallelises across the intra-op thread pool.
            // Single-thread (the previous setting) was the main bottleneck.
            const unsigned hw_threads = std::thread::hardware_concurrency();
            const int num_threads = static_cast<int>(
                hw_threads > 0 ? (hw_threads <= 8 ? hw_threads : 8) : 4);
            session_opts.SetIntraOpNumThreads(num_threads);
            session_opts.SetGraphOptimizationLevel(
                GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_ = std::make_unique<Ort::Session>(
                *env_, model_path.c_str(), session_opts);
            model_path_ = model_path;
            model_loaded_ = true;
            sync_num_bins_from_model();
            // Cache MemoryInfo (constant for the session lifetime).
            mem_info_ = std::make_unique<Ort::MemoryInfo>(
                Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            // Cache input/output names once (avoids 14 string allocations
            // per frame in infer_onnx()).
            Ort::AllocatorWithDefaultOptions allocator;
            const std::size_t n_in = session_->GetInputCount();
            const std::size_t n_out = session_->GetOutputCount();
            input_name_owners_.clear();
            output_name_owners_.clear();
            cached_input_names_.clear();
            cached_output_names_.clear();
            input_name_owners_.reserve(n_in);
            cached_input_names_.reserve(n_in);
            for (std::size_t i = 0; i < n_in; ++i) {
                input_name_owners_.push_back(
                    session_->GetInputNameAllocated(i, allocator));
                cached_input_names_.push_back(input_name_owners_.back().get());
            }
            output_name_owners_.reserve(n_out);
            cached_output_names_.reserve(n_out);
            for (std::size_t i = 0; i < n_out; ++i) {
                output_name_owners_.push_back(
                    session_->GetOutputNameAllocated(i, allocator));
                cached_output_names_.push_back(
                    output_name_owners_.back().get());
            }
            return true;
        } catch (const Ort::Exception&) {
            model_loaded_ = false;
            return false;
        }
    }
#else
    bool load_model(const std::string& model_path) {
        model_path_ = model_path;
        model_loaded_ = false;  // ONNX Runtime not available
        return false;
    }
#endif

    /// @brief Returns true if a model is loaded and ready for inference.
    bool is_model_loaded() const { return model_loaded_; }

    /// @brief Runs inference on a batch of events.
    /// @param events Event array.
    /// @param n Number of events.
    /// @return ONNX path: CV_32FC1 padded image in [0,1] (full_crop_h x
    ///         full_crop_w) — the caller is responsible for cropping back
    ///         to sensor size. Heuristic path: CV_8UC1 sensor-sized image.
    cv::Mat infer(const Event* events, std::size_t n) {
        if (events == nullptr || n == 0 || width_ <= 0 || height_ <= 0) {
            return cv::Mat::zeros(height_, width_, CV_8UC1);
        }

        // 1. Build voxel grid. When downsample_ is on, keep only events whose
        //    x AND y are both even, and remap (x, y) → (x/2, y/2) into the
        //    half-size grid. For a 128×128 ROI this produces a 64×64 grid,
        //    cutting ONNX inference cost ~4×.
        if (downsample_) {
            const Event* src = events;
            std::size_t src_n = n;
            if (src_n > downsampled_events_.size()) {
                downsampled_events_.reserve(src_n / 4 + 16);
            }
            downsampled_events_.clear();
            for (std::size_t i = 0; i < src_n; ++i) {
                const auto& e = src[i];
                if ((e.x & 1u) == 0 && (e.y & 1u) == 0) {
                    downsampled_events_.push_back(
                        Event(static_cast<std::uint16_t>(e.x >> 1),
                              static_cast<std::uint16_t>(e.y >> 1),
                              e.p, e.t));
                }
            }
            voxel_grid_.build(downsampled_events_.data(),
                              downsampled_events_.size());
        } else {
            voxel_grid_.build(events, n);
        }
        if (normalize_input_) {
            voxel_grid_.normalize();
        }

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        if (model_loaded_ && session_) {
            cv::Mat result = infer_onnx();  // effective crop size
            // Upsample to full crop dimensions so crop_to_sensor works
            // uniformly regardless of downsample_.
            if (downsample_ &&
                (result.rows != full_crop_.crop_height ||
                 result.cols != full_crop_.crop_width)) {
                cv::resize(result, result,
                           cv::Size(full_crop_.crop_width,
                                    full_crop_.crop_height),
                           0, 0, cv::INTER_NEAREST);
            }
            return result;
        }
#endif
        // Fallback: heuristic reconstruction from voxel grid.
        return infer_heuristic();
    }

    /// @brief Crops a padded image back to the sensor dimensions.
    /// No-op if the image already matches the sensor size (heuristic path).
    /// When downsample_ is on, the image has been upsampled to full_crop_
    /// dimensions, so use full_crop_ for cropping.
    cv::Mat crop_to_sensor(const cv::Mat& img) const {
        if (img.rows == height_ && img.cols == width_) {
            return img;
        }
        if (downsample_) {
            return full_crop_.crop(img);
        }
        return crop_.crop(img);
    }

    /// @brief Sets whether to normalize the input voxel grid.
    void set_normalize_input(bool v) { normalize_input_ = v; }
    bool normalize_input() const { return normalize_input_; }

    /// @brief Sets whether to 1/4-downsample the ROI before inference.
    /// When on, only events with even x AND even y are kept, and coordinates
    /// are halved. For 128×128 ROI this produces a 64×64 grid, ~4× faster.
    void set_downsample(bool v) {
        if (downsample_ == v) return;
        downsample_ = v;
        rebuild_effective_buffers();
    }
    bool downsample() const { return downsample_; }

    /// @brief Sets the hot-pixel mask for the voxel grid preprocessor.
    /// Accepts either effective-resolution (eff HxW) or full sensor-resolution
    /// (HxW) masks; the latter is 2x2-downsampled by the voxel grid when
    /// downsampling is active (§四-M3). Mismatched sizes are rejected — check
    /// hot_pixel_mask_rejected().
    void set_hot_pixel_mask(const std::vector<std::uint8_t>& mask) {
        hot_pixel_mask_ = mask;  // cache so it survives num_bins changes
        voxel_grid_.set_hot_pixel_mask(mask);
    }
    void clear_hot_pixel_mask() {
        hot_pixel_mask_.clear();
        voxel_grid_.clear_hot_pixel_mask();
    }
    /// @brief True if the last mask was rejected (size mismatch, §四-M3).
    bool hot_pixel_mask_rejected() const {
        return voxel_grid_.hot_mask_rejected();
    }

    void set_num_bins(int b) {
        int target = clamp_bins(b);
        // When a model is loaded, num_bins is dictated by the model's input
        // channels (rpg_e2vid: model.num_bins). Ignore the caller's value so
        // the voxel grid always matches the model — otherwise the ONNX input
        // shape would mismatch and inference would fail.
        if (model_loaded_ && model_num_bins_ > 0) {
            target = model_num_bins_;
        }
        num_bins_ = target;
        rebuild_effective_buffers();
    }
    int num_bins() const { return num_bins_; }

    const std::string& model_path() const { return model_path_; }

    void reset() {
        voxel_grid_.reset();
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        // Clear recurrent states if applicable.
        prev_states_.clear();
        state_buffers_.clear();
        input_buffer_.clear();
#endif
    }

    int width() const { return width_; }
    int height() const { return height_; }

private:
    static int clamp_bins(int b) {
        // Match EventVoxelGrid's clamp range: [1, 20].
        if (b < 1) return 1;
        if (b > 20) return 20;
        return b;
    }

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    /// @brief Reads num_bins and num_encoders from the loaded ONNX model.
    /// rpg_e2vid determines num_bins from the model config (model.num_bins);
    /// the ONNX equivalent is the 2nd dimension of the first input tensor
    /// (shape = [N, C, H, W]). Updates model_num_bins_ and re-syncs the voxel
    /// grid. Best-effort: on any failure keeps the existing num_bins_.
    ///
    /// Also infers num_encoders from the input count:
    ///   E2VIDRecurrent: n_inputs = 1 + 2 * num_encoders (event + h/c per level)
    ///   E2VID (non-recurrent): n_inputs = 1 (num_encoders stays at constructor default)
    /// and recomputes CropParameters to match (rpg_e2vid: model.num_encoders).
    void sync_num_bins_from_model() {
        if (!session_) return;
        try {
            Ort::AllocatorWithDefaultOptions allocator;
            auto info = session_->GetInputTypeInfo(0);
            auto shape = info.GetTensorTypeAndShapeInfo().GetShape();
            // shape = [N, C, H, W]; C is the num_bins channel dimension.
            if (shape.size() >= 2 && shape[1] > 0) {
                model_num_bins_ = static_cast<int>(shape[1]);
                if (model_num_bins_ != num_bins_) {
                    num_bins_ = model_num_bins_;
                }
            }
            // Infer num_encoders from input count (E2VIDRecurrent only).
            const std::size_t n_inputs = session_->GetInputCount();
            if (n_inputs > 1) {
                int inferred = static_cast<int>((n_inputs - 1) / 2);
                if (inferred > 0 && inferred != num_encoders_) {
                    num_encoders_ = inferred;
                }
            }
            // Rebuild all effective-size buffers (voxel grid, crop, states).
            rebuild_effective_buffers();
        } catch (const Ort::Exception&) {
            // Keep existing num_bins_ (best-effort).
        }
    }

    /// @brief ONNX Runtime inference path.
    /// Returns the padded CV_32FC1 image in [0,1] (crop_h x crop_w).
    /// The caller crops it back to sensor size after postprocessing.
    /// Handles both plain UNet (1 input/1 output) and UNetRecurrent
    /// (N inputs/M outputs) by feeding zero-initialized states on the first
    /// call and persisting returned states across calls (matches rpg_e2vid's
    /// prev_states handling). Any Ort::Exception falls back to heuristic.
    cv::Mat infer_onnx() {
        const int ch = crop_.crop_height;
        const int cw = crop_.crop_width;

        try {
            // --- Reuse input buffer across frames (resize only on dim change) ---
            const std::size_t input_size =
                static_cast<std::size_t>(num_bins_) * ch * cw;
            if (cached_crop_w_ != cw || cached_crop_h_ != ch ||
                cached_num_bins_ != num_bins_ ||
                input_buffer_.size() != input_size) {
                input_buffer_.assign(input_size, 0.0f);
                cached_crop_w_ = cw;
                cached_crop_h_ = ch;
                cached_num_bins_ = num_bins_;
                // State buffers must also be rebuilt when dims change.
                state_buffers_.clear();
            } else {
                std::fill(input_buffer_.begin(), input_buffer_.end(), 0.0f);
            }

            // Copy voxel grid into padded tensor (reflection padding).
            // voxel_grid_ is at effective dimensions (possibly downsampled).
            const int ew = eff_width();
            const int eh = eff_height();
            const float* grid = voxel_grid_.data();
            const int stride_hw = ew * eh;
            for (int b = 0; b < num_bins_; ++b) {
                cv::Mat bin(eh, ew, CV_32FC1,
                            const_cast<float*>(grid + b * stride_hw));
                cv::copyMakeBorder(bin, padded_buffer_,
                                   crop_.pad_top, crop_.pad_bottom,
                                   crop_.pad_left, crop_.pad_right,
                                   cv::BORDER_REFLECT_101);
                std::copy(padded_buffer_.begin<float>(), padded_buffer_.end<float>(),
                          input_buffer_.begin() +
                              static_cast<std::size_t>(b) * ch * cw);
            }

            std::array<std::int64_t, 4> input_shape = {1, num_bins_, ch, cw};

            const std::size_t n_inputs = session_->GetInputCount();

            // Build input Ort::Values. First input is always the event voxel
            // grid; subsequent inputs (if any) are recurrent state tensors.
            std::vector<Ort::Value> inputs;
            inputs.reserve(n_inputs);
            inputs.push_back(Ort::Value::CreateTensor<float>(
                *mem_info_, input_buffer_.data(), input_buffer_.size(),
                input_shape.data(), input_shape.size()));

            // Allocate zero-initialized state buffers only once (or after a
            // dimension change). On subsequent frames prev_states_ holds the
            // recurrent state and state_buffers_ is skipped entirely.
            const bool need_zero_states =
                state_buffers_.empty() &&
                (prev_states_.empty() || prev_states_.size() != n_inputs - 1);
            if (need_zero_states) {
                state_buffers_.clear();
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    auto info = session_->GetInputTypeInfo(i);
                    auto tensor_info = info.GetTensorTypeAndShapeInfo();
                    auto shape = tensor_info.GetShape();
                    if (shape.size() >= 4) {
                        const int level = static_cast<int>((i - 1) / 2);
                        const int divisor = (1 << (level + 1));
                        shape[0] = 1;
                        shape[2] = ch / divisor;
                        shape[3] = cw / divisor;
                    }
                    std::size_t total = 1;
                    for (auto d : shape) {
                        if (d <= 0) d = 1;
                        total *= static_cast<std::size_t>(d);
                    }
                    state_buffers_.emplace_back(total, 0.0f);
                    inputs.push_back(Ort::Value::CreateTensor<float>(
                        *mem_info_, state_buffers_.back().data(),
                        state_buffers_.back().size(), shape.data(),
                        shape.size()));
                }
            } else {
                // Pad inputs with placeholder tensors (will be replaced by
                // prev_states_ below, or by existing state_buffers_).
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    inputs.push_back(Ort::Value{nullptr});
                }
            }

            // If we have prev_states_ from a previous call, replace the zero
            // state tensors with the persisted states.
            if (!prev_states_.empty() && prev_states_.size() == n_inputs - 1) {
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    inputs[i] = std::move(prev_states_[i - 1]);
                }
                prev_states_.clear();
            } else if (!state_buffers_.empty()) {
                // Rebuild Ort::Value wrappers around existing state_buffers_.
                for (std::size_t i = 1; i < n_inputs; ++i) {
                    auto info = session_->GetInputTypeInfo(i);
                    auto tensor_info = info.GetTensorTypeAndShapeInfo();
                    auto shape = tensor_info.GetShape();
                    if (shape.size() >= 4) {
                        const int level = static_cast<int>((i - 1) / 2);
                        const int divisor = (1 << (level + 1));
                        shape[0] = 1;
                        shape[2] = ch / divisor;
                        shape[3] = cw / divisor;
                    }
                    inputs[i] = Ort::Value::CreateTensor<float>(
                        *mem_info_, state_buffers_[i - 1].data(),
                        state_buffers_[i - 1].size(), shape.data(),
                        shape.size());
                }
            }

            // Run inference (uses cached name pointers — no per-frame alloc).
            auto outputs = session_->Run(
                Ort::RunOptions{nullptr},
                cached_input_names_.data(), inputs.data(), inputs.size(),
                cached_output_names_.data(), cached_output_names_.size());

            // Persist recurrent states (outputs beyond the first image).
            prev_states_.clear();
            for (std::size_t i = 1; i < outputs.size(); ++i) {
                prev_states_.push_back(std::move(outputs[i]));
            }

            // Extract output: 1 x 1 x crop_h x crop_w.
            const float* output_data = outputs[0].GetTensorData<float>();
            auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
            const int out_h = static_cast<int>(out_shape[2]);
            const int out_w = static_cast<int>(out_shape[3]);
            cv::Mat output(out_h, out_w, CV_32FC1,
                           const_cast<float*>(output_data));
            return output.clone();  // deep copy (Ort owns the buffer)
        } catch (const Ort::Exception& e) {
            fprintf(stderr, "[e2vid] ONNX inference failed: %s (falling back "
                    "to heuristic, will retry next batch)\n", e.what());
            prev_states_.clear();
            return infer_heuristic();
        }
    }
#endif

    /// @brief Heuristic fallback: reconstructs from voxel grid without a model.
    /// Sums bins, applies sigmoid, returns CV_8UC1 at sensor dimensions.
    cv::Mat infer_heuristic() {
        const int eh = eff_height();
        const int ew = eff_width();
        // Sum across bins to get a 2D event count map.
        cv::Mat sum_img(eh, ew, CV_32FC1, cv::Scalar(0.0f));
        const float* grid = voxel_grid_.data();
        const int stride_hw = ew * eh;
        for (int b = 0; b < num_bins_; ++b) {
            cv::Mat bin(eh, ew, CV_32FC1,
                        const_cast<float*>(grid + b * stride_hw));
            sum_img += bin;
        }
        // Apply sigmoid: out = 1 / (1 + exp(-k * sum))
        cv::Mat sig;
        const float k = 0.5f;
        cv::exp(-k * sum_img, sig);
        sig = 1.0f / (1.0f + sig);
        cv::Mat gray;
        sig.convertTo(gray, CV_8UC1, 255.0);
        // Upsample if downsampled.
        if (downsample_ && (gray.rows != height_ || gray.cols != width_)) {
            cv::resize(gray, gray, cv::Size(width_, height_), 0, 0,
                       cv::INTER_NEAREST);
        }
        return gray;
    }

    int width_;
    int height_;
    int num_bins_;
    int model_num_bins_{0};  ///< Channel count read from the loaded ONNX model.
    int num_encoders_;
    // Default true: 1/4-downsample (halve width and height) before inference.
    // For 128×128 ROI → 64×64 grid, ~4× faster inference. Events whose x OR
    // y is odd are discarded; the rest are remapped (x/2, y/2).
    // Declared before crop_/voxel_grid_ because the constructor initialiser
    // list uses effective_dim() which reads downsample_.
    bool downsample_{true};
    E2VIDCropParams crop_;       ///< Crop at effective (possibly downsampled) dims.
    E2VIDCropParams full_crop_;  ///< Crop at original sensor dims (for upsampled output).
    EventVoxelGrid voxel_grid_;
    std::vector<std::uint8_t> hot_pixel_mask_;  ///< Cached for num_bins rebuilds.
    // Default false: rpg_e2vid README says --no-normalize "will improve speed
    // a bit, but might degrade the image quality a bit". The speed gain
    // matters more for real-time GUI usage than the minor quality drop.
    bool normalize_input_{false};
    bool model_loaded_{false};
    std::string model_path_;

    /// Effective dimensions after optional 1/4 downsampling.
    int eff_width() const { return effective_dim(width_); }
    int eff_height() const { return effective_dim(height_); }

    /// Returns half the dimension when downsample_ is on, else the dimension.
    int effective_dim(int d) const {
        return downsample_ ? (d > 0 ? (d + 1) / 2 : 0) : d;
    }

    /// Rebuilds voxel_grid_, crop_, and ONNX state buffers for the current
    /// effective dimensions. Called whenever downsample_ or num_bins_ or
    /// num_encoders_ changes. Preserves the hot-pixel mask.
    void rebuild_effective_buffers() {
        const int ew = eff_width();
        const int eh = eff_height();
        voxel_grid_ = EventVoxelGrid(ew, eh, num_bins_);
        if (!hot_pixel_mask_.empty()) {
            voxel_grid_.set_hot_pixel_mask(hot_pixel_mask_);
        }
        crop_ = E2VIDCropParams::compute(ew, eh, num_encoders_);
        full_crop_ = E2VIDCropParams::compute(width_, height_, num_encoders_);
#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
        state_buffers_.clear();
        prev_states_.clear();
        input_buffer_.clear();
        cached_crop_w_ = 0;  // force resize on next infer
#endif
    }

    /// Pre-allocated buffer for downsampled events (reused across frames).
    std::vector<Event> downsampled_events_;

#if defined(GUI_ALGO_HAS_ONNXRUNTIME)
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<Ort::Value> prev_states_;  ///< Recurrent states (UNetRecurrent).
    std::vector<std::vector<float>> state_buffers_;  ///< Backing storage for zero-init states.

    // --- Hot-path caches (avoid per-frame allocations) ---
    // input_buffer_ is reused across frames; resized only when crop/bin dims
    // change. Previously every infer_onnx() call did a 320 KB malloc+memset.
    std::vector<float> input_buffer_;
    cv::Mat padded_buffer_;  ///< Reusable padded image buffer (avoids per-bin allocation)
    // Input/output name strings are fetched once at load_model() time.
    std::vector<Ort::AllocatedStringPtr> input_name_owners_;
    std::vector<Ort::AllocatedStringPtr> output_name_owners_;
    std::vector<const char*> cached_input_names_;
    std::vector<const char*> cached_output_names_;
    // MemoryInfo is constant for the lifetime of the session.
    std::unique_ptr<Ort::MemoryInfo> mem_info_;
    // Crop dims last used to size input_buffer_ (resize only on change).
    int cached_crop_w_{0};
    int cached_crop_h_{0};
    int cached_num_bins_{0};
#endif
};

} // namespace gui_algo

#endif // GUI_ALGO_ANALYTICS_E2VID_E2VID_INFERENCE_H
