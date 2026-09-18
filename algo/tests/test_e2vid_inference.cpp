// test_e2vid_inference.cpp — E2VIDInference runtime-selection diagnostic
// (§4.4.2-GPU). Loads REAL converted ONNX models and exercises the OpenVINO
// GPU / ONNX Runtime CPU runtime selection, recurrent-state inference
// (ConvLSTM pairs, ConvGRU singles, full-res feedback), stateless models,
// reset, the dims-change recompile path and 2-channel (flow) outputs.
// Exit code = number of failed checks.
//
// Registered with CTest with models/e2vid_lightweight.onnx; skips (code 77)
// when the model file is not present (models/ is user-installed,
// git-ignored — same policy as third_party/onnxruntime).
//
// Run:
//   ./build/algo/tests/test_e2vid_inference models/e2vid_lightweight.onnx \
//       [models/e2vid_plus.onnx models/firenet_plus.onnx \
//        models/hypere2vid.onnx models/evflownet.onnx ...]

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "algo/common/event.h"
#include "algo/analytics/e2vid/e2vid_inference.h"

using gui_algo::E2VIDInference;
using gui_algo::Event;

static int g_failures = 0;

#define CHECK(cond, msg)                                          \
    do {                                                          \
        if (cond) {                                               \
            std::printf("  [PASS] %s\n", msg);                    \
        } else {                                                  \
            std::printf("  [FAIL] %s\n", msg);                    \
            ++g_failures;                                         \
        }                                                         \
    } while (0)

namespace {

/// Synthetic noise burst: n events scattered over w×h with both polarities —
/// enough spatial coverage for the network to produce a non-degenerate
/// result (values are asserted only for sanity, not content).
std::vector<Event> make_events(int n, int w, int h, std::uint64_t t0,
                               std::uint64_t dt_us) {
    std::vector<Event> evs;
    evs.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const std::uint16_t x = static_cast<std::uint16_t>((i * 73) % w);
        const std::uint16_t y = static_cast<std::uint16_t>((i * 131) % h);
        const int p = (i & 1) ? 1 : 0;
        const std::uint64_t t =
            t0 + static_cast<std::uint64_t>(i % 1000) * dt_us;
        evs.emplace_back(x, y, p, t);
    }
    return evs;
}

/// Neural-path output sanity: float depth (the heuristic fallback returns
/// CV_8UC1), all values finite, within a loose bound. Raw-head models
/// (HyperE2VID: no final activation) legitimately exceed [0,1].
bool neural_output_sane(const cv::Mat& m, bool expect_flow) {
    if (m.empty()) return false;
    if (m.depth() != CV_32F) return false;
    if (expect_flow && m.type() != CV_32FC2) return false;
    if (!expect_flow && m.channels() != 1) return false;
    std::vector<cv::Mat> ch;
    cv::split(m, ch);
    for (const auto& c : ch) {
        double lo = 0, hi = 0;
        cv::minMaxLoc(c, &lo, &hi);
        if (!std::isfinite(lo) || !std::isfinite(hi)) return false;
        if (lo < -3.0 || hi > 4.0) return false;
    }
    return true;
}

void check_model(const std::string& model_path) {
    std::printf("=== %s ===\n", model_path.c_str());
    if (FILE* f = std::fopen(model_path.c_str(), "rb")) {
        std::fclose(f);
    } else {
        std::printf("  SKIP: model not found\n");
        return;
    }
    const bool expect_flow =
        model_path.find("flow") != std::string::npos ||
        model_path.find("flownet") != std::string::npos;

    // GUI-equivalent geometry: 256x144 ROI with internal 1/4 downsample →
    // 128x72 effective inference (EVFlowNet needs /16: 128x72 ✓; FireNet+
    // is full-resolution with no divisibility constraint).
    E2VIDInference inf(256, 144, 5);
    inf.set_downsample(true);

    // --- 1) Default Auto policy: model must load; runtime reported. --------
    CHECK(inf.load_model(model_path), "Auto: model loads");
    CHECK(inf.is_model_loaded(), "Auto: model_loaded flag set");
    const std::string rt_auto = inf.active_runtime();
    std::printf("  active_runtime (Auto) = \"%s\"\n", rt_auto.c_str());
    CHECK(rt_auto == "gpu" || rt_auto == "cpu", "Auto: runtime reported");

    // --- 2) Recurrent/stateless inference: 12 steps. ------------------------
    bool all_valid = true;
    for (int step = 0; step < 12; ++step) {
        const std::vector<Event> evs = make_events(
            20000, 256, 144, static_cast<std::uint64_t>(step) * 50000, 50);
        cv::Mat out = inf.infer(evs.data(), evs.size());
        out = inf.crop_to_sensor(out);
        if (!neural_output_sane(out, expect_flow) || out.rows != 144 ||
            out.cols != 256) {
            all_valid = false;
        }
    }
    CHECK(all_valid, "Auto: 12 recurrent steps produce sane 256x144 frames");

    // --- 3) Explicit CPU policy. -------------------------------------------
    inf.set_device(E2VIDInference::Device::CPU);
    CHECK(inf.active_runtime() == "cpu", "CPU: runtime switches to cpu");
    {
        const std::vector<Event> evs = make_events(20000, 256, 144, 600000, 50);
        cv::Mat out = inf.crop_to_sensor(inf.infer(evs.data(), evs.size()));
        CHECK(neural_output_sane(out, expect_flow), "CPU: output sane");
    }

    // --- 4) Explicit GPU policy. -------------------------------------------
    inf.set_device(E2VIDInference::Device::GPU);
    const std::string rt_gpu = inf.active_runtime();
    std::printf("  active_runtime (GPU) = \"%s\"\n", rt_gpu.c_str());
    CHECK(rt_gpu == "gpu" || rt_gpu == "cpu",
          "GPU: gpu runtime or graceful cpu degradation");
    {
        const std::vector<Event> evs = make_events(20000, 256, 144, 700000, 50);
        cv::Mat out = inf.crop_to_sensor(inf.infer(evs.data(), evs.size()));
        CHECK(neural_output_sane(out, expect_flow), "GPU policy: output sane");
    }

    // --- 5) reset() restarts the recurrence cleanly. ------------------------
    inf.reset();
    {
        const std::vector<Event> evs = make_events(20000, 256, 144, 800000, 50);
        cv::Mat out = inf.crop_to_sensor(inf.infer(evs.data(), evs.size()));
        CHECK(neural_output_sane(out, expect_flow),
              "reset: output sane after reset");
    }

    // --- 6) Dims change (downsample off) → lazy recompile, still works. ----
    inf.set_downsample(false);
    {
        const std::vector<Event> evs = make_events(60000, 256, 144, 900000, 50);
        cv::Mat out = inf.crop_to_sensor(inf.infer(evs.data(), evs.size()));
        CHECK(neural_output_sane(out, expect_flow),
              "dims change: output sane after recompile");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "Usage: %s <model.onnx> [more models...]\n", argv[0]);
        return 1;
    }
    for (int i = 1; i < argc; ++i) {
        check_model(argv[i]);
    }
    if (g_failures == 0) {
        std::printf("e2vid_inference diagnostic: ALL CHECKS PASSED\n");
        return 0;
    }
    std::printf("e2vid_inference diagnostic: %d CHECK(S) FAILED\n", g_failures);
    return g_failures;
}
