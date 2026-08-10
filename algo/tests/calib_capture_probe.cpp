// algo/tests/calib_capture_probe.cpp — diagnostic for Phase 4 capture rejection.
//
// Replays a .raw file and, at sampled timestamps, renders the last W µs of CD
// events several ways, then runs cv::findCirclesGrid(ASYMMETRIC_GRID) on each —
// reproducing exactly what CalibrationWizard + IntrinsicCalibration do, plus
// alternative strategies (decay accumulation, erode+dilate preprocessing, tuned
// blob detector). Saves PNGs and prints a per-strategy detection tally so we
// can see WHY captures are rejected and WHAT makes detection succeed.
//
// Usage: calib_capture_probe <file.raw> [cols] [rows] [sample_period_us] [max_samples]
//   cols/rows default 6 5 (wizard default grid).

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

using Metavision::EventCD;
using Metavision::timestamp;

namespace {

// Exactly CalibrationWizard::render_event_frame: white bg, any event (ON/OFF) → black.
// Iterates the ring from the back (newest) and stops once t < t0 — the ring
// holds ~60 ms but a capture window is often ≤500 us, so a front-to-back scan
// would waste >99% of iterations on out-of-window events.
cv::Mat render_binary(const std::deque<EventCD>& ring, timestamp t_last,
                      timestamp window_us, int w, int h) {
    cv::Mat frame(h, w, CV_8UC1, cv::Scalar(255));
    const timestamp t0 = t_last - window_us;
    for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
        if (it->t < t0) break;
        if (it->x >= w || it->y >= h) continue;
        frame.ptr<uchar>(it->y)[it->x] = 0;
    }
    return frame;
}

// Decay accumulation (approximation of the main display's time-surface): each
// event darkens its pixel by `step`, clamped to [0,255]. Over the window, edges
// that fire repeatedly become dark — a denser, more image-like frame than the
// flat binary.
cv::Mat render_decay(const std::deque<EventCD>& ring, timestamp t_last,
                     timestamp window_us, int w, int h, int step) {
    cv::Mat frame(h, w, CV_8UC1, cv::Scalar(255));
    const timestamp t0 = t_last - window_us;
    for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
        if (it->t < t0) break;
        if (it->x >= w || it->y >= h) continue;
        uchar& v = frame.ptr<uchar>(it->y)[it->x];
        v = static_cast<uchar>(std::max(0, int(v) - step));
    }
    return frame;
}

// Erode then dilate (user-suggested preprocessing): erode with a small kernel
// removes isolated noise pixels; dilate with a larger kernel expands the
// remaining event clusters, potentially merging waffle edges into filled blobs.
// erode_k=0 skips erode; dilate_k=0 skips dilate.
cv::Mat erode_dilate(const cv::Mat& gray, int erode_k, int dilate_k) {
    cv::Mat out = gray.clone();
    if (erode_k > 0) {
        const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(erode_k, erode_k));
        cv::erode(out, out, k);
    }
    if (dilate_k > 0) {
        const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(dilate_k, dilate_k));
        cv::dilate(out, out, k);
    }
    return out;
}

cv::Ptr<cv::FeatureDetector> default_detector() {
    // Same as findCirclesGrid's default SimpleBlobDetector.
    return cv::SimpleBlobDetector::create();
}

cv::Ptr<cv::FeatureDetector> tuned_detector() {
    cv::SimpleBlobDetector::Params p;
    p.thresholdStep = 10;
    p.minThreshold = 10;
    p.maxThreshold = 220;
    p.minRepeatability = 1;
    p.filterByColor = true;
    p.blobColor = 0;  // dark blobs
    p.filterByArea = true;
    p.minArea = 5;
    p.maxArea = 500000;  // allow large blobs on high-res sensors
    p.filterByCircularity = false;
    p.filterByInertia = false;  // sparse/edge blobs aren't inertia-clean
    p.filterByConvexity = false;  // event-painted circles aren't perfectly convex
    return cv::SimpleBlobDetector::create(p);
}

bool detect(const cv::Mat& gray, cv::Size board, int flags,
            const cv::Ptr<cv::FeatureDetector>& det,
            std::vector<cv::Point2f>& corners) {
    return cv::findCirclesGrid(gray, board, corners, flags, det);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.raw> [cols] [rows] [sample_period_us] [max_samples]\n",
                     argv[0]);
        return 2;
    }
    const std::string path = argv[1];
    const int cols = (argc > 2) ? std::max(1, std::atoi(argv[2])) : 6;
    const int rows = (argc > 3) ? std::max(1, std::atoi(argv[3])) : 5;
    const timestamp sample_period = (argc > 4) ? std::atoll(argv[4]) : 300000;
    const int max_samples = (argc > 5) ? std::atoi(argv[5]) : 20;
    const cv::Size board(cols, rows);

    std::fprintf(stderr, "[probe] %s\n[probe] grid=%dx%d asymmetric, sample every %lld us, max %d samples\n",
                 path.c_str(), cols, rows,
                 static_cast<long long>(sample_period), max_samples);

    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);
    Metavision::Camera cam;
    try {
        cam = Metavision::Camera::from_file(path, hints);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[probe] open failed: %s\n", e.what());
        return 1;
    }
    const int W = cam.geometry().get_width();
    const int H = cam.geometry().get_height();
    std::fprintf(stderr, "[probe] sensor %dx%d\n", W, H);

    constexpr timestamp kKeepUs = 60000;  // keep last 60 ms in the ring
    std::deque<EventCD> ring;
    timestamp last_t = 0;

    // Strategy table: (window_us, decay?, erode_k, dilate_k, tuned?, tag).
    // erode_k/dilate_k: 0=skip. When both >0, erode (remove noise) then dilate
    // (merge waffle cells into filled blobs) — user-suggested preprocessing.
    struct Strat {
        timestamp window_us;
        bool decay;
        int erode_k;
        int dilate_k;
        bool tuned;
        const char* tag;
    };
    const std::vector<Strat> strats = {
        // 5000µs = actual wizard capture window (binary = what the wizard does).
        {5000,  false, 0, 0, false, "bin_5ms_def"},
        {5000,  false, 0, 0, true,  "bin_5ms_tuned"},
        // 5000µs + erode(denoise) + dilate(merge waffle cells), various kernel sizes.
        {5000,  false, 3, 5,  true,  "bin_5ms_e3d5_tuned"},
        {5000,  false, 3, 7,  true,  "bin_5ms_e3d7_tuned"},
        {5000,  false, 3, 9,  true,  "bin_5ms_e3d9_tuned"},
        {5000,  false, 3, 11, true,  "bin_5ms_e3d11_tuned"},
        {5000,  false, 3, 15, true,  "bin_5ms_e3d15_tuned"},
        // 5000µs decay + erode/dilate.
        {5000,  true,  0, 0, true,  "decay5ms_tuned"},
        {5000,  true,  3, 9, true,  "decay5ms_e3d9_tuned"},
        // 50ms for comparison (denser events).
        {50000, false, 0, 0, true,  "bin_50ms_tuned"},
        {50000, false, 3, 9, true,  "bin_50ms_e3d9_tuned"},
    };
    std::vector<int> hits(strats.size(), 0);
    int total_samples = 0;

    // First sample fires after one sample-period of event time has elapsed
    // (e.g. 50 ms → analyze only the first 50 ms of the recording, not the
    // whole 796 MB file).
    timestamp next_sample = sample_period;
    int sample_idx = 0;
    bool done = false;
    const std::string outdir = "/tmp/ring_probe";

    cam.cd().add_callback([&](const EventCD* b, const EventCD* e) {
        for (const EventCD* p = b; p != e; ++p) {
            ring.push_back(*p);
            last_t = p->t;
        }
        while (!ring.empty() && last_t - ring.front().t > kKeepUs) ring.pop_front();

        if (done || sample_idx >= max_samples || last_t < next_sample) return;
        next_sample = last_t + sample_period;
        const int s = sample_idx++;
        total_samples++;
        std::fprintf(stderr, "[probe] sample %d @ t=%lld us (ring=%zu)\n", s,
                     static_cast<long long>(last_t), ring.size());

        for (std::size_t i = 0; i < strats.size(); ++i) {
            const Strat& st = strats[i];
            cv::Mat gray = st.decay
                ? render_decay(ring, last_t, st.window_us, W, H, 40)
                : render_binary(ring, last_t, st.window_us, W, H);
            if (st.erode_k > 0 || st.dilate_k > 0) {
                gray = erode_dilate(gray, st.erode_k, st.dilate_k);
            }

            // Quick structural stats: dark-pixel ratio + connected-component
            // count (tells us whether the frame has filled blobs vs sparse dots).
            cv::Mat bw = gray < 128;
            int dark = cv::countNonZero(bw);
            cv::Mat labels, stats_cc, centroids;
            int ncc = cv::connectedComponentsWithStats(bw, labels, stats_cc, centroids, 8);
            int big_blobs = 0;  // components with area >= 20 px (potential circles)
            for (int k = 1; k < ncc; ++k) {
                if (stats_cc.at<int>(k, cv::CC_STAT_AREA) >= 20) ++big_blobs;
            }

            // Downscale for detection — findCirclesGrid's 17-threshold sweep is
            // the bottleneck; ~7x fewer pixels at 480 wide keeps it fast while
            // preserving circle geometry.
            cv::Mat det_gray = gray;
            if (gray.cols > 480) {
                cv::resize(gray, det_gray, cv::Size(), 480.0 / gray.cols,
                           480.0 / gray.cols, cv::INTER_AREA);
            }
            std::vector<cv::Point2f> corners;
            const auto det = st.tuned ? tuned_detector() : default_detector();
            const bool found = detect(det_gray, board, cv::CALIB_CB_ASYMMETRIC_GRID, det, corners);
            if (found) hits[i]++;

            std::fprintf(stderr, "    %-24s dark=%5.1f%%  CC=%4d(big>=%4d)  %s\n",
                         st.tag, 100.0 * dark / (gray.cols * gray.rows),
                         ncc - 1, big_blobs, found ? "DETECTED" : "no");

            // Save PNG (downscale to 1024 wide for inspection).
            cv::Mat vis = gray.clone();
            if (vis.cols > 1024) {
                cv::resize(vis, vis, cv::Size(), 1024.0 / vis.cols, 1024.0 / vis.cols,
                           cv::INTER_NEAREST);
            }
            cv::Mat bgr;
            cv::cvtColor(vis, bgr, cv::COLOR_GRAY2BGR);
            if (found) {
                const double sx = static_cast<double>(vis.cols) / det_gray.cols;
                for (auto& c : corners) { c.x *= float(sx); c.y *= float(sx); }
                cv::drawChessboardCorners(bgr, board, corners, true);
            }
            char fn[256];
            std::snprintf(fn, sizeof(fn), "%s/s%02d_%s_%s.png",
                          outdir.c_str(), s, st.tag, found ? "OK" : "no");
            cv::imwrite(fn, bgr);
        }
        if (sample_idx >= max_samples) done = true;
    });

    cam.start();
    while (cam.is_running()) {
        if (done) { cam.stop(); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (cam.is_running()) cam.stop();

    std::fprintf(stderr, "\n[probe] === summary: %d samples, grid %dx%d asymmetric ===\n",
                 total_samples, cols, rows);
    for (std::size_t i = 0; i < strats.size(); ++i) {
        std::fprintf(stderr, "[probe] %-22s  %3d / %3d detected\n",
                     strats[i].tag, hits[i], total_samples);
    }
    std::fprintf(stderr, "[probe] PNGs in %s/ (tagged _OK / _no)\n", outdir.c_str());
    return 0;
}
