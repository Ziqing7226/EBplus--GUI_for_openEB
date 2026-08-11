// algo/tests/calib_capture_probe.cpp — diagnostic for Zhou's Screw-Head Grid
// calibration capture.
//
// Replays a .raw file and, at sampled timestamps, renders the last W µs of CD
// events as a three-valued colour frame (black bg / gold=ON / white=OFF, blend
// where both polarities fired) — exactly what CalibrationWizard::render_event_frame
// produces — then runs detect_screwheads on it. Saves annotated PNGs and prints
// a per-window detection tally so we can see which capture windows succeed and
// what the screw-head detector finds in real event data.
//
// Usage: calib_capture_probe <file.raw> [window_us] [sample_period_us] [max_samples]
//   window_us   default 5000 (wizard default capture window)
//   dot_gap     fixed 2      (wizard default; 1/2/3 supported)
//   sample_period_us  default 300000
//   max_samples default 20

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include "algo/calibration/screwhead_detect.h"

using Metavision::EventCD;
using Metavision::timestamp;
using gui_algo::detect_screwheads;

namespace {

// Three-valued colour constants — must match CalibrationWizard::render_event_frame.
const cv::Vec3b kGold(0, 215, 255);    // ON polarity
const cv::Vec3b kWhite(255, 255, 255); // OFF polarity
const cv::Vec3b kBlend = (kGold + kWhite) * 0.5;

// Renders the last window_us of events as a three-valued BGR frame (black bg,
// gold=ON, white=OFF, blend where both fired). Identical to
// CalibrationWizard::render_event_frame. Iterates the ring from the back
// (newest) and stops once t < t0 — the ring holds ~60 ms but a capture window
// is ≤20000 µs, so a front-to-back scan would waste most iterations.
cv::Mat render_three_valued(const std::deque<EventCD>& ring, timestamp t_last,
                            timestamp window_us, int w, int h) {
    cv::Mat frame(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat has_on(h, w, CV_8U, cv::Scalar(0));
    cv::Mat has_off(h, w, CV_8U, cv::Scalar(0));
    const timestamp t0 = t_last - window_us;
    for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
        if (it->t < t0) break;
        if (it->x >= w || it->y >= h) continue;
        if (it->p != 0) has_on.at<uchar>(it->y, it->x) = 1;
        else            has_off.at<uchar>(it->y, it->x) = 1;
    }
    for (int y = 0; y < h; ++y) {
        const uchar* on = has_on.ptr<uchar>(y);
        const uchar* off = has_off.ptr<uchar>(y);
        cv::Vec3b* f = frame.ptr<cv::Vec3b>(y);
        for (int x = 0; x < w; ++x) {
            if (on[x] && off[x])      f[x] = kBlend;
            else if (on[x])           f[x] = kGold;
            else if (off[x])          f[x] = kWhite;
        }
    }
    return frame;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <file.raw> [window_us] "
                     "[sample_period_us] [max_samples]\n", argv[0]);
        return 2;
    }
    const std::string path = argv[1];
    const timestamp window_us = (argc > 2) ? std::max(timestamp(200), std::atoll(argv[2])) : 5000;
    constexpr int dot_gap = 2;  // wizard default; 1/2/3 supported
    const timestamp sample_period = (argc > 3) ? std::atoll(argv[3]) : 300000;
    const int max_samples = (argc > 4) ? std::atoi(argv[4]) : 20;
    constexpr int kCols = 6, kRows = 5;

    std::fprintf(stderr, "[probe] %s\n[probe] screw-head %dx%d, window=%lld us, "
                 "dot_gap=%d, sample every %lld us, max %d samples\n",
                 path.c_str(), kCols, kRows,
                 static_cast<long long>(window_us), dot_gap,
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

    int hits = 0;
    int total_samples = 0;

    timestamp next_sample = sample_period;
    int sample_idx = 0;
    bool done = false;
    const std::string outdir = "/tmp/screwhead_probe";

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

        // Count events in the capture window for diagnostics.
        const timestamp t0 = last_t - window_us;
        std::size_t n_ev = 0;
        for (auto it = ring.rbegin(); it != ring.rend(); ++it) {
            if (it->t < t0) break;
            ++n_ev;
        }

        cv::Mat frame = render_three_valued(ring, last_t, window_us, W, H);
        auto res = detect_screwheads(frame, kCols, kRows, dot_gap, true);
        if (res.found) hits++;

        std::fprintf(stderr, "[probe] sample %d @ t=%lld us  events=%zu  %s\n",
                     s, static_cast<long long>(last_t), n_ev,
                     res.found ? "DETECTED" : "no");

        // Save annotated PNG (downscale to 1024 wide for inspection).
        cv::Mat vis = res.image.empty() ? frame : res.image;
        if (vis.cols > 1024) {
            cv::resize(vis, vis, cv::Size(), 1024.0 / vis.cols, 1024.0 / vis.cols,
                       cv::INTER_NEAREST);
        }
        char fn[256];
        std::snprintf(fn, sizeof(fn), "%s/s%02d_%s.png",
                      outdir.c_str(), s, res.found ? "OK" : "no");
        cv::imwrite(fn, vis);

        if (sample_idx >= max_samples) done = true;
    });

    cam.start();
    while (cam.is_running()) {
        if (done) { cam.stop(); break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (cam.is_running()) cam.stop();

    std::fprintf(stderr, "\n[probe] === summary: %d samples, screw-head %dx%d, "
                 "window=%lld us, dot_gap=%d ===\n",
                 total_samples, kCols, kRows,
                 static_cast<long long>(window_us), dot_gap);
    std::fprintf(stderr, "[probe] detected %d / %d\n", hits, total_samples);
    std::fprintf(stderr, "[probe] PNGs in %s/ (tagged _OK / _no)\n", outdir.c_str());
    return 0;
}
