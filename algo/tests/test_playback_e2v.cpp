// Diagnostic: simulate the GUI file-playback path for EventToVideo.
// Feeds events in small batches with scaled timestamps (as
// on_events_window_ready does), and reports frame statistics.
//
// Build:
//   cmake --build build --target test_playback_e2v
// Run:
//   ./build/algo/tests/test_playback_e2v ../test/sparklers.raw

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>

#include "algo/analytics/event_to_video.h"
#include "algo/common/event.h"

using gui_algo::EventToVideo;
using gui_algo::Event;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <file.raw>\n", argv[0]);
        return 1;
    }
    const std::string path = argv[1];

    // 1. Collect all events from the raw file (centered 128x128 ROI).
    int sensor_w = 0, sensor_h = 0;
    {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        Metavision::Camera cam = Metavision::Camera::from_file(path, hints);
        sensor_w = cam.geometry().get_width();
        sensor_h = cam.geometry().get_height();
    }
    std::printf("Sensor: %dx%d\n", sensor_w, sensor_h);

    const int roi_x = (sensor_w - 128) / 2;
    const int roi_y = (sensor_h - 128) / 2;

    std::vector<Event> all_events;
    {
        Metavision::FileConfigHints hints;
        hints.real_time_playback(false);
        Metavision::Camera cam = Metavision::Camera::from_file(path, hints);
        cam.cd().add_callback([&](const Metavision::EventCD* b,
                                  const Metavision::EventCD* e) {
            for (const auto* p = b; p < e; ++p) {
                if (p->x < roi_x || p->x >= roi_x + 128 ||
                    p->y < roi_y || p->y >= roi_y + 128) continue;
                all_events.emplace_back(
                    static_cast<uint16_t>(p->x - roi_x),
                    static_cast<uint16_t>(p->y - roi_y),
                    static_cast<uint8_t>(p->p),
                    static_cast<uint64_t>(p->t));
            }
        });
        cam.start();
        while (cam.is_running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    std::printf("Total ROI events: %zu\n", all_events.size());
    if (all_events.empty()) return 0;
    const Metavision::timestamp t_max = all_events.back().t;
    std::printf("Time span: %lld us (%.3f s)\n",
                static_cast<long long>(t_max),
                t_max / 1.0e6);

    // 2. Simulate GUI playback: small batches with timestamp scaling.
    //    playback_rate = fps * accumulation_window_us / 1e6
    //    scaled_t = real_t / playback_rate
    const double fps = 30.0;
    const double window_us = 100.0;       // 100us per frame (slow motion)
    const double playback_rate = fps * window_us / 1.0e6;  // 0.003
    const double inv_rate = 1.0 / playback_rate;
    std::printf("Playback rate: %.6f (inv_rate=%.2f)\n", playback_rate, inv_rate);

    // Test both modes.
    for (int mode_idx = 0; mode_idx <= 1; ++mode_idx) {
        const auto mode = static_cast<EventToVideo::Mode>(mode_idx);
        const char* mode_name =
            (mode_idx == 0) ? "BardowVariational" : "InteractingMaps";

        EventToVideo v(128, 128, mode);
        v.set_downsample(true);
        v.set_output_fps(30);
        v.reset();  // Simulate the reset-on-connect fix.

        std::printf("\n=== %s (reset, scaled timestamps, small batches) ===\n",
                    mode_name);

        int frame_count = 0;
        int black_count = 0;  // frames where min==max==0
        int gray_count = 0;   // frames where min==max==128

        // Feed events in 100us windows (simulating FileFrameGenerator).
        Metavision::timestamp cursor = 0;
        size_t idx = 0;
        while (idx < all_events.size()) {
            const Metavision::timestamp window_end = cursor +
                static_cast<Metavision::timestamp>(window_us);
            // Collect events in [cursor, window_end).
            std::vector<Event> batch;
            while (idx < all_events.size() && all_events[idx].t < window_end) {
                batch.push_back(all_events[idx]);
                // Scale timestamp.
                batch.back().t = static_cast<Metavision::timestamp>(
                    static_cast<double>(all_events[idx].t) * inv_rate);
                ++idx;
            }
            cursor = window_end;

            if (!batch.empty()) {
                v.process(batch.data(), batch.size());
            }

            // Simulate pull_result → get_frame (called every frame).
            cv::Mat frame = v.get_frame();
            if (!frame.empty()) {
                frame_count++;
                double mn = 0, mx = 0;
                cv::minMaxLoc(frame, &mn, &mx);
                if (frame_count <= 15 || frame_count % 200 == 0) {
                    double mean = cv::mean(frame)[0];
                    std::printf("[%s] frame %d: min=%.0f max=%.0f mean=%.1f "
                                "(batch: %zu events)\n",
                                mode_name, frame_count, mn, mx, mean,
                                batch.size());
                }
                if (mx < 1.0) black_count++;
                else if (mx - mn < 1.0) gray_count++;
            }
        }

        std::printf("[%s] Total: %d frames, %d black, %d gray, %d visible "
                    "(%.1f%% black, %.1f%% gray)\n",
                    mode_name, frame_count, black_count, gray_count,
                    frame_count - black_count - gray_count,
                    frame_count > 0 ? 100.0 * black_count / frame_count : 0.0,
                    frame_count > 0 ? 100.0 * gray_count / frame_count : 0.0);
    }

    return 0;
}
