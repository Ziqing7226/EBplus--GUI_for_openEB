// Standalone diagnostic: verify FileFrameGenerator produces frames at the
// correct playback rate (fps * window / 1e6).
//
// Build:
//   cmake --build build --target test_file_frame_generator
// Run:
//   ./build/algo/tests/test_file_frame_generator test/sparklers.raw
//
// Expected behavior for sparklers.raw (95871 us duration, 521252 events):
//   fps=30, window=100us  → 958 frames, rate=0.003 (slow motion)
//   fps=30, window=33000us → 2 frames,  rate=0.99 (real-time)
//   fps=60, window=33000us → 2 frames,  rate=1.98 (fast forward)

#include <QCoreApplication>
#include <QTimer>
#include <chrono>
#include <cstdio>
#include <thread>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/stream/offline_streaming_control.h>

#include "app/file_frame_generator.h"

using namespace gui;
using std::chrono::steady_clock;
using std::chrono::milliseconds;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <file.raw>\n", argv[0]);
        return 1;
    }
    QCoreApplication app(argc, argv);
    const std::string path = argv[1];

    // --- Phase 1: Open file and buffer all events ---
    FileFrameGenerator gen;
    int frame_count = 0;
    Metavision::timestamp last_frame_ts = 0;
    QObject::connect(&gen, &FileFrameGenerator::frame_ready,
                     [&](QImage, Metavision::timestamp ts) {
                         frame_count++;
                         last_frame_ts = ts;
                     });

    Metavision::FileConfigHints hints;
    hints.real_time_playback(false);  // read as fast as possible
    Metavision::Camera cam = Metavision::Camera::from_file(path, hints);
    const long w = cam.geometry().get_width();
    const long h = cam.geometry().get_height();
    gen.set_geometry(w, h);

    bool camera_done = false;
    cam.cd().add_callback([&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
        gen.add_events(b, e);
    });
    cam.add_status_change_callback([&](const Metavision::CameraStatus& status) {
        if (status == Metavision::CameraStatus::STOPPED) {
            camera_done = true;
        }
    });

    Metavision::timestamp duration = 0;
    cam.start();
    // Query duration from OSC
    try {
        auto& osc = cam.offline_streaming_control();
        if (osc.is_ready()) duration = osc.get_duration();
    } catch (...) {}
    gen.set_duration_us(duration);

    // Wait for all events to be buffered (camera stops after EOF)
    auto t0 = steady_clock::now();
    while (!camera_done && steady_clock::now() - t0 < milliseconds(3000)) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(milliseconds(5));
    }
    if (cam.is_running()) cam.stop();

    std::fprintf(stderr, "=== File: %s ===\n", path.c_str());
    std::fprintf(stderr, "Geometry: %ldx%ld\n", w, h);
    std::fprintf(stderr, "OSC duration: %lld us\n", (long long)duration);
    std::fprintf(stderr, "Gen duration: %lld us\n", (long long)gen.duration_us());
    std::fprintf(stderr, "Buffered events: %zu\n", gen.event_count());
    std::fprintf(stderr, "Buffering time: %ld ms\n\n",
                 (long)std::chrono::duration_cast<milliseconds>(
                     steady_clock::now() - t0).count());

    // --- Phase 2: Test playback at fps=30, window=100us (slow motion) ---
    {
        frame_count = 0;
        last_frame_ts = 0;
        gen.set_fps(30);
        gen.set_accumulation_time_us(100);
        const double expected_rate = 30.0 * 100.0 / 1e6;
        const int expected_frames = static_cast<int>(gen.duration_us() / 100) + 1;

        std::fprintf(stderr, "=== fps=30, window=100us ===\n");
        std::fprintf(stderr, "Expected rate: %.6f (slow motion)\n", expected_rate);
        std::fprintf(stderr, "Expected frames: %d\n", expected_frames);

        // Play for 200ms of wall-clock time, then check frame count.
        // At 30fps, 200ms should produce ~6 frames.
        gen.play();
        t0 = steady_clock::now();
        while (steady_clock::now() - t0 < milliseconds(200)) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(milliseconds(1));
        }
        gen.pause();
        const long elapsed_ms = std::chrono::duration_cast<milliseconds>(
            steady_clock::now() - t0).count();
        std::fprintf(stderr, "Produced %d frames in %ld ms (expected ~6 at 30fps)\n",
                     frame_count, elapsed_ms);
        std::fprintf(stderr, "Last frame ts: %lld us (cursor advanced %lld us)\n\n",
                     (long long)last_frame_ts, (long long)gen.position_us());
    }

    // --- Phase 3: Test playback at fps=30, window=33000us (real-time) ---
    {
        frame_count = 0;
        last_frame_ts = 0;
        gen.seek(0);
        gen.set_fps(30);
        gen.set_accumulation_time_us(33000);
        const double expected_rate = 30.0 * 33000.0 / 1e6;
        const int expected_frames = static_cast<int>(gen.duration_us() / 33000) + 1;

        std::fprintf(stderr, "=== fps=30, window=33000us ===\n");
        std::fprintf(stderr, "Expected rate: %.6f (real-time)\n", expected_rate);
        std::fprintf(stderr, "Expected frames: %d\n", expected_frames);

        gen.play();
        t0 = steady_clock::now();
        while (steady_clock::now() - t0 < milliseconds(200)) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(milliseconds(1));
        }
        gen.pause();
        const long elapsed_ms = std::chrono::duration_cast<milliseconds>(
            steady_clock::now() - t0).count();
        std::fprintf(stderr, "Produced %d frames in %ld ms\n", frame_count, elapsed_ms);
        std::fprintf(stderr, "Last frame ts: %lld us (cursor: %lld us)\n\n",
                     (long long)last_frame_ts, (long long)gen.position_us());
    }

    // --- Phase 4: Test full playback (run to EOF) ---
    {
        frame_count = 0;
        last_frame_ts = 0;
        gen.seek(0);
        gen.set_fps(60);        // max speed
        gen.set_accumulation_time_us(33000);
        const double expected_rate = 60.0 * 33000.0 / 1e6;
        const int expected_frames = static_cast<int>(gen.duration_us() / 33000) + 1;

        std::fprintf(stderr, "=== Full playback: fps=60, window=33000us ===\n");
        std::fprintf(stderr, "Expected rate: %.6f (fast forward)\n", expected_rate);
        std::fprintf(stderr, "Expected frames: %d\n", expected_frames);

        bool eof = false;
        QObject::connect(&gen, &FileFrameGenerator::eof_reached, [&]() {
            eof = true;
        });

        gen.play();
        t0 = steady_clock::now();
        while (!eof && steady_clock::now() - t0 < milliseconds(5000)) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(milliseconds(1));
        }
        gen.pause();
        const long elapsed_ms = std::chrono::duration_cast<milliseconds>(
            steady_clock::now() - t0).count();
        std::fprintf(stderr, "Produced %d frames in %ld ms (EOF reached: %s)\n",
                     frame_count, elapsed_ms, eof ? "yes" : "no");
        std::fprintf(stderr, "Final cursor: %lld us (duration: %lld us)\n\n",
                     (long long)gen.position_us(), (long long)gen.duration_us());
    }

    // --- Phase 5: Test loop ---
    {
        frame_count = 0;
        gen.seek(0);
        gen.set_fps(60);
        gen.set_accumulation_time_us(33000);
        gen.set_loop(true);

        int eof_count = 0;
        QObject::connect(&gen, &FileFrameGenerator::eof_reached, [&]() {
            eof_count++;
        });

        std::fprintf(stderr, "=== Loop test: fps=60, window=33000us ===\n");
        gen.play();
        t0 = steady_clock::now();
        while (steady_clock::now() - t0 < milliseconds(2000)) {
            QCoreApplication::processEvents();
            std::this_thread::sleep_for(milliseconds(1));
        }
        gen.pause();
        const long elapsed_ms = std::chrono::duration_cast<milliseconds>(
            steady_clock::now() - t0).count();
        std::fprintf(stderr, "Produced %d frames in %ld ms (loop cursor resets: %d)\n",
                     frame_count, elapsed_ms, eof_count);
        std::fprintf(stderr, "Cursor after 2s: %lld us (should be < duration if looping)\n\n",
                     (long long)gen.position_us());
        gen.set_loop(false);
    }

    std::fprintf(stderr, "=== All tests complete ===\n");
    return 0;
}
