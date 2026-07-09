// Standalone diagnostic: open a .raw file with real_time_playback(true)
// (like the GUI does) and report event delivery + duration.
//
// Build:
//   cmake --build build --target test_raw_playback
// Run:
//   ./build/algo/tests/test_raw_playback test/sparklers.raw
//
// This test was created to diagnose why file playback didn't work in the
// GUI. Key findings:
//   - OSC (OfflineStreamingControl) is NOT ready before start(); becomes
//     ready only after start().
//   - After EOF, seek(0)+start() does NOT restart the camera (SDK terminal
//     state). The only recovery is to reopen the file.
//   - The status change callback fires STARTED then STOPPED on EOF.

#include <chrono>
#include <cstdio>
#include <thread>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/file_config_hints.h>
#include <metavision/sdk/stream/offline_streaming_control.h>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <file.raw>\n", argv[0]);
        return 1;
    }
    const std::string path = argv[1];

    // Test both real_time_playback modes.
    for (int rt = 1; rt >= 0; --rt) {
        std::fprintf(stderr, "\n=== real_time_playback(%s) ===\n", rt ? "true" : "false");
        Metavision::FileConfigHints hints;
        hints.real_time_playback(rt != 0);

        Metavision::Camera cam = Metavision::Camera::from_file(path, hints);
        std::fprintf(stderr, "Sensor: %dx%d\n",
                     cam.geometry().get_width(), cam.geometry().get_height());

        // OSC is not ready before start.
        try {
            auto& osc = cam.offline_streaming_control();
            std::fprintf(stderr, "OSC before start: is_ready=%d\n",
                         static_cast<int>(osc.is_ready()));
        } catch (const std::exception& e) {
            std::fprintf(stderr, "OSC query failed: %s\n", e.what());
        }

        long total = 0;
        long batches = 0;
        Metavision::timestamp last_ts = 0;
        cam.cd().add_callback([&](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            const auto n = e - b;
            if (n > 0) {
                total += n;
                last_ts = (e - 1)->t;
            }
            batches++;
        });

        cam.add_runtime_error_callback([&](const Metavision::CameraException& ex) {
            std::fprintf(stderr, "Runtime error: %s\n", ex.what());
        });

        cam.add_status_change_callback([&](const Metavision::CameraStatus& status) {
            std::fprintf(stderr, "Status change: %s\n",
                         status == Metavision::CameraStatus::STARTED ? "STARTED" : "STOPPED");
        });

        cam.start();

        // OSC is ready after start.
        try {
            auto& osc = cam.offline_streaming_control();
            if (osc.is_ready()) {
                std::fprintf(stderr, "Duration: %lld us\n",
                             static_cast<long long>(osc.get_duration()));
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Duration query failed: %s\n", e.what());
        }

        // Wait for EOF (up to 3 seconds).
        const auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
            if (!cam.is_running()) {
                std::fprintf(stderr, "Camera stopped after %ld ms\n",
                             static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - start).count()));
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (cam.is_running()) cam.stop();

        // Test seek(0) after EOF.
        try {
            auto& osc = cam.offline_streaming_control();
            if (osc.is_ready()) {
                bool ok = osc.seek(0);
                std::fprintf(stderr, "seek(0) after EOF: %s\n", ok ? "OK" : "FAILED");
                // Note: seek(0) succeeds but start() will NOT restart the
                // camera — the SDK enters a terminal state after EOF.
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "seek(0) failed: %s\n", e.what());
        }

        std::fprintf(stderr, "Events: %ld, batches: %ld, last_ts: %lld\n",
                     total, batches, static_cast<long long>(last_ts));
    }

    return 0;
}
