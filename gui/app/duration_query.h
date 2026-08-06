// gui/app/duration_query.h — non-blocking source-duration query helper.
//
// Metavision's OfflineStreamingControl::get_duration() can block for
// seconds-to-minutes on raw files (it builds the .tmp_index synchronously).
// Calling it on a worker thread freezes progress reporting (and cancel)
// until the whole operation is done; calling it on the GUI thread freezes
// the UI. query_duration_async() runs the query on a detached thread and
// publishes the result into a shared atomic when it arrives.

#ifndef GUI_APP_DURATION_QUERY_H
#define GUI_APP_DURATION_QUERY_H

#include <atomic>
#include <memory>
#include <thread>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/stream/offline_streaming_control.h>

namespace gui {

/// Queries the OSC duration on a detached thread. The Metavision::Camera
/// handle is ref-counted and the duration target is shared_ptr-held, so
/// both outlive the caller's scope safely.
inline void query_duration_async(
    std::shared_ptr<Metavision::Camera> cam,
    std::shared_ptr<std::atomic<Metavision::timestamp>> dur_us) {
    std::thread([cam = std::move(cam), dur_us = std::move(dur_us)]() mutable {
        try {
            auto& osc = cam->offline_streaming_control();
            for (int i = 0; i < 200; ++i) {  // wait for readiness, ≤10 s
                if (osc.is_ready()) {
                    const auto d = osc.get_duration();
                    if (d > 0) dur_us->store(d, std::memory_order_relaxed);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        } catch (...) {}
    }).detach();
}

} // namespace gui

#endif // GUI_APP_DURATION_QUERY_H
