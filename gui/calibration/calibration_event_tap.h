// gui/calibration/calibration_event_tap.h — rolling CD-event buffer for the
// Phase 4 calibration wizard.
//
// Subscribes to CameraController::cd_events_ready and keeps recent batches. The
// wizard calls drain_last_window() when the user presses Space to grab the most
// recent window_us of events (polarity-agnostic capture) and render a full-
// resolution binary frame for cv::findCirclesGrid.
//
// The connection uses Qt::DirectConnection so on_events_ready() runs on the
// SDK streaming thread (where the signal is emitted), NOT the GUI thread.
// A queued connection would post every batch to the GUI thread's event queue,
// and when findCirclesGrid blocks the worker thread the queue grows without
// bound — each batch is a heap-allocated shared_ptr<vector<EventCD>> — and the
// process is OOM-killed. With a direct connection, batches go straight into
// the ring (under mutex_) and never enter the event queue. on_events_ready() is
// thread-safe (mutex only, no GUI) and the destructor disconnects + drains to
// prevent use-after-free.
//
// Performance design (Phase 4 perf pass, referencing FocusAssistant):
// The camera controller already deep-copies each batch into a
// shared_ptr<vector<EventCD>> before emitting. The tap used to copy every
// event AGAIN into a flat vector — a second per-event copy on the SDK thread
// that doubled the memory-allocation overhead of cd_broadcast and starved the
// main display pipeline at high event rates. The tap now stores the shared_ptr
// directly (refcount increment only — no per-event copy) in a deque of batches.
// drain_last_window() walks the batches and collects the in-window events on
// the GUI thread, which runs only once per Space press. This makes the SDK-
// thread cost of cd_broadcast O(1) per batch instead of O(N) per batch.
//
// Phase 4 bug-absorption: attach() disconnects any prior connection before
// connecting, so repeated set_camera() calls do not stack duplicate
// connections (the main caee2c0 tap connected unconditionally on every call).

#ifndef GUI_CALIBRATION_CALIBRATION_EVENT_TAP_H
#define GUI_CALIBRATION_CALIBRATION_EVENT_TAP_H

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include <QObject>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

class CameraController;

class CalibrationEventTap : public QObject {
    Q_OBJECT
public:
    explicit CalibrationEventTap(QObject* parent = nullptr);
    ~CalibrationEventTap();

    /// @brief Connects to @p camera's cd_events_ready signal. Disconnects any
    /// existing connection first, so calling this repeatedly (e.g. each time
    /// the wizard opens) never stacks duplicate connections. Safe with nullptr
    /// (drain becomes a no-op).
    void attach(CameraController* camera);

    /// @brief Drains the buffered batches and copies the most recent
    ///        window_us-wide slice (the events with the largest timestamps,
    ///        i.e. [t_last - window_us, t_last]) into @p out. All drained
    ///        batches are dropped (older events outside the window are stale).
    ///        Runs on the GUI thread in response to the Space key.
    std::size_t drain_last_window(Metavision::timestamp window_us,
                                  std::vector<Metavision::EventCD>& out);

    /// @brief Drops all buffered batches (e.g. when the user starts a fresh
    /// capture session or resets the wizard).
    void clear();

private slots:
    void on_events_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events);

private:
    using BatchPtr = std::shared_ptr<std::vector<Metavision::EventCD>>;

    std::mutex mutex_;
    /// Ring of recent batches (shared_ptr — no per-event copy on the SDK
    /// thread). Batches are in chronological order by arrival.
    std::deque<BatchPtr> batches_;
    CameraController* camera_{nullptr};

    /// Keep batches whose last event is within this many microseconds of the
    /// most recent event. Must be >= the capture window (user-tunable 100–1000 µs
    /// in the wizard, default 500 µs) so drain_last_window always finds a full
    /// window. 1100 µs gives a 100 µs margin beyond the 1000 µs maximum window.
    static constexpr Metavision::timestamp kKeepWindowUs = 1100;

    /// Safety cap on the number of batches (pathological case: tiny batches at
    /// an extreme event rate). 256 batches × typical 1–10 K events/batch =
    /// 256 K–2.5 M events kept — well within memory budget and always spans
    /// > kKeepWindowUs at realistic batch rates.
    static constexpr std::size_t kMaxBatches = 256;
};

} // namespace gui

#endif // GUI_CALIBRATION_CALIBRATION_EVENT_TAP_H
