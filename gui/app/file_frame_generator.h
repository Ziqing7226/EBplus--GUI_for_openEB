// gui/app/file_frame_generator.h — playback-rate-controlled frame generator.
//
// Unlike Metavision::CDFrameGenerator (which shows the LATEST accumulation
// window of a live event stream), FileFrameGenerator buffers ALL events from
// a file and replays them at a user-controlled rate:
//
//   playback_rate = fps * accumulation_time_us / 1e6
//
// For example, fps=30, accumulation=100us → rate=0.003 → a 96ms file plays
// in ~32 seconds (slow motion). fps=60, accumulation=33000us → rate=1.98 →
// fast forward. This is impossible with CDFrameGenerator, which always
// delivers events at 1x speed (real_time_playback=true) or dumps them
// instantly (real_time_playback=false).
//
// Events are buffered via add_events() (called from the SDK streaming thread).
// Frames are produced by a QTimer on the GUI thread at 1/fps interval. Each
// tick renders events in [cursor, cursor+window) to a cv::Mat, emits
// frame_ready, and advances the cursor by window_us.
//
// Loop = cursor reset (no file reopen). Seek = set cursor. Pause = stop timer.
// All O(1) operations that the CDFrameGenerator-based approach could not do.

#ifndef GUI_APP_FILE_FRAME_GENERATOR_H
#define GUI_APP_FILE_FRAME_GENERATOR_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo_bridge/filter_chain.h"
#include "app/stream_conditioner.h"
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/utils/colors.h>

namespace gui {

class FilterChain;  ///< Forward decl — applied at render time for file mode.
class FrameModeRenderer;  ///< Forward decl — non-integration display frame modes.

class FileFrameGenerator : public QObject {
    Q_OBJECT
public:
    explicit FileFrameGenerator(QObject* parent = nullptr);
    ~FileFrameGenerator();

    /// @brief Thread-safe: buffers events from the SDK streaming thread.
    /// Events MUST be sorted by timestamp (the SDK guarantees this).
    /// Stops appending once kMaxBufferedEvents is reached (OOM guard,
    /// audit §六-C2): the buffer is truncated and buffer_truncated() is
    /// emitted once. Playback of the buffered prefix continues normally.
    void add_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

    /// @brief Hard cap on buffered events (OOM guard, audit §六-C2).
    /// Each buffered Metavision::EventCD is 16 bytes, so 300M events is
    /// ~4.8 GB resident — beyond this we drop incoming batches.
    static constexpr std::size_t kMaxBufferedEvents = 300'000'000;

    /// @brief Sets the sensor geometry for frame rendering.
    void set_geometry(long width, long height);

    /// @brief Sets the display frame rate (Hz). Updates the timer interval
    /// immediately if playing.
    void set_fps(std::uint16_t fps);

    /// @brief Sets the per-frame event accumulation window (μs).
    void set_accumulation_time_us(Metavision::timestamp us);

    /// @brief Sets the total file duration (μs), from OSC or last event.
    /// Used to detect EOF. The generator also updates this from incoming
    /// events, so this is primarily for pre-buffering setup.
    void set_duration_us(Metavision::timestamp us);

    /// @brief Sets the color palette for rendering (matches CDFrameGenerator).
    void set_color_palette(Metavision::ColorPalette palette);

    /// @brief Sets the FilterChain for event transformation (flip, polarity,
    /// etc.) during file playback. Applied per-frame in render_frame() to
    /// BOTH the display rendering and the events emitted via
    /// events_window_ready, so that transforms take effect immediately
    /// and algorithm output is also transformed. nullptr = no filtering.
    void set_filter_chain(FilterChain* fc) {
        filter_chain_ = fc;
        conditioner_.set_filter_chain(fc);
    }

    /// @brief Sets the shared frame-mode renderer (non-integration display
    /// modes). render_frame() feeds each window's (filtered) events to it and
    /// uses its generated frame for display. nullptr keeps the palette render.
    void set_frame_mode_renderer(FrameModeRenderer* renderer) {
        frame_mode_renderer_ = renderer;
    }

    // --- Playback control (GUI thread only) ---

    /// @brief Starts frame production. If at EOF, restarts from the beginning.
    void play();
    /// @brief Stops frame production. Cursor position is preserved.
    void pause();
    bool is_playing() const { return playing_; }

    /// @brief Jumps the cursor to @p t_us and renders immediately.
    void seek(Metavision::timestamp t_us);

    /// @brief When true, the cursor resets to 0 on EOF instead of stopping.
    void set_loop(bool on) { loop_ = on; }

    // --- State queries ---

    Metavision::timestamp position_us() const { return cursor_us_; }
    Metavision::timestamp duration_us() const;
    std::uint16_t fps() const { return fps_; }
    Metavision::timestamp accumulation_time_us() const { return accumulation_us_; }
    bool loop() const { return loop_; }
    std::size_t event_count() const;

    /// @brief Clears the event buffer and resets the cursor. Called when
    /// opening a new file or stopping the pipeline. Also resets the
    /// loading_complete_/truncated_ state: a new file is about to stream
    /// in, so EOF handling is suspended until set_loading_complete(true).
    void clear();

    /// @brief Marks whether the file loader has finished streaming the
    /// whole file into the buffer (SDK file-camera EOF). EOF handling in
    /// on_timer() (stop / loop wrap) only engages once this is true;
    /// before that, a cursor that caught up with the read progress waits
    /// instead of triggering a false EOF (audit §六-P2).
    /// Defaults to true so standalone use (no loader attached) behaves as
    /// before; clear() resets it to false for a fresh file load.
    void set_loading_complete(bool complete) {
        loading_complete_.store(complete, std::memory_order_release);
    }
    bool is_loading_complete() const {
        return loading_complete_.load(std::memory_order_acquire);
    }

    /// @brief True once the buffer hit kMaxBufferedEvents and incoming
    /// events were dropped (audit §六-C2). Reset by clear().
    bool is_truncated() const;

signals:
    /// @brief Emitted on each timer tick with the rendered frame.
    /// @p ts is the cursor position at the start of the rendered window.
    void frame_ready(QImage frame, Metavision::timestamp ts);

    /// @brief Emitted after each frame with the new cursor position and
    /// total duration. Drives the playback slider.
    void position_changed(Metavision::timestamp pos_us,
                          Metavision::timestamp dur_us);

    /// @brief Emitted when the cursor reaches the end of the buffer and
    /// loop is off. The timer is stopped before emitting.
    void eof_reached();

    /// @brief Emitted when the cursor wraps back to 0 on loop. Algorithm
    /// instances with temporal state (time_surface current_t_, E2VID
    /// log_intensity_, etc.) must be reset to avoid stale-state freezes
    /// where new events have timestamps < current_t_ and are ignored.
    void looped();

    /// @brief Emitted when seek() moves the cursor. Stateful algorithms
    /// whose internal timestamps are ahead of the new cursor position must
    /// be reset to avoid the same stale-state freeze as looped(). Emitted
    /// BEFORE render_frame() so the reset takes effect before new events
    /// are pushed.
    void seeked(Metavision::timestamp t_us);

    /// @brief Emitted with the (filtered) events in the current accumulation
    /// window [start, end). Used to feed algorithm instances synchronously
    /// with the displayed frame during file playback. When a FilterChain is
    /// set, the events are already filtered (flip/etc. applied) so
    /// algorithm output matches the display orientation.
    void events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                             Metavision::timestamp ts);

    /// @brief Emitted once when the event buffer reaches kMaxBufferedEvents
    /// and further events are dropped (audit §六-C2). Emitted from the SDK
    /// streaming thread (queued to listeners on the GUI thread).
    void buffer_truncated();

private:
    void on_timer();
    void render_frame(Metavision::timestamp start_us, Metavision::timestamp end_us);

    // Event buffer — appended from SDK thread, read from GUI thread.
    std::vector<Metavision::EventCD> events_;
    mutable std::mutex mutex_;
    // OOM guard state (audit §六-C2), guarded by mutex_.
    bool truncated_{false};

    // Geometry
    long width_{0};
    long height_{0};

    // Parameters
    std::uint16_t fps_{30};
    Metavision::timestamp accumulation_us_{33000};
    std::atomic<Metavision::timestamp> duration_us_{0};
    // True once the file loader has streamed the whole file (SDK file EOF).
    // Default true: with no loader attached (standalone tests, pre-filled
    // buffer) EOF handling works as before. clear() resets it to false.
    std::atomic<bool> loading_complete_{true};
    bool loop_{false};
    Metavision::ColorPalette palette_{Metavision::ColorPalette::Dark};

    // Playback state (GUI thread only)
    QTimer timer_;
    Metavision::timestamp cursor_us_{0};
    bool playing_{false};

    // Reused render buffer (BGR)
    cv::Mat frame_;

    // Filter chain (file mode). Applied in render_frame() to both the
    // rendered pixels and the events emitted via events_window_ready, so
    // algorithm output matches the display orientation.
    FilterChain* filter_chain_{nullptr};

    // Frame-mode rendering (non-integration modes). Shared with FramePipeline;
    // render_frame feeds each window's filtered events and uses the generated
    // frame for display when the renderer is active.
    FrameModeRenderer* frame_mode_renderer_{nullptr};

    // Shared conditioning (Phase 2.5 rework 2026-08-19): applied ONCE per
    // rendered window in render_frame(). The SAME output feeds the rendered
    // pixels AND events_window_ready — algorithm instances no longer re-run
    // the noise filter per instance (the old display-only display_preproc_
    // plus per-backend Preprocessors collapsed into this one pass).
    // GUI thread only (render_frame runs there).
    StreamConditioner conditioner_;

    // Software ROI (Phase 2.6). The resolved rect feeds conditioner_ (which
    // crops+shifts); roi_roni_ (Phase 2.6 debug D-5) inverts the predicate:
    // RONI keeps events OUTSIDE the rect with absolute coordinates.
    bool roi_enabled_{false};
    bool roi_roni_{false};
    int roi_x_{-1}, roi_y_{-1}, roi_w_{0}, roi_h_{0};
    int roi_x0_{0}, roi_y0_{0}, roi_x1_{0}, roi_y1_{0};

    // Display placement: origin added back to the conditioned ROI-relative
    // events for RENDERING only (full-sensor frame, pre-rework visuals).
    // Set by set_display_roi / FramePipeline::set_display_roi_origin.
    bool render_origin_active_{false};
    int render_origin_x_{0};
    int render_origin_y_{0};
    std::vector<Metavision::EventCD> render_shift_buf_;
public:
    /// @brief Applies a conditioning parameter (preproc_* keys).
    void set_conditioner_param(const std::string& key, const std::string& value) {
        conditioner_.set_param(key, value);
    }
    /// @brief Clears the conditioner's temporal state (seek/loop).
    void reset_conditioner() { conditioner_.reset_temporal(); }
    /// @brief Display-side ROI origin (rendering only; canonical emitted
    /// stream stays ROI-relative).
    void set_render_origin(bool active, int x, int y) {
        render_origin_active_ = active;
        render_origin_x_ = x;
        render_origin_y_ = y;
    }

    /// @brief Sets the file-mode software ROI (Phase 2.6). Events outside the
    /// rect are dropped from BOTH the rendered frame and events_window_ready
    /// (same "source only outputs ROI events" semantics as the hardware ROI).
    /// @p x/@p y = -1 = auto-center on the sensor; w/h <= 0 = full sensor.
    /// @p roni = true inverts the semantics (keep outside, drop inside).
    void set_display_roi(bool enabled, int x, int y, int w, int h, bool roni = false);
    /// @brief Reads the software ROI state (computed rect [x0,x1) × [y0,y1)).
    void display_roi(bool& enabled, int& x0, int& y0, int& x1, int& y1) const {
        enabled = roi_enabled_;
        x0 = roi_x0_; y0 = roi_y0_; x1 = roi_x1_; y1 = roi_y1_;
    }
    /// @brief True when the software ROI is in RONI (keep-outside) mode.
    bool display_roi_roni() const { return roi_roni_; }
private:
};

} // namespace gui

#endif // GUI_APP_FILE_FRAME_GENERATOR_H
