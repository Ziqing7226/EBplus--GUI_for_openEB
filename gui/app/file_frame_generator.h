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
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/core/utils/colors.h>

namespace gui {

class FilterChain;  ///< Forward decl — applied at render time for file mode.

class FileFrameGenerator : public QObject {
    Q_OBJECT
public:
    explicit FileFrameGenerator(QObject* parent = nullptr);
    ~FileFrameGenerator();

    /// @brief Thread-safe: buffers events from the SDK streaming thread.
    /// Events MUST be sorted by timestamp (the SDK guarantees this).
    void add_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

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

    /// @brief Sets the FilterChain for event transformation (flip, rotate,
    /// etc.) during file playback. Applied per-frame in render_frame() to
    /// BOTH the display rendering and the events emitted via
    /// events_window_ready, so that flip/rotate/etc. take effect immediately
    /// and algorithm output is also transformed. nullptr = no filtering.
    void set_filter_chain(FilterChain* fc) { filter_chain_ = fc; }

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
    bool has_events() const;
    std::size_t event_count() const;

    /// @brief Clears the event buffer and resets the cursor. Called when
    /// opening a new file or stopping the pipeline.
    void clear();

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
    /// set, the events are already filtered (flip/rotate/etc. applied) so
    /// algorithm output matches the display orientation.
    void events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                             Metavision::timestamp ts);

private:
    void on_timer();
    void render_frame(Metavision::timestamp start_us, Metavision::timestamp end_us);

    // Event buffer — appended from SDK thread, read from GUI thread.
    std::vector<Metavision::EventCD> events_;
    mutable std::mutex mutex_;

    // Geometry
    long width_{0};
    long height_{0};

    // Parameters
    std::uint16_t fps_{30};
    Metavision::timestamp accumulation_us_{33000};
    std::atomic<Metavision::timestamp> duration_us_{0};
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
};

} // namespace gui

#endif // GUI_APP_FILE_FRAME_GENERATOR_H
