// gui/app/frame_mode_renderer.h — per-mode display frame generation for the
// non-integration frame modes (Contrast Map / Histogram / Diff / Time Decay /
// Events Integration), driven by the DisplayPanel's "Frame mode" selector.
//
// Integration mode is handled by the existing paths (CDFrameGenerator live,
// palette render in FileFrameGenerator); this renderer is inactive for it.
//
// Contrast Map, Time Decay and Events Integration use the vendored OpenEB
// algorithms (core module). Histogram and Diff accumulate per-pixel
// positive/negative counts / signed sums directly — the vendored histo/diff
// generators target the EVK3 raw-event-frame pipeline (hardware frame layout),
// not the display path.
//
// add_events() may run on the SDK thread (live mode) while generate() runs on
// the GUI thread — the renderer guards both with an internal mutex.

#ifndef GUI_APP_FRAME_MODE_RENDERER_H
#define GUI_APP_FRAME_MODE_RENDERER_H

#include <memory>
#include <mutex>

#include <opencv2/core.hpp>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>
#include <metavision/sdk/core/utils/colors.h>
#include <metavision/sdk/core/algorithms/contrast_map_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/time_decay_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/events_integration_algorithm.h>

namespace gui {

/// @brief Display frame modes selectable in the DisplayPanel.
enum class FrameMode {
    Integration,      ///< Existing integration-frame path (CDFrameGenerator / palette render).
    ContrastMap,      ///< Per-pixel contrast map (ON − OFF accumulation, tonemapped).
    Histo,            ///< Per-pixel positive/negative event histogram.
    Diff,             ///< Per-pixel signed polarity sum (diff frame).
    TimeDecay,        ///< Exponential time-decay visualization of recent events.
    EventsIntegration ///< Events integrated into a grayscale intensity frame.
};

/// @brief Builds display frames for the non-integration frame modes.
class FrameModeRenderer {
public:
    FrameModeRenderer() = default;

    void set_geometry(int width, int height);
    void set_mode(FrameMode mode);
    FrameMode mode() const { return mode_; }
    /// @brief Characteristic decay time for TimeDecay / EventsIntegration (µs).
    void set_decay_time_us(Metavision::timestamp us);
    /// @brief Color palette for the TimeDecay visualization.
    void set_palette(Metavision::ColorPalette palette);

    /// @brief Clears temporal state (camera restart, file seek/loop, mode change).
    void reset();

    /// @brief Feeds a range of CD events into the active mode's accumulator.
    /// May run on the SDK thread (live mode).
    void add_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

    /// @brief Produces the current-mode frame as CV_8UC3 BGR. Per-window modes
    /// (contrast/histo/diff) reset their accumulation; TimeDecay and
    /// EventsIntegration keep their decaying state until reset(). Returns an
    /// empty Mat when the mode is Integration or nothing has been generated.
    cv::Mat generate();

    bool active() const { return mode_ != FrameMode::Integration; }

private:
    void ensure_instances();
    void reset_locked();
    cv::Mat generate_contrast();
    cv::Mat generate_histo();
    cv::Mat generate_diff();
    cv::Mat generate_time_decay();
    cv::Mat generate_integration();

    std::mutex mutex_;
    FrameMode mode_{FrameMode::Integration};
    int width_{0};
    int height_{0};
    Metavision::timestamp decay_us_{100000};
    Metavision::ColorPalette palette_{Metavision::ColorPalette::Dark};

    // Vendored OpenEB algorithms (created lazily for the active mode).
    std::unique_ptr<Metavision::ContrastMapGenerationAlgorithm> contrast_;
    std::unique_ptr<Metavision::TimeDecayFrameGenerationAlgorithm> time_decay_;
    std::unique_ptr<Metavision::EventsIntegrationAlgorithm> integration_;

    // Direct per-pixel accumulators for histo/diff.
    cv::Mat_<int> histo_pos_;
    cv::Mat_<int> histo_neg_;
    cv::Mat_<int> diff_;
};

} // namespace gui

#endif // GUI_APP_FRAME_MODE_RENDERER_H
