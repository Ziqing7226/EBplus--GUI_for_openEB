// gui/algo_bridge/algo_bridge.h — bridge between the Qt GUI layer and the
// algo/ C++ algorithm modules (self-developed) plus the OpenEB built-in
// algorithms (wrapped).
//
// Design reference: design.md §3.8 and §4.
//
// The bridge真正实例化并调用 algo/cv 与 algo/analytics 的真实算法类。
// AlgoInstance 持有一个 AlgoBackend，push_events 时零拷贝 reinterpret_cast
// EventCD→gui_algo::Event 后调用真实 process()/filter()，pull_result 返回
// 过滤事件 + 叠加层 + 帧。注册表列出全部 29 个自研模块 + 30 个 openEB 能力。

#ifndef GUI_ALGO_BRIDGE_ALGO_BRIDGE_H
#define GUI_ALGO_BRIDGE_ALGO_BRIDGE_H

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "algo_backend.h"  // AlgoBackend, AlgoResult, Overlay* structs

class QImage;  // Qt forward declaration (used in apply_strategy signature).

namespace gui {

// Forward declarations to break the include cycle with gui/display/display_strategy.h
// (which itself only forward-declares AlgoInstance/AlgoInfo/AlgoResult).
class IDisplayStrategy;
struct DisplayContext;

/// How an algorithm's result is presented in the UI (design §5.6.1).
enum class AlgoDisplayMode {
    Overlay,    ///< Drawn on top of the main display frame.
    Replace,    ///< Replaces the main display frame.
    Standalone, ///< Shown in an independent child window.
    Passive,    ///< No visual output (e.g. filters, rate estimators).
};

/// Specification of a single algorithm parameter.
struct AlgoParamSpec {
    std::string key;
    std::string display_name;
    std::string type;                 // "int" | "float" | "bool" | "enum" | "string"
    std::string default_value;
    std::string min_value;
    std::string max_value;
    std::vector<std::string> enum_values;
    /// Mode-applicability filter. Empty = always visible (common param).
    /// Otherwise a comma-separated list of 0-based indices into the algo's
    /// "mode" enum param (e.g. "0" or "1,2"); the UI hides this param when the
    /// currently selected mode index is not in the list. Lets a single algo
    /// expose mode-specific parameter sets (see event_to_video, design §4.4.2).
    std::string mode_filter;
};

/// Static description of an algorithm.
struct AlgoInfo {
    std::string name;            // unique id, e.g. "noise_filter"
    std::string display_name;    // human readable
    std::string category;        // cv | analytics | calibration | openeb_filter | openeb_frame | openeb_preproc | openeb_util
    std::string source;          // "self" | "openeb"
    AlgoDisplayMode display_mode{AlgoDisplayMode::Passive};
    std::vector<AlgoParamSpec> params;
};

/// A live algorithm instance. Holds a real AlgoBackend that wraps an algo/ class.
///
/// Methods are thread-safe: push_events() is called from the SDK data thread
/// while set_param / set_enabled / pull_result are called from the GUI thread.
class AlgoInstance {
public:
    explicit AlgoInstance(const AlgoInfo& info, int width = 1280, int height = 720);

    // Defined out-of-line (in algo_bridge.cpp) so the std::unique_ptr<IDisplayStrategy>
    // member can be destroyed with a complete type — IDisplayStrategy is only
    // forward-declared in this header.
    ~AlgoInstance();

    const AlgoInfo& info() const { return info_; }

    void set_param(const std::string& key, const std::string& value);
    std::string get_param(const std::string& key) const;

    void set_enabled(bool e);
    bool is_enabled() const;

    /// Returns true if the instance was auto-disabled by the flood guard.
    /// The GUI should surface this so the user knows why the algo stopped.
    bool is_overloaded() const;

    /// Clears the overload flag (called when the user re-enables the algo).
    void clear_overload();

    /// @brief Total number of events pushed to this instance (received via
    /// push_events) since it was last enabled. Thread-safe.
    std::size_t total_pushed() const;

    /// @brief Total number of events dropped by the flood guard (capped
    /// batches, auto-disable) since the instance was last enabled.
    /// Thread-safe. Drop rate = total_dropped() / total_pushed().
    std::size_t total_dropped() const;

    /// Push events to the algorithm backend. Thread-safe.
    /// A flood guard caps the batch size and auto-disables the instance if
    /// events arrive far faster than the algo can process them, preventing
    /// memory blowup and GUI freezes under high event rates.
    void push_events(const Metavision::EventCD* begin, const Metavision::EventCD* end);

    /// Pull the latest result (filtered events + overlay + frame).
    AlgoResult pull_result();

    /// Dispatches the already-pulled @p result to this instance's display
    /// strategy (selected at construction from info().display_mode). The
    /// caller fills @p ctx with its display members; apply_strategy() then
    /// sets ctx.instance = this and forwards to the strategy. Replaces the
    /// former switch in MainWindow::process_algo_results() (design §3.5.4).
    void apply_strategy(QImage& frame, AlgoResult& result, DisplayContext& ctx);

    /// Reset the underlying backend.
    void reset();

    /// Updates sensor dimensions on the underlying backend and recomputes
    /// the ROI. Called when a new camera/file connects with different
    /// dimensions than the instance was originally created with.
    void set_sensor_dimensions(int width, int height);

private:
    AlgoInfo info_;
    int width_;
    int height_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::string> param_values_;
    std::unique_ptr<AlgoBackend> backend_;
    // Display strategy selected from info_.display_mode at construction
    // (design §3.5.3). Owned by the instance; apply_strategy() forwards to it.
    std::unique_ptr<IDisplayStrategy> strategy_;
    bool enabled_{false};

    // --- Flood guard (design §5.6.7) -------------------------------------
    // When event rates spike (e.g. 10-100 Mev/s) a slow algorithm cannot keep
    // up; without backpressure its internal buffers grow unbounded and the
    // GUI thread stalls. The guard:
    //   1. caps each batch to the most recent kMaxBatchEvents events,
    //   2. counts consecutive capped batches; if that count exceeds
    //      kFloodStrikes the instance is auto-disabled (overloaded_=true).
    bool overloaded_{false};
    int flood_strikes_{0};
    Metavision::timestamp last_batch_t_{0};
    static constexpr std::size_t kMaxBatchEvents = 50000;
    static constexpr int kFloodStrikes = 4;

    // --- Drop-rate telemetry (design §5.6.7) ----------------------------
    // total_pushed_ = events received via push_events (the denominator).
    // total_dropped_ = events discarded by the flood guard (capped batches,
    // auto-disable, or calls while disabled/overloaded). The ratio
    // total_dropped_/total_pushed_ is surfaced in InformationPanel as the
    // max drop rate across all live instances. Reset in set_enabled(true)
    // so re-enabling gives a fresh session.
    std::size_t total_pushed_{0};
    std::size_t total_dropped_{0};
};

/// @brief Unified algorithm-call bridge (design §3.8).
class AlgoBridge {
public:
    AlgoBridge();

    /// @brief Lists every registered algorithm (OpenEB + self-developed).
    std::vector<AlgoInfo> list_algos() const;

    /// @brief Looks up an algorithm description by name.
    const AlgoInfo* find(const std::string& name) const;

    /// @brief Creates an algorithm instance by name.
    /// @return Shared pointer to the instance, or nullptr if unknown.
    /// @note Always constructs a NEW instance and overwrites any existing live
    /// entry. Prefer find_or_create() to preserve already-set parameters.
    std::shared_ptr<AlgoInstance> create(const std::string& name);

    /// @brief Returns the live instance for @p name if one exists, otherwise
    /// creates one. Use this instead of create() to avoid discarding
    /// parameters that were set before the instance was enabled.
    std::shared_ptr<AlgoInstance> find_or_create(const std::string& name);

    /// @brief Sets the actual sensor dimensions so new instances are created
    /// with the correct width/height instead of the 1280x720 default.
    void set_sensor_dimensions(int width, int height);

    /// @brief Applies a shared preprocessing parameter (preproc_*) to every
    /// live self-developed instance. Preprocessing (noise filter + 1/4
    /// downsample) is stackable and overlays on top of any main algorithm;
    /// it is NOT mutually exclusive. Used by AlgorithmsPanel's preproc
    /// selector so a single control updates all enabled algorithms.
    void apply_global_preproc(const std::string& key, const std::string& value);

    /// @brief Applies shared ROI parameters (roi_*) to every live
    /// self-developed instance and caches them so future instances
    /// created via create() inherit the current ROI settings (N3).
    void apply_global_roi(const std::string& enabled, const std::string& x,
                          const std::string& y, const std::string& w,
                          const std::string& h);

    /// @brief Caches per-algorithm parameters for an instance that is not
    /// yet live. The cached values are replayed in create() so that
    /// parameters loaded from a config file are not lost when the
    /// algorithm is later enabled (N1).
    void cache_algo_params(const std::string& name,
                           const std::map<std::string, std::string>& params);

    /// @brief Looks up a live instance by name. Returns nullptr if no live
    /// instance exists (either never created or already destroyed).
    /// Used by ConfigManager to capture/apply runtime parameter values.
    std::shared_ptr<AlgoInstance> find_live(const std::string& name);

    /// @brief Returns all live instances (for batch event push / result pull).
    std::vector<std::shared_ptr<AlgoInstance>> list_live();

    void push_events(const std::shared_ptr<AlgoInstance>& inst,
                     const Metavision::EventCD* begin,
                     const Metavision::EventCD* end);

    AlgoResult pull_result(const std::shared_ptr<AlgoInstance>& inst);

private:
    void register_openeb_filters();
    void register_openeb_frame_modes();
    void register_openeb_preprocessors();
    void register_openeb_utils();
    void register_self_cv();
    void register_self_analytics();
    void register_self_calibration();

    std::unordered_map<std::string, AlgoInfo> registry_;
    /// Weak references to live instances so ConfigManager can query/apply
    /// runtime parameter values without owning the instances. Expired
    /// entries are pruned lazily on each lookup.
    std::unordered_map<std::string, std::weak_ptr<AlgoInstance>> live_instances_;
    mutable std::mutex live_mutex_;
    int sensor_w_{1280};
    int sensor_h_{720};
    /// Cache of the latest global preproc_* parameter values (BUG-R4).
    /// Replayed in create() so new instances inherit the shared
    /// preprocessing state even when created by other code paths
    /// (ConfigManager, calibration wizard).
    std::unordered_map<std::string, std::string> preproc_cache_;

    /// Cache of the latest global roi_* parameter values (N3). Replayed
    /// in create() alongside preproc_cache_ so new instances inherit the
    /// current ROI settings.
    std::unordered_map<std::string, std::string> roi_cache_;

    /// Per-algorithm parameter cache for instances not yet live (N1).
    /// Populated by ConfigManager::apply_algo_state when an algorithm has
    /// no live instance; replayed in create() so saved values are not lost.
    std::unordered_map<std::string, std::map<std::string, std::string>> algo_param_cache_;
};

} // namespace gui

#endif // GUI_ALGO_BRIDGE_ALGO_BRIDGE_H
