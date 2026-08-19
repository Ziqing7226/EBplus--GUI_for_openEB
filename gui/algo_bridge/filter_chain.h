// gui/algo_bridge/filter_chain.h — sequential chain of OpenEB event filters
// (design §4.3.1), wrapped behind a uniform interface so the GUI can enable /
// disable / re-parameterize each stage without depending on the concrete
// algorithm headers.
//
// Supported stages: polarity filter, polarity invert, flip X/Y. Each stage is
// identified by the same name used in AlgoBridge::registry_ (e.g.
// "polarity_filter").
//
// rotate / transpose / rescale were removed (2026-08-18): they change event
// coordinates WITHOUT propagating the new frame geometry, so downstream
// W×H buffers (live display time surface, algorithm instances) can be
// written out of bounds → heap corruption / delayed segfault.

#ifndef GUI_ALGO_BRIDGE_FILTER_CHAIN_H
#define GUI_ALGO_BRIDGE_FILTER_CHAIN_H

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

namespace gui {

/// @brief One configurable stage in the preprocessing filter chain.
class FilterStage {
public:
    virtual ~FilterStage() = default;
    /// @brief Process the input range, appending to @p out.
    virtual void process(const Metavision::EventCD* begin,
                         const Metavision::EventCD* end,
                         std::vector<Metavision::EventCD>& out) = 0;
    /// @brief Set a named parameter; returns false if unknown.
    virtual bool set_param(const std::string& key, const std::string& value) = 0;
    virtual bool enabled() const { return enabled_; }
    virtual void set_enabled(bool e) { enabled_ = e; }
    virtual std::string name() const = 0;
    /// @brief True if the stage changes event COORDINATES (mirroring). The
    /// shared stream conditioner runs value-only stages before the noise
    /// filter and geometry stages after undistort (a mirrored frame must be
    /// undistorted with the physical K first), so the two groups are applied
    /// at different pipeline points.
    virtual bool is_geometry() const { return false; }

protected:
    bool enabled_{false};
};

/// @brief Ordered chain of event filters applied left-to-right.
class FilterChain {
public:
    /// Which stage group a process call applies (see FilterStage::is_geometry).
    enum class Group { All, ValueOnly, GeometryOnly };

    FilterChain();

    /// @brief Sets the geometry the flip stages mirror within. Must match the
    /// coordinate system of the events at the flip application point (sensor
    /// dims for a full frame, ROI dims for an ROI-relative frame).
    void set_geometry(int width, int height);

    /// @brief Returns the named stage. The pointer is returned WITHOUT the
    /// chain mutex held; callers must only use it for read-only queries that
    /// do not race with the SDK thread. (The chain mutex is a function-local
    /// static inside filter_chain.cpp and is NOT part of the public API —
    /// external code cannot hold it.)
    /// For GUI-thread mutations prefer set_stage_enabled / set_stage_param,
    /// which take the lock internally.
    FilterStage* stage(const std::string& name);

    /// @brief Thread-safe stage mutators. GUI threads must use these instead
    /// of stage()->set_enabled/set_param to avoid racing the SDK thread's
    /// process() call (which reads the same fields under chain_mutex()).
    void set_stage_enabled(const std::string& name, bool enabled);
    bool set_stage_param(const std::string& name, const std::string& key,
                         const std::string& value);
    bool is_stage_enabled(const std::string& name) const;

    /// @brief Applies all enabled stages in order.
    void process(const Metavision::EventCD* begin,
                 const Metavision::EventCD* end,
                 std::vector<Metavision::EventCD>& out);
    /// @brief Applies only the enabled value-only (non-coordinate) stages.
    void process_value(const Metavision::EventCD* begin,
                       const Metavision::EventCD* end,
                       std::vector<Metavision::EventCD>& out);
    /// @brief Applies only the enabled geometry (coordinate-mirroring) stages.
    void process_geometry(const Metavision::EventCD* begin,
                          const Metavision::EventCD* end,
                          std::vector<Metavision::EventCD>& out);

    /// @brief True if at least one stage is enabled.
    bool has_enabled() const;
    /// @brief True if at least one enabled stage belongs to @p group.
    bool has_enabled(Group group) const;

private:
    void process_group(const Metavision::EventCD* begin,
                       const Metavision::EventCD* end,
                       std::vector<Metavision::EventCD>& out, Group group);

    int width_{0};
    int height_{0};
    std::unordered_map<std::string, std::unique_ptr<FilterStage>> stages_;
    std::vector<std::string> order_;
    /// @brief Reused scratch buffers for process(). All callers serialize on
    /// chain_mutex() (process takes it), so the buffers stay single-threaded
    /// — steady-state zero-allocation even with many enabled stages.
    std::vector<Metavision::EventCD> scratch_in_;
    std::vector<Metavision::EventCD> scratch_out_;
};

} // namespace gui

#endif // GUI_ALGO_BRIDGE_FILTER_CHAIN_H
