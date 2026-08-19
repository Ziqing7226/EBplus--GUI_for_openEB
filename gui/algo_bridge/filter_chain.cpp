// gui/algo_bridge/filter_chain.cpp

#include "filter_chain.h"

#include <mutex>
#include <sstream>

#include <metavision/sdk/core/algorithms/flip_x_algorithm.h>
#include <metavision/sdk/core/algorithms/flip_y_algorithm.h>
#include <metavision/sdk/core/algorithms/polarity_filter_algorithm.h>
#include <metavision/sdk/core/algorithms/polarity_inverter_algorithm.h>

namespace gui {

// FilterChain is mutated from the GUI thread (set_enabled / set_param /
// set_geometry) and read from the SDK data thread (process / has_enabled).
// A mutex serialises the two; the per-stage algorithms themselves are not
// otherwise thread-safe.
namespace {
std::mutex& chain_mutex() {
    static std::mutex m;
    return m;
}
} // namespace

namespace {

// Helper to parse a typed value from a string.
template<class T>
bool parse(const std::string& s, T& out) {
    std::istringstream iss(s);
    iss >> out;
    return !iss.fail();
}
template<>
bool parse<bool>(const std::string& s, bool& out) {
    out = (s == "1" || s == "true" || s == "True");
    return true;
}
template<>
bool parse<std::string>(const std::string& s, std::string& out) {
    out = s;
    return true;
}

// --- Concrete stages ---

class PolarityFilterStage : public FilterStage {
public:
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string& k, const std::string& v) override {
        if (k == "polarity") {
            std::int16_t p = 0;
            if (!parse(v, p)) return false;
            algo_.set_polarity(p);
            return true;
        }
        return false;
    }
    std::string name() const override { return "polarity_filter"; }
private:
    Metavision::PolarityFilterAlgorithm algo_{0};
};

class PolarityInvertStage : public FilterStage {
public:
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string&, const std::string&) override { return false; }
    std::string name() const override { return "polarity_invert"; }
private:
    Metavision::PolarityInverterAlgorithm algo_;
};

class FlipXStage : public FilterStage {
public:
    explicit FlipXStage(int w) : algo_(static_cast<std::int16_t>(w - 1)) {}
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string& k, const std::string& v) override {
        if (k == "width_minus_one") {
            std::int16_t w = 0;
            if (!parse(v, w)) return false;
            algo_.set_width_minus_one(w);
            return true;
        }
        return false;
    }
    std::string name() const override { return "flip_x"; }
    bool is_geometry() const override { return true; }
private:
    Metavision::FlipXAlgorithm algo_;
};

class FlipYStage : public FilterStage {
public:
    explicit FlipYStage(int h) : algo_(static_cast<std::int16_t>(h - 1)) {}
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string& k, const std::string& v) override {
        if (k == "height_minus_one") {
            std::int16_t hgt = 0;
            if (!parse(v, hgt)) return false;
            algo_.set_height_minus_one(hgt);
            return true;
        }
        return false;
    }
    std::string name() const override { return "flip_y"; }
    bool is_geometry() const override { return true; }
private:
    Metavision::FlipYAlgorithm algo_;
};

// Phase 2.6 debug D-6: RoiFilterStage was deleted (superseded by the
// unified ROI — see the FilterChain ctor).
//
// RotateStage / TransposeStage / RescaleStage were deleted (2026-08-18):
// they change event coordinates without propagating the new frame geometry
// to the W×H downstream buffers (live display time surface, algorithm
// instances), causing out-of-bounds writes → heap corruption / delayed
// segfaults (munmap_chunk / corrupted size / double free / unaligned
// tcache, depending on which heap metadata got clobbered). Old configs
// referencing them hit the unknown-stage warning path, like roi_filter.

} // namespace

FilterChain::FilterChain() {
    auto add = [&](const std::string& n, std::unique_ptr<FilterStage> s) {
        order_.push_back(n);
        stages_[n] = std::move(s);
    };
    add("polarity_filter", std::make_unique<PolarityFilterStage>());
    add("polarity_invert", std::make_unique<PolarityInvertStage>());
    add("flip_x", std::make_unique<FlipXStage>(width_));
    add("flip_y", std::make_unique<FlipYStage>(height_));
    // Phase 2.6 debug D-6: the roi_filter stage was deleted (superseded by
    // the unified ROI). 2026-08-18: rotate / transpose / rescale were deleted
    // (coordinate-changing stages without geometry propagation — see the
    // note above). Old configs referencing them hit the unknown-stage
    // warning path.
}

void FilterChain::set_geometry(int width, int height) {
    std::lock_guard<std::mutex> lk(chain_mutex());
    width_ = width;
    height_ = height;
    // Rebuild geometry-dependent stages (the flips mirror coordinates within
    // [0,W-1]x[0,H-1], so they must track the sensor dims).
    auto apply = [this](const std::string& name, const std::string& key, const std::string& val) {
        auto it = stages_.find(name);
        if (it != stages_.end()) it->second->set_param(key, val);
    };
    apply("flip_x", "width_minus_one", std::to_string(width - 1));
    apply("flip_y", "height_minus_one", std::to_string(height - 1));
}

FilterStage* FilterChain::stage(const std::string& name) {
    auto it = stages_.find(name);
    return it == stages_.end() ? nullptr : it->second.get();
}

void FilterChain::set_stage_enabled(const std::string& name, bool enabled) {
    std::lock_guard<std::mutex> lk(chain_mutex());
    auto it = stages_.find(name);
    if (it != stages_.end()) it->second->set_enabled(enabled);
}

bool FilterChain::set_stage_param(const std::string& name, const std::string& key,
                                  const std::string& value) {
    std::lock_guard<std::mutex> lk(chain_mutex());
    auto it = stages_.find(name);
    if (it == stages_.end()) return false;
    return it->second->set_param(key, value);
}

bool FilterChain::is_stage_enabled(const std::string& name) const {
    std::lock_guard<std::mutex> lk(chain_mutex());
    auto it = stages_.find(name);
    return it != stages_.end() && it->second->enabled();
}

void FilterChain::process(const Metavision::EventCD* begin,
                          const Metavision::EventCD* end,
                          std::vector<Metavision::EventCD>& out) {
    process_group(begin, end, out, Group::All);
}

void FilterChain::process_value(const Metavision::EventCD* begin,
                                const Metavision::EventCD* end,
                                std::vector<Metavision::EventCD>& out) {
    process_group(begin, end, out, Group::ValueOnly);
}

void FilterChain::process_geometry(const Metavision::EventCD* begin,
                                   const Metavision::EventCD* end,
                                   std::vector<Metavision::EventCD>& out) {
    process_group(begin, end, out, Group::GeometryOnly);
}

void FilterChain::process_group(const Metavision::EventCD* begin,
                                const Metavision::EventCD* end,
                                std::vector<Metavision::EventCD>& out,
                                Group group) {
    std::lock_guard<std::mutex> lk(chain_mutex());
    out.clear();
    // Reuse the scratch buffers (safe under chain_mutex_) instead of
    // allocating cur/next per call and per stage — steady-state zero
    // allocation on the hot SDK-thread path.
    scratch_in_.clear();
    scratch_in_.insert(scratch_in_.end(), begin, end);
    scratch_out_.clear();
    scratch_out_.reserve(std::max(scratch_out_.capacity(), scratch_in_.size()));
    for (const auto& name : order_) {
        auto* s = stages_[name].get();
        if (!s || !s->enabled()) continue;
        if (group == Group::ValueOnly && s->is_geometry()) continue;
        if (group == Group::GeometryOnly && !s->is_geometry()) continue;
        scratch_out_.clear();
        s->process(scratch_in_.data(), scratch_in_.data() + scratch_in_.size(),
                   scratch_out_);
        scratch_in_.swap(scratch_out_);
    }
    out.assign(scratch_in_.begin(), scratch_in_.end());
}

bool FilterChain::has_enabled() const { return has_enabled(Group::All); }

bool FilterChain::has_enabled(Group group) const {
    std::lock_guard<std::mutex> lk(chain_mutex());
    for (const auto& kv : stages_) {
        if (!kv.second->enabled()) continue;
        if (group == Group::All) return true;
        if (group == Group::ValueOnly && !kv.second->is_geometry()) return true;
        if (group == Group::GeometryOnly && kv.second->is_geometry()) return true;
    }
    return false;
}

} // namespace gui
