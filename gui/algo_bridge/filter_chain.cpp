// gui/algo_bridge/filter_chain.cpp

#include "filter_chain.h"

#include <mutex>
#include <sstream>

#include <metavision/sdk/core/algorithms/event_rescaler_algorithm.h>
#include <metavision/sdk/core/algorithms/flip_x_algorithm.h>
#include <metavision/sdk/core/algorithms/flip_y_algorithm.h>
#include <metavision/sdk/core/algorithms/polarity_filter_algorithm.h>
#include <metavision/sdk/core/algorithms/polarity_inverter_algorithm.h>
#include <metavision/sdk/core/algorithms/rotate_events_algorithm.h>
#include <metavision/sdk/core/algorithms/transpose_events_algorithm.h>

#include <metavision/sdk/core/algorithms/roi_filter_algorithm.h>

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
private:
    Metavision::FlipYAlgorithm algo_;
};

class RotateStage : public FilterStage {
public:
    RotateStage() : algo_(0, 0, 0.0f) {}
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        if (orthogonal_) {
            // For 90/180/270 we apply the transform directly so the output
            // coordinates are mathematically correct. RotateEventsAlgorithm
            // rotates about ((W)/2, (H)/2) and bounds-checks against the
            // SAME [0,W-1]x[0,H-1] box, which for 90/270 on non-square
            // sensors both drops ~45% of events AND mis-positions the
            // survivors. Direct math drops the events that cannot fit (x>=H
            // or y>=W after the swap) but positions the rest correctly.
            const std::int16_t W = width_minus_one_;
            const std::int16_t H = height_minus_one_;
            for (auto it = b; it != e; ++it) {
                Metavision::EventCD ev = *it;
                switch (angle_) {
                    case 90:  { std::int16_t nx = static_cast<std::int16_t>(H - ev.y);
                                std::int16_t ny = ev.x;
                                if (nx < 0 || ny < 0 || ny > W) continue;
                                ev.x = nx; ev.y = ny; break; }
                    case 180: { ev.x = static_cast<std::int16_t>(W - ev.x);
                                ev.y = static_cast<std::int16_t>(H - ev.y); break; }
                    case 270: { std::int16_t nx = ev.y;
                                std::int16_t ny = static_cast<std::int16_t>(W - ev.x);
                                if (nx < 0 || nx > H || ny < 0 || ny > W) continue;
                                ev.x = nx; ev.y = ny; break; }
                    default: break; // 0° = identity
                }
                out.push_back(ev);
            }
        } else {
            algo_.process_events(b, e, std::back_inserter(out));
        }
    }
    bool set_param(const std::string& k, const std::string& v) override {
        if (k == "rotation") {
            // Accept degrees (0/90/180/270) or a numeric radian. Orthogonal
            // angles take the direct-math fast path; arbitrary radians go
            // through RotateEventsAlgorithm (which clips to the original
            // frame — acceptable for non-integer angles).
            if (v == "0")      { angle_ = 0;   orthogonal_ = true;  return true; }
            if (v == "90")     { angle_ = 90;  orthogonal_ = true;  return true; }
            if (v == "180")    { angle_ = 180; orthogonal_ = true;  return true; }
            if (v == "270")    { angle_ = 270; orthogonal_ = true;  return true; }
            float rad = 0;
            if (!parse(v, rad)) return false;
            rotation_ = rad;
            orthogonal_ = false;
            rebuild();
            return true;
        }
        if (k == "width_minus_one") {
            std::int16_t w = 0;
            if (!parse(v, w)) return false;
            width_minus_one_ = w;
            rebuild();
            return true;
        }
        if (k == "height_minus_one") {
            std::int16_t h = 0;
            if (!parse(v, h)) return false;
            height_minus_one_ = h;
            rebuild();
            return true;
        }
        return false;
    }
    std::string name() const override { return "rotate"; }
private:
    // RotateEventsAlgorithm is only used for arbitrary (non-orthogonal) angles.
    void rebuild() {
        if (orthogonal_) return;
        algo_ = Metavision::RotateEventsAlgorithm(width_minus_one_, height_minus_one_, rotation_);
    }
    int angle_{0};
    bool orthogonal_{true};
    std::int16_t width_minus_one_{0};
    std::int16_t height_minus_one_{0};
    float rotation_{0.0f};
    Metavision::RotateEventsAlgorithm algo_;
};

class TransposeStage : public FilterStage {
public:
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string&, const std::string&) override { return false; }
    std::string name() const override { return "transpose"; }
private:
    Metavision::TransposeEventsAlgorithm algo_;
};

class RescaleStage : public FilterStage {
public:
    RescaleStage() { rebuild(); }
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_ || !algo_) return;
        algo_->process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string& k, const std::string& v) override {
        float f = 0;
        if (!parse(v, f) || f <= 0) return false;
        if (k == "scale_width")  { sw_ = f; rebuild(); return true; }
        if (k == "scale_height") { sh_ = f; rebuild(); return true; }
        return false;
    }
    std::string name() const override { return "rescale"; }
private:
    void rebuild() { algo_ = std::make_unique<Metavision::EventRescalerAlgorithm>(sw_, sh_); }
    float sw_{1.0f}, sh_{1.0f};
    std::unique_ptr<Metavision::EventRescalerAlgorithm> algo_;
};

class RoiFilterStage : public FilterStage {
public:
    RoiFilterStage() : algo_(0, 0, 0, 0) {}
    void process(const Metavision::EventCD* b, const Metavision::EventCD* e,
                 std::vector<Metavision::EventCD>& out) override {
        if (!enabled_) return;
        algo_.process_events(b, e, std::back_inserter(out));
    }
    bool set_param(const std::string& k, const std::string& v) override {
        if (k == "output_relative_coordinates") {
            rel_ = (v == "1" || v == "true" || v == "True" || v == "on" || v == "yes");
            rebuild();
            return true;
        }
        int n = 0;
        if (!parse(v, n)) return false;
        if (k == "x0") { x0_ = n; rebuild(); return true; }
        if (k == "y0") { y0_ = n; rebuild(); return true; }
        if (k == "x1") { x1_ = n; rebuild(); return true; }
        if (k == "y1") { y1_ = n; rebuild(); return true; }
        return false;
    }
    std::string name() const override { return "roi_filter"; }
private:
    void rebuild() { algo_ = Metavision::RoiFilterAlgorithm(x0_, y0_, x1_, y1_, rel_); }
    int x0_{0}, y0_{0}, x1_{0}, y1_{0};
    bool rel_{false};
    Metavision::RoiFilterAlgorithm algo_;
};

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
    add("rotate", std::make_unique<RotateStage>());
    add("transpose", std::make_unique<TransposeStage>());
    add("rescale", std::make_unique<RescaleStage>());
    add("roi_filter", std::make_unique<RoiFilterStage>());
}

void FilterChain::set_geometry(int width, int height) {
    std::lock_guard<std::mutex> lk(chain_mutex());
    width_ = width;
    height_ = height;
    // Rebuild geometry-dependent stages. RotateStage needs both dimensions
    // (it bounds-checks rotated coordinates against width/height-1).
    auto apply = [this](const std::string& name, const std::string& key, const std::string& val) {
        auto it = stages_.find(name);
        if (it != stages_.end()) it->second->set_param(key, val);
    };
    apply("flip_x", "width_minus_one", std::to_string(width - 1));
    apply("flip_y", "height_minus_one", std::to_string(height - 1));
    apply("rotate", "width_minus_one", std::to_string(width - 1));
    apply("rotate", "height_minus_one", std::to_string(height - 1));
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
    std::lock_guard<std::mutex> lk(chain_mutex());
    out.clear();
    // Start from the input; if no stage is enabled, just copy.
    std::vector<Metavision::EventCD> cur(begin, end);
    std::vector<Metavision::EventCD> next;
    next.reserve(cur.size());
    for (const auto& name : order_) {
        auto* s = stages_[name].get();
        if (!s || !s->enabled()) continue;
        next.clear();
        s->process(cur.data(), cur.data() + cur.size(), next);
        cur.swap(next);
    }
    out = std::move(cur);
}

bool FilterChain::has_enabled() const {
    std::lock_guard<std::mutex> lk(chain_mutex());
    for (const auto& kv : stages_) {
        if (kv.second->enabled()) return true;
    }
    return false;
}

} // namespace gui
