// algo/cv/trigger_synced_filter.h — ✅ 移植自 jAER FilterSyncedEvents (laser line tracker).
//
// Faithful C++ port of ch.unizh.ini.jaer.projects.laser3d.FilterSyncedEvents
// (jAER reference: ref/jaer/src/ch/unizh/ini/jaer/projects/laser3d/FilterSyncedEvents.java).
//
// 跟踪脉冲激光线 (pulsed laser-line tracker)。在外部触发信号对齐的时间窗内,
// 仅保留同时满足以下全部条件的 OFF 事件:
//   1. 同一像素在 [trigger + t0, trigger + t0 + t1) 内曾出现过 ON 事件
//      (t0 = 激光相对触发的延迟 latency, t1 = ON 事件窗口长度 windowlength);
//   2. 当前 OFF 事件时间戳 > trigger + laserPeriod/2
//      (滤除上一激光周期的残影 / 噪声)。
// laserPeriod 由相邻两次触发的时间差自动估计 —— 对应 jAER 中以 special event
// 形式内联到达的触发信号处理。本实现由于 C++ 事件无 special 标志位, 改由
// add_trigger() 适配器注入触发沿, process() 内按时间序推进触发游标并更新
// lastTriggerTimestamp / laserPeriod, 语义与 jAER filterPacket 一致。
//
// Header-only.

#ifndef GUI_ALGO_CV_TRIGGER_SYNCED_FILTER_H
#define GUI_ALGO_CV_TRIGGER_SYNCED_FILTER_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <metavision/sdk/base/utils/timestamp.h>

#include "algo/common/event.h"
#include "algo/common/event_packet.h"

namespace gui_algo {

/// @brief Trigger-synced pulsed laser-line filter (port of jAER FilterSyncedEvents).
///
/// Mirrors jAER's filterPacket: maintains per-pixel last ON/OFF timestamps,
/// tracks the laser period from consecutive triggers, and emits only the OFF
/// events whose preceding ON event landed in the [trigger+t0, trigger+t0+t1)
/// window and whose own timestamp is later than trigger + laserPeriod/2.
class TriggerSyncedFilter {
public:
    static constexpr int kNumChannels = 8;

    /// jAER DEFAULT_TIMESTAMP = 0 (sentinel meaning "no activity recorded").
    static constexpr Metavision::timestamp kDefaultTimestamp = 0;

    /// jAER defaults: t0 = 500 us, t1 = 500 us.
    static constexpr Metavision::timestamp kDefaultT0 = 500;
    static constexpr Metavision::timestamp kDefaultT1 = 500;
    /// jAER getMaxT0() / getMaxT1() = 1000 (GUI spinner bounds; not hard-clamped).
    static constexpr Metavision::timestamp kMaxT = 1000;

    /// @brief Default constructor (sensor dims unknown; per-pixel maps grow
    ///        lazily on first event). Mirrors jAER default-constructed filter.
    TriggerSyncedFilter() : t0_(kDefaultT0), t1_(kDefaultT1) {}

    /// @brief Construct with sensor dimensions, mirroring jAER initFilter()
    ///        which sizes lastOnTimestamps/lastOffTimestamps from
    ///        chip.getSizeX()/getSizeY().
    TriggerSyncedFilter(int width, int height) : t0_(kDefaultT0), t1_(kDefaultT1) {
        init_maps(width, height);
    }

    /// @brief Registers an external trigger edge.
    /// @details jAER receives triggers inline as special events in the packet;
    ///          C++ events carry no special flag, so triggers are supplied via
    ///          this adapter and consumed in time order inside process().
    /// @param t Trigger timestamp (us).
    /// @param channel Trigger channel [0, 7].
    void add_trigger(Metavision::timestamp t, int channel) {
        if (channel < 0 || channel >= kNumChannels) return;
        triggers_[static_cast<std::size_t>(channel)].push_back(t);
    }

    /// @brief Filters @p packet, returning the OFF events that lie on the
    ///        pulsed laser line. Faithful port of filterPacket.
    std::vector<Event> process(const EventPacket& packet) {
        std::vector<Event> kept;
        if (packet.empty()) return kept;

        auto& trigs = triggers_[static_cast<std::size_t>(trigger_channel_)];

        // 前缀截断（§四-待验证1）：triggers_ 列表只增不减、process() 此前
        // 每包从 0 重扫（O(n²)）。先把早于本包窗口的触发按序结算进
        // laserPeriod/lastTriggerTimestamp（与下方 while 循环同一语义），
        // 再从列表移除，使每包只扫新增的触发。
        {
            const Metavision::timestamp win_start = packet[0].t;
            std::size_t consumed = 0;
            while (consumed < trigs.size() && trigs[consumed] < win_start) {
                const Metavision::timestamp tt = trigs[consumed];
                if (last_trigger_ts_ != kDefaultTimestamp) {
                    laser_period_ = tt - last_trigger_ts_;
                    if (laser_period_ < 0) laser_period_ = 0;
                }
                last_trigger_ts_ = tt;
                ++consumed;
            }
            if (consumed > 0) {
                trigs.erase(trigs.begin(),
                            trigs.begin() + static_cast<long>(consumed));
            }
        }

        std::size_t trig_idx = 0;
        for (const Event& e : packet) {
            // Consume every trigger at or before e.t — mirrors jAER handling a
            // special event inline: updates laserPeriod and lastTriggerTimestamp.
            while (trig_idx < trigs.size() && trigs[trig_idx] <= e.t) {
                const Metavision::timestamp tt = trigs[trig_idx];
                if (last_trigger_ts_ != kDefaultTimestamp) {
                    laser_period_ = tt - last_trigger_ts_;
                    if (laser_period_ < 0) {
                        // jAER logs "laserPeriod is smaller 0!" and clamps to 0.
                        laser_period_ = 0;
                    }
                }
                last_trigger_ts_ = tt;
                ++trig_idx;
            }

            update_maps(e);

            // jAER only emits OFF events that satisfy the laser-line gate.
            // 差异：首个触发之前 C++ 全滤（jAER lastTriggerTimestamp 初值 0，
            // 等效 t=0 处有一次隐式触发，流开始处满足窗口的 OFF 事件可通过）。
            if (e.is_off()) {
                const Metavision::timestamp last_on = last_on_at(e.x, e.y);
                const Metavision::timestamp last_off = e.t;
                if (last_trigger_ts_ != kDefaultTimestamp &&
                    last_on >= last_trigger_ts_ + t0_ &&
                    last_on <  last_trigger_ts_ + t0_ + t1_ &&
                    last_off > last_trigger_ts_ + laser_period_ / 2) {
                    kept.push_back(e);
                }
            }
        }
        return kept;
    }

    // Canonical jAER parameters (t0 delay, t1 window) -----------------------
    // jAER setT0/setT1 store the value directly (min/max are GUI hints only).
    Metavision::timestamp t0() const { return t0_; }
    Metavision::timestamp t1() const { return t1_; }
    void set_t0(Metavision::timestamp v) { t0_ = v; }
    void set_t1(Metavision::timestamp v) { t1_ = v; }

    // Laser-sync state (read-only diagnostics) ------------------------------
    Metavision::timestamp last_trigger_ts() const { return last_trigger_ts_; }
    Metavision::timestamp laser_period() const { return laser_period_; }

    // Backward-compatible shims (kept so existing callers compile unchanged).
    // "window_us" maps to t1 (windowlength); trigger_channel selects which
    // registered trigger channel is used as the laser sync signal.
    Metavision::timestamp trigger_window_us() const { return t1_; }
    int trigger_channel() const { return trigger_channel_; }
    void set_trigger_window_us(Metavision::timestamp v) { t1_ = v; }
    void set_trigger_channel(int v) { trigger_channel_ = clamp_channel(v); }

    /// @brief Resets all filter state (triggers, per-pixel maps, laser sync).
    ///        差异：jAER resetFilter 只清两张时间图；本实现全清（§一-1.6）。
    void reset() {
        for (auto& ch : triggers_) ch.clear();
        last_trigger_ts_ = kDefaultTimestamp;
        laser_period_ = 0;
        for (auto& col : last_on_)  std::fill(col.begin(), col.end(), kDefaultTimestamp);
        for (auto& col : last_off_) std::fill(col.begin(), col.end(), kDefaultTimestamp);
    }

private:
    static int clamp_channel(int ch) {
        if (ch < 0) return 0;
        if (ch >= kNumChannels) return kNumChannels - 1;
        return ch;
    }

    /// @brief Allocates the per-pixel maps to [width][height], mirroring jAER
    ///        initFilter(). Indexed [x][y] to match jAER's lastOnTimestamps[x][y].
    void init_maps(int width, int height) {
        if (width <= 0 || height <= 0) return;
        const std::size_t w = static_cast<std::size_t>(width);
        const std::size_t h = static_cast<std::size_t>(height);
        last_on_.assign(w, std::vector<Metavision::timestamp>(h, kDefaultTimestamp));
        last_off_.assign(w, std::vector<Metavision::timestamp>(h, kDefaultTimestamp));
    }

    /// @brief Ensures the per-pixel maps cover pixel (x, y), growing lazily
    ///        when no sensor dimensions were provided at construction.
    void ensure_maps(std::uint16_t x, std::uint16_t y) {
        const std::size_t need_w = static_cast<std::size_t>(x) + 1;
        const std::size_t need_h = static_cast<std::size_t>(y) + 1;
        const std::size_t cur_w = last_on_.size();
        const std::size_t cur_h = cur_w ? last_on_.front().size() : 0;
        if (need_w > cur_w) {
            last_on_.resize(need_w,
                            std::vector<Metavision::timestamp>(cur_h, kDefaultTimestamp));
            last_off_.resize(need_w,
                             std::vector<Metavision::timestamp>(cur_h, kDefaultTimestamp));
        }
        const std::size_t new_h = last_on_.empty() ? 0 : last_on_.front().size();
        if (need_h > new_h) {
            for (auto& col : last_on_)  col.resize(need_h, kDefaultTimestamp);
            for (auto& col : last_off_) col.resize(need_h, kDefaultTimestamp);
        }
    }

    /// @brief Mirrors jAER updateMaps(): records the event timestamp in the
    ///        polarity-appropriate per-pixel map.
    void update_maps(const Event& e) {
        ensure_maps(e.x, e.y);
        if (e.is_on()) {
            last_on_[e.x][e.y] = e.t;
        } else {
            last_off_[e.x][e.y] = e.t;
        }
    }

    /// @brief Mirrors jAER getLastActivity(x, y, Polarity.On).
    Metavision::timestamp last_on_at(std::uint16_t x, std::uint16_t y) const {
        if (last_on_.empty() || x >= last_on_.size() || y >= last_on_[x].size()) {
            return kDefaultTimestamp;
        }
        return last_on_[x][y];
    }

    // Canonical parameters
    Metavision::timestamp t0_{kDefaultT0};  ///< delay: latency of laser to trigger (us)
    Metavision::timestamp t1_{kDefaultT1};  ///< window length for ON events (us)
    int trigger_channel_{0};

    // Trigger sources (multi-channel adapter; jAER reads triggers inline).
    std::array<std::vector<Metavision::timestamp>, kNumChannels> triggers_{};

    // Laser-sync state (mirrors jAER lastTriggerTimestamp / laserPeriod)
    Metavision::timestamp last_trigger_ts_{kDefaultTimestamp};
    Metavision::timestamp laser_period_{0};

    // Per-pixel last-activity maps, indexed [x][y] (mirror jAER 2D arrays).
    std::vector<std::vector<Metavision::timestamp>> last_on_;
    std::vector<std::vector<Metavision::timestamp>> last_off_;
};

} // namespace gui_algo

#endif // GUI_ALGO_CV_TRIGGER_SYNCED_FILTER_H
