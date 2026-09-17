// gui/tests/test_davis_protocol.cpp — unit tests for the ported DAVIS wire
// decoder, bias register encoding and the bias table (Auto Bias name lookup).
// The USB transport itself needs hardware; everything decoded from byte
// streams is verified here against the reference behavior.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include "davis/davis_biases.h"
#include "davis/davis_parser.h"

namespace {

constexpr std::uint16_t ts_word(std::uint16_t value) {
    return static_cast<std::uint16_t>(0x8000 | (value & 0x7FFF));
}
constexpr std::uint16_t y_word(std::uint16_t y) {
    return static_cast<std::uint16_t>(0x1000 | (y & 0x0FFF));
}
constexpr std::uint16_t x_word(std::uint16_t x, bool on) {
    return static_cast<std::uint16_t>((on ? 0x3000 : 0x2000) | (x & 0x0FFF));
}
constexpr std::uint16_t special_word(std::uint16_t data) {
    return static_cast<std::uint16_t>(0x0000 | (data & 0x0FFF));
}
constexpr std::uint16_t wrap_word(std::uint16_t multiplier) {
    return static_cast<std::uint16_t>(0x7000 | (multiplier & 0x0FFF));
}
constexpr std::uint16_t aps_pixel_word(std::uint16_t value) {
    return static_cast<std::uint16_t>(0x4000 | (value & 0x0FFF));
}

void feed(gui::davis::Parser& parser, const std::vector<std::uint16_t>& words,
          std::vector<Metavision::EventCD>& out) {
    std::vector<std::uint8_t> bytes;
    for (const std::uint16_t w : words) {
        bytes.push_back(static_cast<std::uint8_t>(w & 0xFF));
        bytes.push_back(static_cast<std::uint8_t>(w >> 8));
    }
    parser.parse(bytes.data(), bytes.size(),
        [&out](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            out.insert(out.end(), b, e);
        });
}

} // namespace

TEST(DavisParser, DecodesPolarityEventsRebasedToZero) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> events;

    // Timestamp reset special, then ts=500, an event pair, ts=750, another.
    feed(parser, {special_word(1), ts_word(500), y_word(42), x_word(17, true), x_word(18, false),
             ts_word(750), y_word(5), x_word(6, true)},
        events);

    ASSERT_EQ(events.size(), 3u);
    EXPECT_EQ(events[0].t, 0); // rebased to the first timestamp
    EXPECT_EQ(events[0].x, 17);
    EXPECT_EQ(events[0].y, 42);
    EXPECT_EQ(events[0].p, 1);
    EXPECT_EQ(events[1].x, 18);
    EXPECT_EQ(events[1].p, 0);
    EXPECT_EQ(events[1].t, 0); // shares the current timestamp
    EXPECT_EQ(events[2].t, 250);
    EXPECT_EQ(events[2].y, 5);
    EXPECT_TRUE(parser.time_initialized());
}

TEST(DavisParser, DropsEventsBeforeTimestampBase) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> events;
    // Y/X words before any timestamp: no time base yet — dropped.
    feed(parser, {y_word(10), x_word(10, true)}, events);
    EXPECT_TRUE(events.empty());
}

TEST(DavisParser, HandlesTimestampWrap) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> events;
    // Reset, ts near the 15-bit ceiling, wrap by 2×0x8000, next event keeps
    // increasing (monotonic).
    feed(parser, {special_word(1), ts_word(0x7FF0), y_word(1),
             wrap_word(2), ts_word(0x7FF0 + 0x000F), x_word(3, true)},
        events);
    ASSERT_EQ(events.size(), 1u);
    // device ts = (2 * 0x8000) + 0x7FFF; base = 0x7FF0.
    const std::int64_t expected =
        (2LL * 0x8000 + 0x7FFF) - 0x7FF0;
    EXPECT_EQ(events[0].t, expected);
    EXPECT_GT(events[0].t, 0);
}

TEST(DavisParser, TimestampResetRebasesAgain) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> events;
    feed(parser, {special_word(1), ts_word(5000), x_word(1, true)}, events);
    ASSERT_EQ(events.size(), 1u);
    feed(parser, {special_word(1), ts_word(300), x_word(2, true)}, events);
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[1].t, 0); // rebased after the second reset
}

TEST(DavisParser, IgnoresApsAndImuWords) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> events;
    feed(parser, {special_word(1), ts_word(100), aps_pixel_word(0x0321),
             static_cast<std::uint16_t>(0x5000 | 0x0123), static_cast<std::uint16_t>(0x6000 | 0x0234)},
        events);
    EXPECT_TRUE(events.empty());
}

TEST(DavisParser, DropsOutOfRangeCoordinates) {
    gui::davis::Parser parser(8, 8, false);
    std::vector<Metavision::EventCD> events;
    feed(parser, {special_word(1), ts_word(10), y_word(9), x_word(3, true)}, events);
    EXPECT_TRUE(events.empty()); // y=9 outside height 8
    feed(parser, {ts_word(20), y_word(3), x_word(9, true)}, events);
    EXPECT_TRUE(events.empty()); // x=9 outside width 8
}

TEST(DavisParser, InvertXyBoundsUseDeviceCoordinates) {
    // DAVIS346 die: 260 device columns × 346 device rows, orientation bit set
    // (GUI resolution becomes 346×260). Device Y addresses up to 345 are VALID
    // — bounds must be checked against the device dims, not the swapped ones
    // (regression: device-y ≥ 260 was wrongly dropped, losing a third of the
    // sensor width).
    gui::davis::Parser parser(260, 346, true);
    std::vector<Metavision::EventCD> events;
    feed(parser, {special_word(1), ts_word(10), y_word(300), x_word(5, true)}, events);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].x, 300); // device Y → GUI x
    EXPECT_EQ(events[0].y, 5);   // device X → GUI y
    feed(parser, {ts_word(20), y_word(300), x_word(260, true)}, events);
    EXPECT_EQ(events.size(), 1u); // device-x 260 is out of range → dropped
}

TEST(DavisParser, InvertXySwapsCoordinates) {
    gui::davis::Parser parser(260, 346, true); // orientation bit: axes swapped
    std::vector<Metavision::EventCD> events;
    feed(parser, {special_word(1), ts_word(10), y_word(7), x_word(3, true)}, events);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].x, 7); // device Y becomes the GUI x coordinate
    EXPECT_EQ(events[0].y, 3);
}

// ---------------------------------------------------------------------------

TEST(DavisBiases, CoarseFineEncodeDecodeRoundTrip) {
    // diff_on default from the reference: {5, 255, N, normal, normal, on}.
    const std::uint16_t word =
        gui::davis::encode_coarse_fine(true, true, true, true, 255, 5);
    EXPECT_EQ(word, 0x5FFF);

    bool enabled = false, sex_n = false, type_normal = false, current_normal = false;
    std::uint8_t fine = 0, coarse = 0;
    gui::davis::decode_coarse_fine(word, enabled, sex_n, type_normal, current_normal, fine, coarse);
    EXPECT_TRUE(enabled);
    EXPECT_TRUE(sex_n);
    EXPECT_TRUE(type_normal);
    EXPECT_TRUE(current_normal);
    EXPECT_EQ(fine, 255);
    EXPECT_EQ(coarse, 5);
}

TEST(DavisBiases, LinearizedValuesAreMonotonic) {
    EXPECT_EQ(gui::davis::cf_linearize(0, 0), 0);
    EXPECT_EQ(gui::davis::cf_linearize(0, 255), 255);
    EXPECT_EQ(gui::davis::cf_linearize(1, 0), 256);
    EXPECT_EQ(gui::davis::cf_linearize(5, 255), 1535);
    EXPECT_EQ(gui::davis::cf_linearize(7, 255), 2047);
    std::uint8_t coarse = 0, fine = 0;
    gui::davis::cf_delinearize(1535, coarse, fine);
    EXPECT_EQ(coarse, 5);
    EXPECT_EQ(fine, 255);
}

TEST(DavisBiases, VdacEncodeMatchesReference) {
    // dv caerBiasVDACGenerate: word = voltage | current << 6.
    // dv default aps_overflow_level = {voltage 27, current 6} → 0x019B.
    EXPECT_EQ(gui::davis::encode_vdac(27, 6), 27 | (6 << 6));
    EXPECT_EQ(gui::davis::encode_vdac(32, 7), 32 | (7 << 6));

    std::vector<std::pair<std::uint16_t, std::uint16_t>> writes;
    gui::davis::BiasStore store([&](std::uint16_t address, std::uint16_t word) {
        writes.emplace_back(address, word);
    });
    store.apply_defaults();

    // Default aps_overflow_level write: voltage 27, current 6.
    bool found = false;
    for (const auto& [address, word] : writes) {
        if (address == 0) {
            EXPECT_EQ(word, 27 | (6 << 6));
            found = true;
        }
    }
    ASSERT_TRUE(found);

    // Panel set: voltage-only change, current index preserved.
    int v = 0;
    ASSERT_TRUE(store.get_linear("aps_overflow_level", v));
    EXPECT_EQ(v, 27); // panel value = voltage
    writes.clear();
    ASSERT_TRUE(store.set_linear("aps_overflow_level", 40));
    ASSERT_EQ(writes.size(), 1u);
    EXPECT_EQ(writes[0].first, 0);
    EXPECT_EQ(writes[0].second, 40 | (6 << 6));
    EXPECT_TRUE(store.get_linear("aps_overflow_level", v));
    EXPECT_EQ(v, 40);
}

TEST(DavisBiases, StoreDefaultsAndWrites) {
    std::vector<std::pair<std::uint16_t, std::uint16_t>> writes;
    gui::davis::BiasStore store([&](std::uint16_t address, std::uint16_t word) {
        writes.emplace_back(address, word);
    });

    store.apply_defaults();
    // diff_on default word (address 11): flags 0xF + fine 255 << 4 + coarse 5 << 12.
    const bool diff_on_written =
        std::any_of(writes.begin(), writes.end(), [](const auto& w) {
            return w.first == 11 && w.second == 0x5FFF;
        });
    EXPECT_TRUE(diff_on_written);

    // Manual set: linearized value lands as a new register write.
    writes.clear();
    ASSERT_TRUE(store.set_linear("diff_on", 1300)); // coarse 5, fine 20
    ASSERT_EQ(writes.size(), 1u);
    EXPECT_EQ(writes[0].first, 11);
    EXPECT_EQ(writes[0].second,
        gui::davis::encode_coarse_fine(true, true, true, true, 20, 5));

    int value = 0;
    EXPECT_TRUE(store.get_linear("diff_on", value));
    EXPECT_EQ(value, 1300);
    EXPECT_FALSE(store.get_linear("nope", value));
    EXPECT_FALSE(store.set_linear("nope", 1));
}

TEST(DavisBiases, AutoBiasNameLookupIsSatisfiable) {
    // Mirrors BiasApplier::attach: both diff biases must exist with valid,
    // distinct monotonic ranges for Auto Bias to engage.
    gui::davis::BiasStore store([](std::uint16_t, std::uint16_t) {});
    const auto ranges = store.ranges();
    const auto on = ranges.find("diff_on");
    const auto off = ranges.find("diff_off");
    ASSERT_NE(on, ranges.end());
    ASSERT_NE(off, ranges.end());
    EXPECT_EQ(on->second, std::make_pair(0, 2047));
    EXPECT_EQ(off->second, std::make_pair(0, 2047));
}

// ---------------------------------------------------------------------------
// Real-hardware verification (DAVIS346 connected): every parameter write is
// read back from the camera's SPI registers. Gated by EBPLUS_DAVIS_HW=1.
// ---------------------------------------------------------------------------

#if GUI_HAVE_DAVIS
#include <cstdlib>
#include "davis/davis_device.h"

TEST(DavisHardware, BiasRegistersRoundTrip) {
    if (!std::getenv("EBPLUS_DAVIS_HW")) {
        GTEST_SKIP() << "EBPLUS_DAVIS_HW not set (needs a connected DAVIS346/640)";
    }
    auto devices = gui::davis::find_devices();
    ASSERT_FALSE(devices.empty());
    gui::davis::Device dev(devices.front());
    ASSERT_EQ(dev.width(), 346);
    ASSERT_EQ(dev.height(), 260);

    // 1) Every table bias's DEFAULT must be readable back from the camera
    //    (proves apply_defaults() landed register-by-register).
    const auto& table = gui::davis::davis346_bias_table();
    for (const auto& spec : table) {
        const std::uint16_t word = dev.read_bias_register(spec.address);
        if (spec.kind == gui::davis::BiasKind::CoarseFine) {
            bool enabled = false, sex_n = false, type_n = false, cur_n = false;
            std::uint8_t fine = 0, coarse = 0;
            gui::davis::decode_coarse_fine(word, enabled, sex_n, type_n, cur_n, fine, coarse);
            EXPECT_EQ(coarse, spec.coarse) << spec.name;
            EXPECT_EQ(fine, spec.fine) << spec.name;
        } else { // VDAC
            EXPECT_EQ(word & 0x3F, spec.fine) << spec.name;   // voltage
            EXPECT_EQ((word >> 6) & 0x07, spec.coarse) << spec.name; // current
        }
    }

    // 2) Write/readback round trip on diff_on (the Auto Bias knob): extreme
    //    values of the legal linearized range [0, 2047].
    int def = 0;
    ASSERT_TRUE(dev.biases().get_linear("diff_on", def));
    for (int v : {0, 1000, 2047}) {
        ASSERT_TRUE(dev.biases().set_linear("diff_on", v));
        const std::uint16_t word = dev.read_bias_register(11);
        const auto coarse = static_cast<std::uint8_t>((word >> 12) & 0x07);
        const auto fine = static_cast<std::uint8_t>((word >> 4) & 0xFF);
        EXPECT_EQ(gui::davis::cf_linearize(coarse, fine), v) << "diff_on=" << v;
    }

    // 3) VDAC round trip (voltage-only exposure, current index preserved).
    ASSERT_TRUE(dev.biases().set_linear("aps_cascode", 40));
    {
        const std::uint16_t word = dev.read_bias_register(1);
        EXPECT_EQ(word & 0x3F, 40);
        EXPECT_EQ((word >> 6) & 0x07, 6); // reference default current index
    }

    // 4) Restore reference defaults and re-verify.
    dev.biases().apply_defaults();
    for (const auto& spec : table) {
        const std::uint16_t word = dev.read_bias_register(spec.address);
        if (spec.kind == gui::davis::BiasKind::CoarseFine) {
            const auto coarse = static_cast<std::uint8_t>((word >> 12) & 0x07);
            const auto fine = static_cast<std::uint8_t>((word >> 4) & 0xFF);
            EXPECT_EQ(gui::davis::cf_linearize(coarse, fine),
                      gui::davis::cf_linearize(spec.coarse, spec.fine)) << spec.name;
        }
    }
}
#endif

// ---------------------------------------------------------------------------
// BiasApplier homing-target semantics (fake I_LL_Biases — deterministic).
// ---------------------------------------------------------------------------

#if GUI_HAVE_DAVIS
#include "app/bias_applier.h"

class FakeLLBiases final : public Metavision::I_LL_Biases {
public:
    // Base init uses an inline temporary — same trap as DavisLLBiases
    // (a member here would be uninitialized when the base ctor copies it).
    FakeLLBiases() : Metavision::I_LL_Biases(Metavision::DeviceConfig{}) {}

    std::map<std::string, int> get_all_biases() const override {
        return {{"bias_diff_on", state_.at("bias_diff_on")},
                {"bias_diff_off", state_.at("bias_diff_off")}};
    }

    bool get_bias_info_impl(const std::string& name,
                            Metavision::LL_Bias_Info& info) const override {
        if (name != "bias_diff_on" && name != "bias_diff_off") return false;
        info = Metavision::LL_Bias_Info(0, 2047, name, true, "test");
        return true;
    }

protected:
    bool set_impl(const std::string& name, int value) override {
        state_[name] = value;
        return true;
    }
    int get_impl(const std::string& name) const override {
        auto it = state_.find(name);
        return it == state_.end() ? 0 : it->second;
    }

public:
    std::map<std::string, int> state_{{"bias_diff_on", 1700},
                                      {"bias_diff_off", 1700}};
};

TEST(BiasApplier, HomesTowardConfiguredTargets) {
    FakeLLBiases fake;
    gui::BiasApplier applier;
    ASSERT_TRUE(applier.attach(&fake));
    applier.set_home_targets(1535, 1025); // DAVIS reference defaults

    // Repeated homing steps must converge toward the TARGETS, never toward 0.
    for (int i = 0; i < 64 && (fake.state_.at("bias_diff_on") != 1535 ||
                               fake.state_.at("bias_diff_off") != 1025);
         ++i) {
        applier.home(32);
    }
    EXPECT_EQ(fake.state_.at("bias_diff_on"), 1535);
    EXPECT_EQ(fake.state_.at("bias_diff_off"), 1025);
}

TEST(BiasApplier, OffAxisSignFlip) {
    // DAVIS: apply(+delta_off) must DECREASE diff_off (inverted polarity,
    // measured on hardware), while diff_on deltas keep their sign.
    FakeLLBiases fake;
    gui::BiasApplier applier;
    ASSERT_TRUE(applier.attach(&fake));
    applier.set_off_delta_sign(-1);

    EXPECT_EQ(applier.apply(10, -20), gui::BiasApplier::Status::Ok);
    EXPECT_EQ(fake.state_.at("bias_diff_on"), 1710);   // +10: unchanged sign
    EXPECT_EQ(fake.state_.at("bias_diff_off"), 1720);  // -(-20) = +20

    // Without the flip (Prophesee), apply(-20, ...) would have decreased.
    EXPECT_EQ(applier.apply(-5, 0), gui::BiasApplier::Status::Ok);
    EXPECT_EQ(fake.state_.at("bias_diff_on"), 1705);
}

TEST(BiasApplier, DefaultTargetIsZero) {
    // Without explicit targets (Prophesee), homing still walks toward 0.
    FakeLLBiases fake;
    gui::BiasApplier applier;
    ASSERT_TRUE(applier.attach(&fake));
    for (int i = 0; i < 64 && (fake.state_.at("bias_diff_on") != 0 ||
                               fake.state_.at("bias_diff_off") != 0); ++i) {
        applier.home(32);
    }
    EXPECT_EQ(fake.state_.at("bias_diff_on"), 0);
    EXPECT_EQ(fake.state_.at("bias_diff_off"), 0);
}
#endif
