// gui/tests/test_davis_protocol.cpp — unit tests for the ported DAVIS wire
// decoder, bias register encoding and the bias table (Auto Bias name lookup).
// The USB transport itself needs hardware; everything decoded from byte
// streams is verified here against the reference behavior.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "davis/davis_biases.h"
#include "davis/davis_parser.h"
#include "davis/dvxplorer_parser.h"
#include "davis/imu_types.h"

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


// ---------------------------------------------------------------------------
// DVXplorer wire decode (mgroup compression, separate polarity bit).
// The parser is compiled unconditionally, so these run without libusb.
// ---------------------------------------------------------------------------

namespace {

constexpr std::uint16_t dvx_x_word(std::uint16_t x) {
    return static_cast<std::uint16_t>(0x1000 | (x & 0x03FF));
}
// code 4: group1 address (6 bits), group2 offset (5 bits), bit 11 = minus.
constexpr std::uint16_t dvx_y_group_word(std::uint16_t g1, std::uint16_t offset, bool minus) {
    return static_cast<std::uint16_t>(0x4000 | (g1 & 0x003F) | ((offset & 0x001F) << 6) |
                                      (minus ? 0x0800 : 0x0000));
}
// code 2/3 word: bits 7..0 = 8-pixel presence mask, bit 8 = polarity
// (clear = ON/positive, set = OFF/negative).
constexpr std::uint16_t dvx_pixel_word(std::uint16_t code, std::uint16_t mask_pol) {
    return static_cast<std::uint16_t>(((code & 0x7) << 12) | (mask_pol & 0x01FF));
}

void feed(gui::davis::DvxParser& parser, const std::vector<std::uint16_t>& words,
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

TEST(DvxParser, DecodesGroupEventsRebasedToZero) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // TS reset → the first timestamp (2000) becomes the stream base. X latch
    // at column 40; Y-group latch: group1 = base group 5 (rows 40..47),
    // group2 = +2 groups (rows 56..63). Pixel words then reference the two
    // latched groups: code 3 → group 1 ON (bit 8 clear, mask 0x07),
    // code 2 → group 2 OFF (bit 8 set, mask 0x60).
    feed(parser, {special_word(1), ts_word(2000), dvx_x_word(40),
                  dvx_y_group_word(5, 2, false), ts_word(2500),
                  dvx_pixel_word(3, 0x0007), dvx_pixel_word(2, 0x0160)}, events);

    ASSERT_EQ(events.size(), 5u);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(events[i].x, 40);
        EXPECT_EQ(events[i].y, 40 + i);   // group 1 base row 40
        EXPECT_EQ(events[i].p, 1);        // bit 8 clear = ON
        EXPECT_EQ(events[i].t, 500);      // current(2500) − t0(2000)
    }
    // group 2 base row 56, mask bits 5 and 6, bit 8 set = OFF.
    EXPECT_EQ(events[3].x, 40);
    EXPECT_EQ(events[3].y, 61);
    EXPECT_EQ(events[3].p, 0);
    EXPECT_EQ(events[3].t, 500);
    EXPECT_EQ(events[4].y, 62);
    EXPECT_EQ(events[4].p, 0);
}

TEST(DvxParser, YGroupMinusOffset) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // Bit 11 set: group2 = group1 − offset (groups 5 and 3 → rows 40 and 24).
    feed(parser, {special_word(1), ts_word(100), dvx_x_word(7),
                  dvx_y_group_word(5, 2, true), dvx_pixel_word(3, 0x0001),
                  dvx_pixel_word(2, 0x0001)}, events);

    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].y, 40);  // group 1
    EXPECT_EQ(events[0].p, 1);
    EXPECT_EQ(events[1].y, 24);  // group 2 = 40 − 16
    EXPECT_EQ(events[1].p, 1);
}

TEST(DvxParser, XResetMarker) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // X address 1023 = startup-reset marker → lastX latches to 0.
    feed(parser, {special_word(1), ts_word(500), dvx_x_word(1023), ts_word(600),
                  dvx_pixel_word(2, 0x0007)}, events);

    ASSERT_EQ(events.size(), 3u);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(events[i].x, 0);
        EXPECT_EQ(events[i].y, 0 + i);  // lastYG2 still 0 after the reset
    }
}

TEST(DvxParser, TsWrapAndReset) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // Base at 0x7FF0, then a wrap word with multiplier 2 (wrapAdd += 2·2^15),
    // then a timestamp 16 into the new wrap period.
    feed(parser, {special_word(1), ts_word(0x7FF0), wrap_word(2)}, events);
    EXPECT_TRUE(events.empty());
    feed(parser, {ts_word(16), dvx_pixel_word(3, 0x0002)}, events);

    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].t, 2LL * 0x8000 + 16 - 0x7FF0);
}

TEST(DvxParser, DropsOutOfBoundsPixels) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // Group 62 starts at row 496 ≥ 480 → every pixel of the group is dropped;
    // group 59 (rows 472..479) fits and survives in full.
    feed(parser, {special_word(1), ts_word(100), dvx_x_word(0),
                  dvx_y_group_word(62, 0, false), dvx_pixel_word(3, 0x00FF)}, events);
    EXPECT_TRUE(events.empty());

    feed(parser, {dvx_y_group_word(59, 0, false), dvx_pixel_word(3, 0x00FF)}, events);
    ASSERT_EQ(events.size(), 8u);
    EXPECT_EQ(events[0].y, 472);
    EXPECT_EQ(events[7].y, 479);
}

TEST(DvxParser, IgnoresImuAndMiscWords) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> events;

    // Codes 5/6 carry IMU/misc data — consumed and ignored (events-only).
    feed(parser, {special_word(1), ts_word(100),
                  static_cast<std::uint16_t>(0x5000 | 0x0123),
                  static_cast<std::uint16_t>(0x6000 | 0x0234)}, events);
    EXPECT_TRUE(events.empty());
}


// ---------------------------------------------------------------------------
// IMU6 sequence decode (shared ImuDecoder, per-parser tag order + temp).
// ---------------------------------------------------------------------------

namespace {

// Code-5 IMU data word: low 4 bits of the data field = code 0, low byte data.
constexpr std::uint16_t imu_data_word(std::uint8_t byte) {
    return static_cast<std::uint16_t>(0x5000 | byte);
}
// Code-5 IMU scale-config word: code 3 in bits 11..8; type [7:5],
// accel range [4:3], gyro range [2:0].
constexpr std::uint16_t imu_scale_word(std::uint8_t type, std::uint8_t accel_range,
                                       std::uint8_t gyro_range) {
    return static_cast<std::uint16_t>(0x5000 | (3 << 8) | (type << 5) | (accel_range << 3) |
                                      gyro_range);
}

// Feeds one complete 14-byte sample and the IMU end marker; returns the
// decoded sample (default if discarded).
template <typename ParserT>
gui::davis::ImuSample feed_imu_sample(ParserT& parser, std::uint16_t scale_word,
                                      const std::array<std::uint8_t, 14>& bytes,
                                      std::uint16_t end_ts) {
    gui::davis::ImuSample got;
    parser.set_imu_sink([&got](const gui::davis::ImuSample& s) { got = s; });

    std::vector<std::uint16_t> words{special_word(5), scale_word};
    for (const std::uint8_t b : bytes) words.push_back(imu_data_word(b));
    words.push_back(ts_word(end_ts));
    words.push_back(special_word(7));

    std::vector<Metavision::EventCD> dropped;  // event sink placeholder
    feed(parser, words, dropped);
    return got;
}

} // namespace

TEST(DvxImu, DecodesFullSampleWithSwappedTagsAndBmi160Temp) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> dropped;
    feed(parser, {special_word(1), ts_word(1000)}, dropped);  // rebase to 0

    // type = temp|gyro|accel (7), accel ±4 g (1), gyro ±500 °/s (2).
    const std::uint16_t scale = imu_scale_word(7, 1, 2);
    std::array<std::uint8_t, 14> bytes = {
        0x01, 0x00,  // tag1 = accelY  raw +256 → 256/8192 g
        0xFF, 0xFF,  // tag3 = accelX  raw −1
        0x08, 0x00,  // accelZ         raw +2048 → 0.25 g
        0x01, 0x90,  // temperature    raw 400 → 400/512 + 23
        0x02, 0x00,  // tag9  = gyroY  raw +512 → 512/65.536 °/s
        0x00, 0x64,  // tag11 = gyroX  raw +100
        0xFF, 0x9C,  // gyroZ          raw −100
    };
    const auto s = feed_imu_sample(parser, scale, bytes, 1600);

    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.t, 600);  // end marker (1600) − base (1000)
    EXPECT_FLOAT_EQ(s.accel_y, 256.0F / 8192.0F);
    EXPECT_FLOAT_EQ(s.accel_x, -1.0F / 8192.0F);
    EXPECT_FLOAT_EQ(s.accel_z, 0.25F);
    EXPECT_FLOAT_EQ(s.temperature, 400.0F / 512.0F + 23.0F);
    EXPECT_FLOAT_EQ(s.gyro_y, 512.0F / 65.536F);
    EXPECT_FLOAT_EQ(s.gyro_x, 100.0F / 65.536F);
    EXPECT_FLOAT_EQ(s.gyro_z, -100.0F / 65.536F);
}

TEST(DavisImu, DecodesFullSampleWithStraightTagsAndDavistemp) {
    gui::davis::Parser parser(346, 260, false);
    std::vector<Metavision::EventCD> dropped;
    feed(parser, {special_word(1), ts_word(500)}, dropped);

    const std::uint16_t scale = imu_scale_word(7, 3, 0);  // ±16 g, ±2000 °/s
    std::array<std::uint8_t, 14> bytes = {
        0x04, 0x00,  // tag1 = accelX raw +1024 → 1024/2048 g
        0x00, 0x01,  // tag3 = accelY raw +1
        0x00, 0x00,  // accelZ raw 0
        0x0A, 0x28,  // temperature raw 2600 → 2600/340 + 35 (BMI160 model)
        0x10, 0x00,  // tag9  = gyroX raw +4096 → 4096/16.384 °/s
        0x00, 0x00,  // tag11 = gyroY
        0x00, 0x00,  // gyroZ
    };
    const auto s = feed_imu_sample(parser, scale, bytes, 900);

    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.t, 400);
    EXPECT_FLOAT_EQ(s.accel_x, 1024.0F / 2048.0F);
    EXPECT_FLOAT_EQ(s.accel_y, 1.0F / 2048.0F);
    EXPECT_FLOAT_EQ(s.accel_z, 0.0F);
    EXPECT_FLOAT_EQ(s.temperature, 2600.0F / 340.0F + 35.0F);
    EXPECT_FLOAT_EQ(s.gyro_x, 4096.0F / 16.384F);
    EXPECT_FLOAT_EQ(s.gyro_y, 0.0F);
    EXPECT_FLOAT_EQ(s.gyro_z, 0.0F);
}

TEST(DavisImu, InvenSenseTemperatureFormula) {
    gui::davis::Parser parser(346, 260, false);
    parser.set_imu_model(gui::davis::ImuModel::InvenSense6500_9250);
    std::vector<Metavision::EventCD> dropped;
    feed(parser, {special_word(1), ts_word(100)}, dropped);

    std::array<std::uint8_t, 14> bytes{};
    bytes[6] = 0x13; bytes[7] = 0x88;  // temperature raw 5000
    const auto s = feed_imu_sample(parser, imu_scale_word(7, 1, 2), bytes, 200);
    EXPECT_FLOAT_EQ(s.temperature, 5000.0F / 333.87F + 21.0F);
}

TEST(DvxImu, AccelOnlySequenceCountJump) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> dropped;
    feed(parser, {special_word(1), ts_word(100)}, dropped);

    // type = accel only (4): the device streams just the 6 accel bytes; the
    // decoder's count jump (+8 after accelZ, no temp/gyro enabled) must land
    // the sequence at the complete marker (14).
    gui::davis::ImuSample got;
    parser.set_imu_sink([&got](const gui::davis::ImuSample& s) { got = s; });

    std::vector<std::uint16_t> words{special_word(5), imu_scale_word(4, 1, 2),
                                     imu_data_word(0x02), imu_data_word(0x00),
                                     imu_data_word(0x00), imu_data_word(0x01),
                                     imu_data_word(0x00), imu_data_word(0x00),
                                     ts_word(300), special_word(7)};
    feed(parser, words, dropped);

    EXPECT_TRUE(got.valid);
    EXPECT_FLOAT_EQ(got.accel_y, 512.0F / 8192.0F);  // tag1 = accelY on DVX
    EXPECT_FLOAT_EQ(got.temperature, 0.0F);          // never received
    EXPECT_FLOAT_EQ(got.gyro_z, 0.0F);
}

TEST(DvxImu, IncompleteSequenceDiscarded) {
    gui::davis::DvxParser parser(640, 480);
    std::vector<Metavision::EventCD> dropped;
    feed(parser, {special_word(1), ts_word(100)}, dropped);

    gui::davis::ImuSample got;
    bool called = false;
    parser.set_imu_sink([&got, &called](const gui::davis::ImuSample& s) {
        got = s;
        called = true;
    });

    std::vector<std::uint16_t> words{special_word(5), imu_scale_word(7, 1, 2)};
    for (int i = 0; i < 13; ++i) words.push_back(imu_data_word(0x11));  // one short
    words.push_back(special_word(7));
    feed(parser, words, dropped);
    EXPECT_FALSE(called);
}
