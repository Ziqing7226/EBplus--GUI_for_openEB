// gui/davis/davis_parser.h — wire-format decoder for inivation DAVIS cameras.
//
// Ported (C++17, dependency-free) from dv-processing 2.0.4
// (include/dv-processing/io/camera/parsers/davis_parser.hpp, Apache-2.0) —
// the "classic" inivation DAVIS USB stream is a sequence of 16-bit
// little-endian words:
//   bit 15 set            → timestamp: value = wrapAdd + (word & 0x7FFF), 1 µs ticks
//   code = (word >> 12) & 7 (bit 15 clear):
//     0 → special event (data): 1 = timestamp reset, 2/3/4 = external input,
//         5/7 = IMU start/end, 8..17 = APS/generator markers
//     1 → Y address (data = y)
//     2 → X address, polarity OFF
//     3 → X address, polarity ON
//     4     → APS pixel value (decoded when the APS stream is enabled)
//     5/6   → IMU / misc data (IMU decoded; misc10 ignored)
//     7 → timestamp wrap: wrapAdd += 0x8000 * data
// Events are emitted with timestamps rebased to 0 at the first timestamp
// reset/word, sorted by construction. IMU samples and APS frames are decoded
// via set_imu_sink/set_aps_sink consumers (Phase 2/3). Out-of-range
// coordinates are dropped (the reference asserts; dropping is safer).

#ifndef GUI_DAVIS_DAVIS_PARSER_H
#define GUI_DAVIS_DAVIS_PARSER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

#include "aps_decoder.h"
#include "imu_decoder.h"
#include "imu_types.h"

namespace gui::davis {

class Parser {
public:
    /// Sink invoked with the events decoded from each parsed buffer. The span
    /// is only valid for the duration of the call.
    using EventSink =
        std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;

    /// @param device_width/@param device_height the DEVICE-register DVS
    ///        dimensions (DVS_SIZE_COLUMNS/ROWS, before any orientation
    ///        swap); events outside are dropped. @param invert_xy honors the
    ///        device orientation bit (0x04): the emitted coordinates are then
    ///        swapped, so the caller reports the swapped resolution.
    Parser(int device_width, int device_height, bool invert_xy);

    /// Feeds one USB bulk buffer through the decoder; decoded events are
    /// emitted via @p sink (possibly empty — most specials produce none).
    void parse(const std::uint8_t* data, std::size_t size, const EventSink& sink);

    /// True once a timestamp reset was seen (timestamps are now 0-based).
    [[nodiscard]] bool time_initialized() const { return t0_set_; }

    /// Completed IMU6 samples (accel/gyro/temp) — invoked from the USB
    /// thread whenever the IMU stream is enabled on the device.
    void set_imu_sink(const ImuSink& sink) { imu_.set_sink(sink); }

    /// IMU chip model (from MODULE_IMU / IMU_TYPE) — selects the
    /// temperature formula. Default: Bosch BMI160.
    void set_imu_model(ImuModel model) { imu_.set_model(model); }

    /// Completed APS frames (grayscale) — invoked from the USB thread when
    /// the APS stream is enabled on the device.
    void set_aps_sink(const ApsFrameSink& sink) { aps_.set_sink(sink); }

    /// APS sensor configuration (chip model, APS register dimensions before
    /// orientation swap, MODULE_APS orientation info).
    void set_aps_config(int model, int device_width, int device_height, int orientation) {
        aps_.configure(model, device_width, device_height, orientation);
    }

    /// Full reset (device re-open).
    void reset();

private:
    void update_timestamp(std::int64_t ts);

    int device_width_{0};
    int device_height_{0};
    bool invert_xy_{false};

    bool t0_set_{false};
    std::int64_t wrap_add_{0};
    std::int64_t t0_{0};
    std::int64_t current_{0};
    std::int16_t last_y_{0};

    // DAVIS: X/Y tags carry X first; temperature formula by chip model.
    ImuDecoder imu_{false, false};

    // APS frame stream (DAVIS-only hardware).
    ApsDecoder aps_;

    std::vector<Metavision::EventCD> batch_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DAVIS_PARSER_H
