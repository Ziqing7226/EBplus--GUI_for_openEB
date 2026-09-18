// gui/davis/imu_decoder.h — IMU6 sequence decoder shared by the DAVIS and
// DVXplorer wire parsers. Ported from dv-processing 2.0.4 parsers
// (Apache-2.0): the device streams each sample as
//   special data 5        → IMU start (sequence reset)
//   code 5, misc8 code 3  → IMU Scale Config (in-band scales + data-type mask)
//   code 5, misc8 code 0  → data byte (14-byte sequence: 7 × high/low pairs)
//   special data 7        → IMU end (emit only when the sequence is complete)
// The two chips order the X/Y tags differently (the DVXplorer's IMU tags
// carry Y first, DAVIS carries X first), and the temperature formula depends
// on the IMU chip family. Reference flip controls are not exposed by this
// port, so no flip conversion is applied (the reference skips it too when
// flip == flip-control).

#ifndef GUI_DAVIS_IMU_DECODER_H
#define GUI_DAVIS_IMU_DECODER_H

#include <cstdint>

#include "imu_types.h"

namespace gui::davis {

class ImuDecoder {
public:
    /// @param swap_xy true for DVXplorer (its data tags carry Y before X).
    ///        @param bmi160_temp true for the DVXplorer formula
    ///        (raw/512 + 23); false for the DAVIS family (InvenSense
    ///        6500/9250: raw/333.87 + 21, otherwise raw/340 + 35).
    ImuDecoder(bool swap_xy, bool bmi160_temp)
        : swap_xy_(swap_xy), bmi160_temp_(bmi160_temp) {}

    void set_model(ImuModel model) { model_ = model; }
    void set_sink(const ImuSink& sink) { sink_ = sink; }

    /// IMU start (special data 5).
    void start() {
        count_ = 0;
        type_ = 0;
        sample_ = ImuSample{};
    }

    /// IMU Scale Config (code 5, misc8 code 3). @p data is the word's full
    /// 12-bit data field. Bits: [10:8] type mask (1 temp / 2 gyro / 4 accel),
    /// [5:3] accel range (0=±2 g … 3=±16 g), [2:0] gyro range (0=±2000 …
    /// 4=±125 °/s, descending).
    void scale_config(std::uint16_t data) {
        accel_scale_ = 65536.0F / static_cast<float>(4 * (1 << ((data >> 3) & 0x03)));
        // Range codes are 0..4 (descending); clamp corrupted words to 4 —
        // a negative shift would be undefined behavior.
        const auto gyro_range = static_cast<int>(data & 0x07) > 4 ? 4 : (data & 0x07);
        gyro_scale_ = 65536.0F / static_cast<float>(250 * (1 << (4 - gyro_range)));
        type_ = static_cast<std::uint8_t>(data >> 5) & 0x07;
        if (type_ & 0x04) {
            count_ = 0;  // Accelerometer first.
        } else if (type_ & 0x01) {
            count_ = 6;  // Temperature only.
        } else if (type_ & 0x02) {
            count_ = 8;  // Gyroscope only.
        } else {
            count_ = 14;  // Nothing enabled — samples will be discarded.
        }
    }

    /// One data byte (code 5, misc8 code 0): the high half at even sequence
    /// positions, completing the big-endian int16 at odd positions.
    void data_byte(std::uint8_t byte) {
        switch (count_) {
            case 0: case 2: case 4: case 6: case 8: case 10: case 12:
                tmp_ = byte;
                break;
            case 1: accel_tag1() = scaled16(accel_scale_, byte); break;
            case 3: accel_tag3() = scaled16(accel_scale_, byte); break;
            case 5: {
                sample_.accel_z = scaled16(accel_scale_, byte);
                // Sequence continues with temperature only when enabled.
                if ((type_ & 0x01) == 0) count_ += (type_ & 0x02) ? 2 : 8;
                break;
            }
            case 7: {
                const auto raw = static_cast<std::int16_t>((tmp_ << 8) | byte);
                if (bmi160_temp_) {
                    sample_.temperature = (static_cast<float>(raw) / 512.0F) + 23.0F;
                } else if (model_ == ImuModel::InvenSense6500_9250) {
                    sample_.temperature = (static_cast<float>(raw) / 333.87F) + 21.0F;
                } else {
                    sample_.temperature = (static_cast<float>(raw) / 340.0F) + 35.0F;
                }
                // Sequence continues with gyro only when enabled.
                if ((type_ & 0x02) == 0) count_ += 6;
                break;
            }
            case 9: gyro_tag1() = scaled16(gyro_scale_, byte); break;
            case 11: gyro_tag2() = scaled16(gyro_scale_, byte); break;
            case 13: sample_.gyro_z = scaled16(gyro_scale_, byte); break;
            default: break;  // Invalid sequence position — wait for end.
        }
        count_++;
    }

    /// IMU end (special data 7): emit when the sequence is complete.
    /// @param t stream time of the end marker (rebased µs).
    void end(std::int64_t t) {
        if (count_ == 14) {
            sample_.t = t;
            sample_.valid = true;
            if (sink_) sink_(sample_);
        }
    }

    void reset() {
        count_ = 0;
        type_ = 0;
        tmp_ = 0;
        sample_ = ImuSample{};
    }

private:
    float& accel_tag1() { return swap_xy_ ? sample_.accel_y : sample_.accel_x; }
    float& accel_tag3() { return swap_xy_ ? sample_.accel_x : sample_.accel_y; }
    float& gyro_tag1() { return swap_xy_ ? sample_.gyro_y : sample_.gyro_x; }
    float& gyro_tag2() { return swap_xy_ ? sample_.gyro_x : sample_.gyro_y; }

    float scaled16(float scale, std::uint8_t low) const {
        return static_cast<float>(static_cast<std::int16_t>((tmp_ << 8) | low)) / scale;
    }

    bool swap_xy_{false};
    bool bmi160_temp_{false};
    ImuModel model_{ImuModel::BoschBMI160};
    ImuSink sink_;

    std::uint8_t count_{0};
    std::uint8_t type_{0};
    std::uint8_t tmp_{0};
    float accel_scale_{8192.0F};  // ±4 g — replaced by in-band Scale Config.
    float gyro_scale_{65.536F};   // ±500 °/s — replaced by in-band Scale Config.
    ImuSample sample_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_IMU_DECODER_H
