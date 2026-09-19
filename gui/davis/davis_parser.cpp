// gui/davis/davis_parser.cpp — see davis_parser.h for the wire format.
// Ported from dv-processing 2.0.4 parsers/davis_parser.hpp (Apache-2.0).

#include "davis_parser.h"

#include <cstring>
#include <stdexcept>

namespace gui::davis {

Parser::Parser(int device_width, int device_height, bool invert_xy)
    : device_width_(device_width), device_height_(device_height), invert_xy_(invert_xy) {}

void Parser::reset() {
    t0_set_ = false;
    wrap_add_ = 0;
    t0_ = 0;
    current_ = 0;
    last_y_ = 0;
    imu_.reset();
    aps_.reset();
    batch_.clear();
}

void Parser::update_timestamp(std::int64_t ts) {
    if (!t0_set_) {
        // First timestamp after a reset: rebase the stream to start at 0.
        t0_set_ = true;
        t0_ = ts;
    }
    current_ = ts;
}

void Parser::parse(const std::uint8_t* data, std::size_t size, const EventSink& sink) {
    batch_.clear();
    // 16-bit little-endian words; a trailing odd byte cannot happen on a
    // bulk stream from this device family, skip defensively if it does.
    const std::size_t words = size / 2;
    for (std::size_t i = 0; i < words; ++i) {
        std::uint16_t event = 0;
        std::memcpy(&event, data + i * 2, 2);

        if ((event & 0x8000) != 0) {
            // Timestamp word: 15-bit µs value extended by the wrap counter.
            update_timestamp(wrap_add_ + (event & 0x7FFF));
            continue;
        }

        const auto code = static_cast<std::uint8_t>((event & 0x7000) >> 12);
        const auto data_part = static_cast<std::uint16_t>(event & 0x0FFF);

        switch (code) {
            case 0: // Special events (data codes: 1 = TS reset, 2..4 =
                    // external input, 5/7 = IMU start/end, 8..17 = APS /
                    // generator markers — the non-IMU ones stay consumed
                    // and ignored in the events-only stream).
                if (data_part == 1) {
                    // Timestamp reset: the FPGA restarts its 15-bit counter.
                    wrap_add_ = 0;
                    t0_set_ = false;
                    current_ = 0;
                } else if (data_part == 5) {
                    imu_.start();  // IMU start (6 axes).
                } else if (data_part == 7) {
                    imu_.end(t0_set_ ? (current_ - t0_) : current_);
                } else if (data_part == 8) {
                    aps_.frame_start(true);  // APS global-shutter frame start.
                } else if (data_part == 9) {
                    aps_.frame_start(false);  // APS rolling-shutter frame start.
                } else if (data_part == 10) {
                    aps_.frame_end(t0_set_ ? (current_ - t0_) : current_);
                } else if (data_part == 11) {
                    aps_.reset_col_start();
                } else if (data_part == 12) {
                    aps_.signal_col_start();
                } else if (data_part == 13) {
                    aps_.col_end();
                } else if (data_part == 14) {
                    aps_.exposure_start(t0_set_ ? (current_ - t0_) : current_);
                }
                break;

            case 1: // Y address: latched for the following X words.
                last_y_ = static_cast<std::int16_t>(data_part);
                break;

            case 2: // X address, polarity OFF
            case 3: { // X address, polarity ON
                if (!t0_set_) break; // no time base yet — drop
                // Range-check in DEVICE coordinates (X against the device
                // column count, the latched Y against the device row count) —
                // the swap below happens AFTER the validity check, exactly
                // like the reference parser.
                const auto x = static_cast<std::int16_t>(data_part);
                if (x >= device_width_ || last_y_ >= device_height_) break;
                Metavision::EventCD ev;
                ev.t = static_cast<Metavision::timestamp>(current_ - t0_);
                if (invert_xy_) {
                    ev.x = static_cast<std::uint16_t>(last_y_);
                    ev.y = static_cast<std::uint16_t>(x);
                }
                else {
                    ev.x = static_cast<std::uint16_t>(x);
                    ev.y = static_cast<std::uint16_t>(last_y_);
                }
                ev.p = (code & 0x01) ? 1 : 0;
                batch_.push_back(ev);
                break;
            }

            case 4:  // APS pixel: 12-bit ADC value.
                aps_.pixel(data_part);
                break;

            case 5: { // Misc8 data: low 4 bits = code, low byte = data.
                const auto misc8_code =
                    static_cast<std::uint8_t>((data_part & 0x0F00) >> 8);
                const auto misc8_data = static_cast<std::uint8_t>(data_part & 0x00FF);
                if (misc8_code == 0) {
                    imu_.data_byte(misc8_data);
                } else if (misc8_code == 1) {
                    aps_.roi_part1(misc8_data);
                } else if (misc8_code == 2) {
                    aps_.roi_part2(misc8_data);
                } else if (misc8_code == 3) {
                    imu_.scale_config(data_part);
                }
                break;
            }

            case 7: { // Timestamp wrap: data = multiplier of 2^15 µs.
                wrap_add_ += static_cast<std::int64_t>(0x8000) * data_part;
                update_timestamp(wrap_add_);
                break;
            }

            default:
                // 6 = misc10 — consumed and ignored (events-only stream).
                break;
        }
    }

    if (!batch_.empty() && sink) {
        sink(batch_.data(), batch_.data() + batch_.size());
    }
}

void Parser::decode(const std::uint8_t* data, std::size_t size) {
    parse(data, size, nullptr);
}

} // namespace gui::davis
