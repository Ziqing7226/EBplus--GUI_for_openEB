// gui/davis/dvxplorer_parser.cpp — see dvxplorer_parser.h.
// Ported from dv-processing 2.0.4 parsers/dvxplorer_parser.hpp (Apache-2.0).

#include "dvxplorer_parser.h"

#include <cstring>

namespace gui::davis {

DvxParser::DvxParser(int width, int height)
    : width_(width), height_(height) {}

void DvxParser::reset() {
    t0_set_ = false;
    wrap_add_ = 0;
    t0_ = 0;
    current_ = 0;
    last_x_ = 0;
    last_yg1_ = 0;
    last_yg2_ = 0;
    batch_.clear();
}

void DvxParser::update_timestamp(std::int64_t ts) {
    if (!t0_set_) {
        t0_set_ = true;
        t0_ = ts;
    }
    current_ = ts;
}

void DvxParser::parse(const std::uint8_t* data, std::size_t size, const EventSink& sink) {
    batch_.clear();
    const std::size_t words = size / 2;
    for (std::size_t i = 0; i < words; ++i) {
        std::uint16_t event = 0;
        std::memcpy(&event, data + i * 2, 2);

        if ((event & 0x8000) != 0) {
            update_timestamp(wrap_add_ + (event & 0x7FFF));
            continue;
        }

        const auto code = static_cast<std::uint8_t>((event & 0x7000) >> 12);
        const auto data_part = static_cast<std::uint16_t>(event & 0x0FFF);

        switch (code) {
            case 0: // Special events.
                if (data_part == 1) {
                    // Timestamp reset: the FPGA restarts its 15-bit counter.
                    wrap_add_ = 0;
                    t0_set_ = false;
                    current_ = 0;
                    last_x_ = 0;
                    last_yg1_ = 0;
                    last_yg2_ = 0;
                }
                break;

            case 1: { // X column address (10 bits; 1023 = reset marker).
                const auto column_address = static_cast<std::int16_t>(data_part & 0x03FF);
                last_x_ = (column_address == 1023) ? 0 : column_address;
                break;
            }

            case 2:
            case 3: { // 8-pixel group event presence and polarity.
                if (!t0_set_) break;
                // bit 8 CLEAR = ON (positive), SET = OFF (negative).
                const std::uint16_t last_y =
                    (code == 3) ? static_cast<std::uint16_t>(last_yg1_)
                                : static_cast<std::uint16_t>(last_yg2_);
                for (std::uint16_t i = 0, mask = 0x0001; i < 8; ++i, mask <<= 1) {
                    if ((data_part & mask) == 0) continue;
                    Metavision::EventCD ev;
                    ev.t = static_cast<Metavision::timestamp>(current_ - t0_);
                    ev.x = static_cast<std::uint16_t>(last_x_);
                    ev.y = static_cast<std::uint16_t>(last_y + i);
                    // Reference: bit 8 CLEAR = ON (positive), SET = OFF.
                    ev.p = (data_part & 0x0100) ? 0 : 1;
                    if (ev.x >= width_ || ev.y >= height_) continue;
                    batch_.push_back(ev);
                }
                break;
            }

            case 4: { // Y-group address latch: two groups, ±offset from base.
                const int g1 = static_cast<int>(data_part & 0x003F);
                const int offset = static_cast<int>((data_part >> 6) & 0x001F);
                const int g2 = (data_part & 0x0800) ? (g1 - offset) : (g1 + offset);
                last_yg1_ = static_cast<std::int16_t>(g1 * 8);
                last_yg2_ = static_cast<std::int16_t>(g2 * 8);
                break;
            }

            case 7: { // Timestamp wrap: data = multiplier of 2^15 µs.
                wrap_add_ += static_cast<std::int64_t>(0x8000) * data_part;
                update_timestamp(wrap_add_);
                break;
            }

            default:
                // 5/6 = IMU/misc data — consumed and ignored (events-only).
                break;
        }
    }

    if (!batch_.empty() && sink) {
        sink(batch_.data(), batch_.data() + batch_.size());
    }
}

} // namespace gui::davis
