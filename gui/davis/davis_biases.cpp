// gui/davis/davis_biases.cpp — see davis_biases.h.
// Default values ported from dv-processing 2.0.4 io/camera/davis.hpp
// (DAVIS346/DAVIS640 constructor, Apache-2.0).

#include "davis_biases.h"

#include <stdexcept>

namespace gui::davis {

const std::vector<BiasSpec>& davis346_bias_table() {
    static const std::vector<BiasSpec> table = {
        // VDAC biases, addresses 0..4. Fields carry {coarse = current index,
        // fine = voltage} (dv defaults were voltage/current pairs:
        // 27/6, 21/6, 32/7, 1/7, 21/7).
        {"aps_overflow_level",   BiasKind::VDAC, 0, true,  true,  true,  true,  6, 27},
        {"aps_cascode",          BiasKind::VDAC, 1, true,  true,  true,  true,  6, 21},
        {"adc_reference_high",   BiasKind::VDAC, 2, true,  true,  true,  true,  7, 32},
        {"adc_reference_low",    BiasKind::VDAC, 3, true,  true,  true,  true,  7,  1},
        {"adc_test_voltage",     BiasKind::VDAC, 4, true,  true,  true,  true,  7, 21},
        // Coarse/fine biases, addresses 8..34 (enum values in dv-processing).
        {"local_buffer",         BiasKind::CoarseFine,  8, true,  true,  true,  true,  5, 164},
        {"pad_follower",         BiasKind::CoarseFine,  9, false, true,  true,  true,  0,   0},
        {"diff",                 BiasKind::CoarseFine, 10, true,  true,  true,  true,  4,  39},
        {"diff_on",              BiasKind::CoarseFine, 11, true,  true,  true,  true,  5, 255},
        {"diff_off",             BiasKind::CoarseFine, 12, true,  true,  true,  true,  4,   1},
        {"pixel_inverter",       BiasKind::CoarseFine, 13, true,  true,  true,  true,  6, 144},
        {"photoreceptor",        BiasKind::CoarseFine, 14, false, true,  true,  true,  2,  58},
        {"photocircuit_follower", BiasKind::CoarseFine, 15, false, true, true,  true,  1,  16},
        {"refractory",           BiasKind::CoarseFine, 16, false, true,  true,  true,  4,  25},
        {"readout_buffer",       BiasKind::CoarseFine, 17, false, true,  true,  true,  6,  20},
        {"aps_readout_follower", BiasKind::CoarseFine, 18, true,  true,  true,  true,  6, 219},
        {"adc_comparator",       BiasKind::CoarseFine, 19, false, true,  true,  true,  5,  20},
        {"col_select_low",       BiasKind::CoarseFine, 20, true,  true,  true,  true,  0,   1},
        {"dac_buffer",           BiasKind::CoarseFine, 21, false, true,  true,  true,  6,  60},
        {"lcol_timeout",         BiasKind::CoarseFine, 22, true,  true,  true,  true,  5,  49},
        {"aer_pull_down",        BiasKind::CoarseFine, 23, true,  true,  true,  true,  6,  91},
        {"aer_pull_up_x",        BiasKind::CoarseFine, 24, false, true,  true,  true,  4,  80},
        {"aer_pull_up_y",        BiasKind::CoarseFine, 25, false, true,  true,  true,  7, 152},
        {"if_refr_bn",           BiasKind::CoarseFine, 26, true,  true,  true,  false, 0,   0},
        {"if_thr_bn",            BiasKind::CoarseFine, 27, true,  true,  true,  false, 0,   0},
        {"bias_buffer",          BiasKind::CoarseFine, 34, true,  true,  true,  true,  5, 254},
    };
    return table;
}

std::uint16_t encode_coarse_fine(bool enabled, bool sex_n, bool type_normal,
                                 bool current_normal, std::uint8_t fine, std::uint8_t coarse) {
    std::uint16_t v = 0;
    if (enabled) v |= 0x01;
    if (sex_n) v |= 0x02;
    if (type_normal) v |= 0x04;
    if (current_normal) v |= 0x08;
    v |= static_cast<std::uint16_t>((fine & 0xFF) << 4);
    v |= static_cast<std::uint16_t>((coarse & 0x07) << 12);
    return v;
}

void decode_coarse_fine(std::uint16_t word, bool& enabled, bool& sex_n, bool& type_normal,
                        bool& current_normal, std::uint8_t& fine, std::uint8_t& coarse) {
    enabled = (word & 0x01) != 0;
    sex_n = (word & 0x02) != 0;
    type_normal = (word & 0x04) != 0;
    current_normal = (word & 0x08) != 0;
    fine = static_cast<std::uint8_t>((word >> 4) & 0xFF);
    coarse = static_cast<std::uint8_t>((word >> 12) & 0x07);
}

std::uint16_t encode_vdac(std::uint8_t voltage, std::uint8_t current) {
    std::uint16_t v = 0;
    v |= static_cast<std::uint16_t>(voltage & 0x3F);
    v |= static_cast<std::uint16_t>((current & 0x07) << 6);
    return v;
}

bool davis_reference_default(const std::string& name, int& value) {
    for (const auto& spec : davis346_bias_table()) {
        if (name == spec.name) {
            value = (spec.kind == BiasKind::VDAC)
                        ? spec.fine
                        : cf_linearize(spec.coarse, spec.fine);
            return true;
        }
    }
    return false;
}

std::uint16_t encode_shifted_source(std::uint8_t ref, std::uint8_t reg) {
    // Reference defaults: operating mode = SHIFTED_SOURCE, voltage level =
    // SPLIT_GATE (both encode as 0 bits).
    std::uint16_t v = 0;
    v |= static_cast<std::uint16_t>((ref & 0x3F) << 4);
    v |= static_cast<std::uint16_t>((reg & 0x3F) << 10);
    return v;
}

// ---------------------------------------------------------------------------

BiasStore::BiasStore(std::function<void(std::uint16_t, std::uint16_t)> on_write)
    : on_write_(std::move(on_write)), table_(davis346_bias_table()) {}

void BiasStore::send(const BiasSpec& spec) {
    std::uint16_t word = 0;
    if (spec.kind == BiasKind::CoarseFine) {
        word = encode_coarse_fine(spec.enabled, spec.sex_n_type, spec.type_normal,
                                  spec.current_normal, spec.fine, spec.coarse);
    }
    else {
        // VDAC "fine" field stores the voltage, "coarse" the current index.
        word = encode_vdac(spec.fine, spec.coarse);
    }
    if (on_write_) on_write_(spec.address, word);
}

void BiasStore::apply_defaults() {
    // Restore from the IMMUTABLE reference table — runtime adjustments
    // (set_linear) mutate table_ and would otherwise become the new
    // "defaults" on reconnect.
    table_ = davis346_bias_table();
    for (const auto& spec : table_) {
        send(spec);
    }
}

const BiasSpec* BiasStore::find(const std::string& name) const {
    for (const auto& spec : table_) {
        if (name == spec.name) return &spec;
    }
    return nullptr;
}

bool BiasStore::get_linear(const std::string& name, int& value) const {
    const auto* spec = find(name);
    if (spec == nullptr) return false;
    if (spec->kind == BiasKind::VDAC) {
        value = spec->fine; // voltage in [0, 63]
    }
    else {
        value = cf_linearize(spec->coarse, spec->fine);
    }
    return true;
}

bool BiasStore::set_linear(const std::string& name, int value) {
    auto* spec = const_cast<BiasSpec*>(find(name));
    if (spec == nullptr) return false;
    if (spec->kind == BiasKind::CoarseFine) {
        cf_delinearize(value, spec->coarse, spec->fine);
    }
    else {
        if (value < 0) value = 0;
        if (value > 63) value = 63;
        spec->fine = static_cast<std::uint8_t>(value); // voltage
    }
    send(*spec);
    return true;
}

std::map<std::string, std::pair<int, int>> BiasStore::ranges() const {
    std::map<std::string, std::pair<int, int>> out;
    for (const auto& spec : table_) {
        if (spec.kind == BiasKind::CoarseFine) {
            out[spec.name] = {0, 7 * 256 + 255};
        }
        else {
            out[spec.name] = {0, 63};
        }
    }
    return out;
}

} // namespace gui::davis
