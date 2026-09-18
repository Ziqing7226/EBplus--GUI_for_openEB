// gui/davis/davis_biases.h — DAVIS346/640 bias register table + encoding, and
// an adapter exposing them through the Metavision I_LL_Biases interface so the
// existing Biases panel and Auto Bias controller work unchanged.
//
// Ported from dv-processing 2.0.4 io/camera/davis.hpp (Apache-2.0): bias
// registers live in SPI module 5 (MODULE_BIAS), addressed by the enum values
// below; coarse/fine biases encode as a 16-bit word (enable/sex/type/level
// flags + 8-bit fine + 3-bit coarse), VDAC biases as voltage(6b)+current(3b).
//
// The panel-facing value is a linearized integer (coarse * 256 + fine for
// coarse/fine biases, voltage for VDAC) with [min, max] metadata, so the
// generic UI and Auto Bias (which greps for "diff_on"/"diff_off") operate on
// a monotonic 1-D knob without knowing about coarse/fine encoding.

#ifndef GUI_DAVIS_DAVIS_BIASES_H
#define GUI_DAVIS_DAVIS_BIASES_H

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace gui::davis {

enum class BiasKind { CoarseFine, VDAC };

struct BiasSpec {
    const char* name;
    BiasKind kind;
    std::uint16_t address; // SPI register address in MODULE_BIAS
    bool sex_n_type;       // coarse/fine: N-type (vs P-type)
    bool type_normal;      // coarse/fine: normal (vs cascode)
    bool current_normal;   // coarse/fine: normal current level
    bool enabled;          // coarse/fine: bias enabled
    std::uint8_t coarse;   // coarse/fine: coarse current; VDAC: current index
    std::uint8_t fine;     // coarse/fine: fine current;   VDAC: voltage
};

/// DAVIS346/640 bias table (names chosen so Auto Bias's "diff_on"/"diff_off"
/// substring lookup matches). Addresses are the dv-processing enum values.
const std::vector<BiasSpec>& davis346_bias_table();

/// DAVIS240A/B/C bias table — own register map (diff@0, diff_on@1, …, no
/// VDAC biases; aps_overflow_level is coarse/fine there) and own defaults.
const std::vector<BiasSpec>& davis240_bias_table();

/// CDAVIS bias tables — own VDAC set (ovg1_low/ovg2_low/…) and own
/// coarse/fine register map (diff@14/diff_on@15/diff_off@16, fall/rise
/// time, array buffers; several biases default disabled).
const std::vector<BiasSpec>& davis_cdavis_bias_table();

/// Model selector: chip identifier (MODULE_SYSINFO / chip identifier) →
/// table. DAVIS240A/B/C (ids 0/1/2) → 240 table; CDAVIS (7) → CDAVIS
/// table; 346/640 (5/6) → the 346/640 table.
const std::vector<BiasSpec>& davis_bias_table_for(int chip_id);

/// Linearized panel value for a coarse/fine pair (monotonic in coarse, fine).
inline int cf_linearize(std::uint8_t coarse, std::uint8_t fine) {
    return static_cast<int>(coarse) * 256 + static_cast<int>(fine);
}
inline void cf_delinearize(int value, std::uint8_t& coarse, std::uint8_t& fine) {
    if (value < 0) value = 0;
    if (value > 7 * 256 + 255) value = 7 * 256 + 255;
    coarse = static_cast<std::uint8_t>(value / 256);
    fine = static_cast<std::uint8_t>(value % 256);
}

/// 16-bit SPI register word for a coarse/fine bias (dv caerBiasCoarseFineGenerate).
std::uint16_t encode_coarse_fine(bool enabled, bool sex_n, bool type_normal,
                                 bool current_normal, std::uint8_t fine, std::uint8_t coarse);
/// Inverse of encode_coarse_fine (returns {flags-masked fields…} as the raw word decode).
void decode_coarse_fine(std::uint16_t word, bool& enabled, bool& sex_n, bool& type_normal,
                        bool& current_normal, std::uint8_t& fine, std::uint8_t& coarse);
/// 16-bit SPI register word for a VDAC bias (dv caerBiasVDACGenerate).
std::uint16_t encode_vdac(std::uint8_t voltage, std::uint8_t current);
/// 16-bit SPI register word for a shifted-source bias
/// (dv caerBiasShiftedSourceGenerate); ref/reg are 6-bit fields.
std::uint16_t encode_shifted_source(std::uint8_t ref, std::uint8_t reg);

/// Linearized reference default for a DAVIS bias name, e.g.
/// diff_on → 1535 (false when the name is unknown). Uses the 346/640 table.
bool davis_reference_default(const std::string& name, int& value);
/// Model-aware variant (e.g. diff_off → 1025 on 346/640, 1024 on 240).
bool davis_reference_default_for(int chip_id, const std::string& name, int& value);

/// DAVIS346 shifted-source bias register addresses (dv constants).
constexpr std::uint16_t DAVIS346_BIAS_SSP = 35;
constexpr std::uint16_t DAVIS346_BIAS_SSN = 36;
/// DAVIS240 shifted-source bias register addresses.
constexpr std::uint16_t DAVIS240_BIAS_SSP = 20;
constexpr std::uint16_t DAVIS240_BIAS_SSN = 21;

/// Bias state + register synthesis, decoupled from USB so it is unit-testable.
/// @p send_word is invoked for every register change (device write).
class BiasStore {
public:
    /// @p on_write called as (spi_address, register_word) for each change.
    explicit BiasStore(std::function<void(std::uint16_t address, std::uint16_t word)> on_write);

    /// Programs all defaults to the device (reference power-up table).
    void apply_defaults();

    /// @brief Phase 7: selects the model bias table (DAVIS240 vs 346/640).
    /// Must be called before apply_defaults(); apply_defaults() restores
    /// from the selected table.
    void set_table(const std::vector<BiasSpec>& table);

    /// Linearized value lookup/set by name (false when unknown).
    bool get_linear(const std::string& name, int& value) const;
    bool set_linear(const std::string& name, int value);

    /// Panel metadata: all names with their [min, max] linear ranges.
    std::map<std::string, std::pair<int, int>> ranges() const;

private:
    const BiasSpec* find(const std::string& name) const;
    void send(const BiasSpec& spec);

    std::function<void(std::uint16_t address, std::uint16_t word)> on_write_;
    const std::vector<BiasSpec>* model_table_;
    std::vector<BiasSpec> table_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DAVIS_BIASES_H
