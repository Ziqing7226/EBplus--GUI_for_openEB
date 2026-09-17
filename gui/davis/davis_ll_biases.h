// gui/davis/davis_ll_biases.h — bridges the DAVIS bias store into the
// Metavision I_LL_Biases interface, so the existing Biases panel, bias
// save/load and the Auto Bias controller operate on a DAVIS camera without
// any changes (Auto Bias greps for "diff_on"/"diff_off" — both present in
// the DAVIS346 table with monotonic linearized ranges).
//
// Values are linearized: coarse/fine biases map to coarse*256+fine in
// [0, 2047]; VDAC biases map to the voltage in [0, 63].

#ifndef GUI_DAVIS_DAVIS_LL_BIASES_H
#define GUI_DAVIS_DAVIS_LL_BIASES_H

#include <map>
#include <string>

#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/utils/device_config.h>

#include "davis_device.h"

namespace gui::davis {

class DavisLLBiases final : public Metavision::I_LL_Biases {
public:
    // NOTE: the base constructor receives a fresh temporary DeviceConfig —
    // base subobjects initialize BEFORE members, so binding it to a member
    // here would pass an uninitialized object into the HAL (segfault in the
    // HAL's internal config-map copy).
    explicit DavisLLBiases(Device& device) :
        Metavision::I_LL_Biases(Metavision::DeviceConfig{}), device_(device) {}

    [[nodiscard]] std::map<std::string, int> get_all_biases() const override {
        std::map<std::string, int> out;
        for (const auto& [name, range] : device_.biases().ranges()) {
            int value = 0;
            device_.biases().get_linear(name, value);
            out[name] = value;
        }
        return out;
    }

protected:
    bool set_impl(const std::string& bias_name, int bias_value) override {
        return device_.biases().set_linear(bias_name, bias_value);
    }

    int get_impl(const std::string& bias_name) const override {
        int value = 0;
        if (!device_.biases().get_linear(bias_name, value)) {
            return 0;
        }
        return value;
    }

    bool get_bias_info_impl(const std::string& bias_name, Metavision::LL_Bias_Info& bias_info) const override {
        const auto ranges = device_.biases().ranges();
        const auto it = ranges.find(bias_name);
        if (it == ranges.end()) return false;
        bias_info = Metavision::LL_Bias_Info(it->second.first, it->second.second,
            "DAVIS " + bias_name, true, "DAVIS");
        return true;
    }

private:
    Device& device_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DAVIS_LL_BIASES_H
