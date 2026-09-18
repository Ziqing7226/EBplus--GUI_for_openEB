// gui/davis/dvxplorer_ll_biases.h — bridges the DVXplorer ON/OFF contrast
// thresholds into the Metavision I_LL_Biases interface so the Biases panel
// exposes them like camera biases. Note: Auto Bias does not attach here (its
// matcher looks for diff_on/diff_off names) — DVXplorer thresholds are plain
// sensitivity knobs with no Auto Bias support yet.

#ifndef GUI_DAVIS_DVXPLORER_LL_BIASES_H
#define GUI_DAVIS_DVXPLORER_LL_BIASES_H

#include <map>
#include <string>

#include <metavision/hal/facilities/i_ll_biases.h>
#include <metavision/hal/utils/device_config.h>

#include "dvxplorer_device.h"

namespace gui::davis {

class DvxLLBiases final : public Metavision::I_LL_Biases {
public:
    explicit DvxLLBiases(DvxplorerDevice& device) :
        Metavision::I_LL_Biases(Metavision::DeviceConfig{}), device_(device) {}

    [[nodiscard]] std::map<std::string, int> get_all_biases() const override {
        return {{"contrast_on", device_.contrast_on()},
                {"contrast_off", device_.contrast_off()}};
    }

protected:
    bool set_impl(const std::string& bias_name, int bias_value) override {
        if (bias_name == "contrast_on") {
            device_.set_contrast_on(bias_value);
            return true;
        }
        if (bias_name == "contrast_off") {
            device_.set_contrast_off(bias_value);
            return true;
        }
        return false;
    }

    int get_impl(const std::string& bias_name) const override {
        if (bias_name == "contrast_on") return device_.contrast_on();
        if (bias_name == "contrast_off") return device_.contrast_off();
        return 0;
    }

    bool get_bias_info_impl(const std::string& bias_name,
                            Metavision::LL_Bias_Info& bias_info) const override {
        if (bias_name != "contrast_on" && bias_name != "contrast_off") return false;
        bias_info = Metavision::LL_Bias_Info(0, 17, "DVXplorer " + bias_name, true, "DVXplorer");
        return true;
    }

private:
    DvxplorerDevice& device_;
};

} // namespace gui::davis

#endif // GUI_DAVIS_DVXPLORER_LL_BIASES_H
