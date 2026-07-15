// gui/panels/information_panel.cpp

#include "information_panel.h"

#include <QFormLayout>
#include <QLabel>

namespace gui {

InformationPanel::InformationPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* form = new QFormLayout(this);
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    auto mk = [this, form](const QString& title) {
        auto* lbl = new QLabel("—", this);
        lbl->setTextInteractionFlags(Qt::TextSelectableByMouse);
        form->addRow(title, lbl);
        return lbl;
    };

    // --- Camera metadata ---
    value_model_       = mk(tr("Model"));
    value_resolution_  = mk(tr("Resolution"));
    value_serial_      = mk(tr("Serial"));
    value_integrator_  = mk(tr("Integrator"));
    value_plugin_      = mk(tr("Plugin"));
    value_encoding_    = mk(tr("Encoding"));
    value_firmware_    = mk(tr("Firmware"));
    value_source_      = mk(tr("Source"));

    // --- Live performance ---
    form->addRow(new QLabel(this));  // spacer
    value_latency_        = mk(tr("Latency"));
    value_throughput_     = mk(tr("Throughput"));
    value_algo_status_    = mk(tr("Algorithms"));
    value_drop_rate_      = mk(tr("Drop rate"));
}

void InformationPanel::set_info(const SensorInfo& info) {
    value_model_->setText(info.generation_name.isEmpty()
                              ? QStringLiteral("—")
                              : QStringLiteral("%1 v%2.%3")
                                    .arg(info.generation_name)
                                    .arg(info.generation_major)
                                    .arg(info.generation_minor));
    value_resolution_->setText(QStringLiteral("%1 × %2").arg(info.width).arg(info.height));
    value_serial_->setText(info.serial.isEmpty() ? tr("—") : info.serial);
    value_integrator_->setText(info.integrator.isEmpty() ? tr("—") : info.integrator);
    value_plugin_->setText(info.plugin_name.isEmpty() ? tr("—") : info.plugin_name);
    value_encoding_->setText(info.encoding_format.isEmpty() ? tr("—") : info.encoding_format);
    value_firmware_->setText(info.firmware_version.isEmpty() ? tr("—") : info.firmware_version);
    value_source_->setText(info.is_file ? tr("File playback") : tr("Live camera"));
}

void InformationPanel::set_performance(double latency_ms, double throughput_mbs) {
    value_latency_->setText(QStringLiteral("%1 ms").arg(latency_ms, 0, 'f', 1));
    value_throughput_->setText(QStringLiteral("%1 MB/s").arg(throughput_mbs, 0, 'f', 1));
}

void InformationPanel::set_algo_status(int active, int overloaded) {
    if (overloaded > 0) {
        value_algo_status_->setText(
            QStringLiteral("%1 active, %2 overloaded").arg(active).arg(overloaded));
    } else {
        value_algo_status_->setText(
            QStringLiteral("%1 active").arg(active));
    }
}

void InformationPanel::set_drop_rate(double max_drop_pct) {
    value_drop_rate_->setText(QStringLiteral("%1%").arg(max_drop_pct, 0, 'f', 1));
}

void InformationPanel::clear() {
    for (auto* lbl : {value_model_, value_resolution_, value_serial_, value_integrator_,
                      value_plugin_, value_encoding_, value_firmware_, value_source_,
                      value_latency_, value_throughput_, value_algo_status_,
                      value_drop_rate_}) {
        lbl->setText(tr("—"));
    }
}

} // namespace gui
