// gui/panels/biases_panel.cpp

#include "biases_panel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSlider>
#include <QString>
#include <QStyle>
// QScrollArea no longer needed — the BiasesPanel is hosted directly inside the
// Basic tab's outer scroll area, so an inner scroll is redundant.

#include <metavision/hal/facilities/i_ll_biases.h>

#include "app/camera_controller.h"

namespace gui {

BiasesPanel::BiasesPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(8);

    hint_label_ = new QLabel(tr("No live camera connected."), this);
    hint_label_->setWordWrap(true);
    hint_label_->setProperty("class", "hint");
    outer->addWidget(hint_label_);

    // No inner QScrollArea — the BiasesPanel is already hosted inside the
    // Basic tab's outer scroll area, so an inner scroll would just produce
    // a tiny viewport with its own scrollbar (the user explicitly asked to
    // see all bias rows at once). Using a plain layout lets the outer scroll
    // handle overflow naturally and gives every row its full height.
    container_ = new QWidget(this);
    rows_layout_ = new QVBoxLayout(container_);
    rows_layout_->setContentsMargins(8, 8, 8, 8);
    rows_layout_->setSpacing(8);
    rows_layout_->addStretch(1);
    outer->addWidget(container_, 1);

    container_->setEnabled(false);

    // Debounced apply for wheel/keyboard slider edits (see header, §六-U1).
    apply_debounce_.setSingleShot(true);
    apply_debounce_.setInterval(300);
    connect(&apply_debounce_, &QTimer::timeout, this, [this]() {
        for (auto& r : rows_) {
            if (r.name == pending_apply_) {
                apply_value(r, r.slider->value());
                break;
            }
        }
    });
}

void BiasesPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    clear_rows();
    populate();
}

void BiasesPanel::on_camera_disconnected() {
    camera_ = nullptr;
    clear_rows();
    hint_label_->setText(tr("No live camera connected."));
    hint_label_->setProperty("class", "hint");
    restyle(hint_label_);
    container_->setEnabled(false);
    populated_ = false;
}

void BiasesPanel::save_to_file(const QString& path) {
    if (!camera_) {
        emit error_message(tr("No camera connected."));
        return;
    }
    auto* b = camera_->biases_facility();
    if (!b) {
        emit error_message(tr("Bias facility unavailable on this camera."));
        return;
    }
    try {
        b->save_to_file(std::filesystem::path(path.toStdString()));
        emit info_message(tr("Biases saved to %1").arg(path));
    } catch (const std::exception& e) {
        emit error_message(QString::fromUtf8(e.what()));
    }
}

void BiasesPanel::load_from_file(const QString& path) {
    if (!camera_) {
        emit error_message(tr("No camera connected."));
        return;
    }
    auto* b = camera_->biases_facility();
    if (!b) {
        emit error_message(tr("Bias facility unavailable on this camera."));
        return;
    }
    try {
        b->load_from_file(std::filesystem::path(path.toStdString()));
        refresh_row_values();
        emit info_message(tr("Biases loaded from %1").arg(path));
    } catch (const std::exception& e) {
        emit error_message(QString::fromUtf8(e.what()));
    }
}

// ---------------------------------------------------------------------------

void BiasesPanel::clear_rows() {
    for (auto& row : rows_) {
        // Remove from the layout first so synchronous repopulation below
        // doesn't see stale layout state. Deleting the row widget also
        // destroys its slider/spin/label/button children (Qt's parent-child
        // ownership).
        if (row.row_widget) {
            rows_layout_->removeWidget(row.row_widget);
            row.row_widget->deleteLater();
        }
    }
    rows_.clear();
}

void BiasesPanel::populate() {
    if (!camera_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) {
        hint_label_->setText(tr("Biases not supported by this camera."));
        hint_label_->setProperty("class", "hint");
        restyle(hint_label_);
        container_->setEnabled(false);
        populated_ = false;
        return;
    }

    std::map<std::string, int> all;
    try {
        all = biases->get_all_biases();
    } catch (const std::exception& e) {
        hint_label_->setText(tr("Failed to enumerate biases: %1").arg(QString::fromUtf8(e.what())));
        container_->setEnabled(false);
        populated_ = false;
        return;
    }

    if (all.empty()) {
        hint_label_->setText(tr("Camera reports no configurable biases."));
        container_->setEnabled(false);
        populated_ = false;
        return;
    }

    hint_label_->setText(tr("%1 bias parameter(s) available. Edits apply immediately.")
                             .arg(all.size()));
    hint_label_->setProperty("class", "info");
    restyle(hint_label_);

    // Insert rows before the trailing stretch.
    for (const auto& [name, value] : all) {
        BiasRow row;
        row.name = name;
        row.snapshot_value = value;

        auto* row_widget = new QWidget(container_);
        row.row_widget = row_widget;

        // Resolve range + metadata.
        int lo = 0, hi = 0;
        QString description;
        bool modifiable = true;
        try {
            Metavision::LL_Bias_Info info;
            if (biases->get_bias_info(name, info)) {
                const auto r = info.get_bias_range();
                lo = r.first;
                hi = r.second;
                description = QString::fromStdString(info.get_description());
                modifiable = info.is_modifiable();
            }
        } catch (...) {}
        if (hi <= lo) {
            // Fall back to a wide symmetric window around the current value.
            lo = value - 1000;
            hi = value + 1000;
        }

        auto* hl = new QHBoxLayout(row_widget);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->setSpacing(6);

        auto* label = new QLabel(QString::fromStdString(name), row_widget);
        label->setMinimumWidth(70);
        label->setToolTip(description.isEmpty()
                              ? tr("No description available.")
                              : description);
        row.slider = new QSlider(Qt::Horizontal, row_widget);
        row.slider->setRange(lo, hi);
        row.slider->setValue(value);
        row.slider->setToolTip(description);

        row.spin = new QSpinBox(row_widget);
        row.spin->setRange(lo, hi);
        row.spin->setValue(value);
        row.spin->setToolTip(description);

        auto* btn_reset = new QPushButton(tr("Reset"), row_widget);

        if (!modifiable) {
            row.slider->setEnabled(false);
            row.spin->setEnabled(false);
            btn_reset->setEnabled(false);
            label->setProperty("class", "muted");
        }

        hl->addWidget(label, 0);
        hl->addWidget(row.slider, 1);
        hl->addWidget(row.spin, 0);
        hl->addWidget(btn_reset, 0);

        rows_layout_->insertWidget(rows_layout_->count() - 1, row_widget);

        // Wire edits. Capture name (not the row pointer) to stay safe if the
        // vector reallocates — we look up by name when applying.
        const std::string bias_name = name;
        connect(row.slider, &QSlider::valueChanged, this,
                [this, bias_name, spin = row.spin](int v) {
                    QSignalBlocker b(spin);
                    spin->setValue(v);
                    // Don't apply per tick — valueChanged fires continuously
                    // during a drag and would flood USB writes. Drag edits
                    // are applied on sliderReleased; wheel/keyboard edits
                    // (which never emit sliderReleased) go through a 300 ms
                    // debounce so they still reach the hardware (§六-U1).
                    pending_apply_ = bias_name;
                    apply_debounce_.start();
                });
        connect(row.slider, &QSlider::sliderReleased, this,
                [this, bias_name]() {
                    for (auto& r : rows_) {
                        if (r.name == bias_name) {
                            apply_value(r, r.slider->value());
                            break;
                        }
                    }
                });
        connect(row.spin, QOverload<int>::of(&QSpinBox::valueChanged), this,
                [this, bias_name, slider = row.slider](int v) {
                    QSignalBlocker b(slider);
                    slider->setValue(v);
                    for (auto& r : rows_) {
                        if (r.name == bias_name) { apply_value(r, v); break; }
                    }
                });
        connect(btn_reset, &QPushButton::clicked, this,
                [this, bias_name, snap = value,
                 slider = row.slider, spin = row.spin]() {
                    QSignalBlocker bs(slider);
                    QSignalBlocker bp(spin);
                    slider->setValue(snap);
                    spin->setValue(snap);
                    for (auto& r : rows_) {
                        if (r.name == bias_name) { apply_value(r, snap); break; }
                    }
                });

        rows_.push_back(std::move(row));
    }

    container_->setEnabled(true);
    populated_ = true;
}

void BiasesPanel::apply_value(BiasRow& row, int value) {
    if (!camera_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) return;
    try {
        biases->set(row.name, value);
    } catch (const std::exception& e) {
        emit error_message(tr("Failed to set %1: %2")
                               .arg(QString::fromStdString(row.name))
                               .arg(QString::fromUtf8(e.what())));
        // Roll the slider/spin back to the hardware's actual value so the
        // UI doesn't show a value that never took effect.
        refresh_row_values();
    }
}

void BiasesPanel::refresh_row_values() {
    if (!camera_ || !populated_) return;
    auto* biases = camera_->biases_facility();
    if (!biases) return;
    std::map<std::string, int> all;
    try { all = biases->get_all_biases(); } catch (...) { return; }
    for (auto& row : rows_) {
        auto it = all.find(row.name);
        if (it == all.end()) continue;
        QSignalBlocker bs(row.slider);
        QSignalBlocker bp(row.spin);
        row.slider->setValue(it->second);
        row.spin->setValue(it->second);
    }
}

} // namespace gui
