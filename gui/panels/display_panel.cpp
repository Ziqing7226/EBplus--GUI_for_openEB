// gui/panels/display_panel.cpp

#include "display_panel.h"

#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>

#include <cmath>

namespace {
// Exponential slider mapping: slider position [0, 1000] → us [10, 100000].
// The mapping is value = 10 * 10000^(pos/1000), so the slider midpoint (500)
// yields 1000 us. This gives fine granularity at low values (10-1000 us
// occupies half the slider) while still reaching 100000 us at the top.
constexpr double kSliderMinUs  = 10.0;
constexpr double kSliderMaxUs  = 100000.0;
constexpr int    kSliderSteps  = 1000;

int slider_pos_to_us(int pos) {
    const double t = static_cast<double>(pos) / kSliderSteps;
    return static_cast<int>(std::round(
        kSliderMinUs * std::pow(kSliderMaxUs / kSliderMinUs, t)));
}

int us_to_slider_pos(int us) {
    if (us <= static_cast<int>(kSliderMinUs)) return 0;
    if (us >= static_cast<int>(kSliderMaxUs)) return kSliderSteps;
    const double t = std::log(static_cast<double>(us) / kSliderMinUs) /
                     std::log(kSliderMaxUs / kSliderMinUs);
    return static_cast<int>(std::round(t * kSliderSteps));
}
} // namespace

namespace gui {

DisplayPanel::DisplayPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* form = new QFormLayout(this);
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Accumulation time: spinbox 1-1000000 us, slider uses exponential
    // mapping [0, 1000] → [10, 100000] us for fine low-end control.
    // Default 33000 us. The spinbox allows precise values outside the
    // slider's mapped range; QSlider clamps silently (no feedback).
    auto* accum_row = new QWidget(this);
    auto* accum_layout = new QHBoxLayout(accum_row);
    accum_layout->setContentsMargins(0, 0, 0, 0);
    accum_slider_ = new QSlider(Qt::Horizontal, accum_row);
    accum_slider_->setRange(0, kSliderSteps);
    accum_slider_->setValue(us_to_slider_pos(33000));
    accum_spin_ = new QSpinBox(accum_row);
    accum_spin_->setRange(1, 1000000);
    accum_spin_->setSingleStep(100);
    accum_spin_->setSuffix(" us");
    accum_spin_->setValue(33000);
    accum_layout->addWidget(accum_slider_, 1);
    accum_layout->addWidget(accum_spin_, 0);
    form->addRow(tr("Accumulation"), accum_row);

    // Frame rate: 1 .. fps_limit (default 30 fps, limit 60).
    fps_spin_ = new QSpinBox(this);
    fps_spin_->setRange(1, 60);
    fps_spin_->setValue(30);
    fps_spin_->setSuffix(tr(" fps"));
    fps_spin_->setToolTip(tr("Display frame rate. Clamped to the FPS limit."));
    form->addRow(tr("Frame rate"), fps_spin_);

    // FPS limit: 1 .. 1000 (default 60). Changing this updates the fps range.
    fps_limit_spin_ = new QSpinBox(this);
    fps_limit_spin_->setRange(1, 1000);
    fps_limit_spin_->setValue(60);
    fps_limit_spin_->setSuffix(tr(" fps"));
    fps_limit_spin_->setToolTip(tr("Upper bound on display frame rate."));
    form->addRow(tr("FPS limit"), fps_limit_spin_);

    palette_combo_ = new QComboBox(this);
    palette_combo_->addItem(tr("Dark"), 0);
    palette_combo_->addItem(tr("Light"), 1);
    palette_combo_->addItem(tr("CoolWarm"), 2);
    palette_combo_->addItem(tr("Gray"), 3);
    form->addRow(tr("Color theme"), palette_combo_);

    // Wire slider <-> spinbox with exponential mapping.
    // Block the peer widget to prevent feedback loops: the round-trip
    // slider→us→slider is not identity due to integer rounding, so
    // unblocked feedback would cause the slider to jump/oscillate.
    connect(accum_slider_, &QSlider::valueChanged, this,
            [this](int pos) {
                const int us = slider_pos_to_us(pos);
                QSignalBlocker b(accum_spin_);
                accum_spin_->setValue(us);
                emit accumulation_time_changed_us(us);
            });
    connect(accum_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this](int v) {
                QSignalBlocker b(accum_slider_);
                accum_slider_->setValue(us_to_slider_pos(v));
                emit accumulation_time_changed_us(v);
            });

    // FPS spinbox -> signal.
    connect(fps_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this](int v) { emit fps_changed(v); });

    // FPS-limit spinbox -> signal + update fps range.
    connect(fps_limit_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this](int limit) {
                // Clamp the fps range to the new limit. If the current fps
                // exceeds the limit, QSpinBox::setMaximum will clamp the
                // value and emit valueChanged, which flows through to
                // fps_changed and ultimately to FramePipeline::set_fps.
                fps_spin_->setMaximum(limit);
                emit fps_limit_changed(limit);
            });

    connect(palette_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &DisplayPanel::color_palette_changed);
}

int DisplayPanel::accumulation_time_us() const {
    return accum_spin_->value();
}

int DisplayPanel::fps() const {
    return fps_spin_->value();
}

int DisplayPanel::fps_limit() const {
    return fps_limit_spin_->value();
}

void DisplayPanel::set_accumulation_time_us(int us) {
    QSignalBlocker bs(accum_slider_);
    QSignalBlocker bp(accum_spin_);
    accum_spin_->setValue(us);
    accum_slider_->setValue(us_to_slider_pos(us));
}

void DisplayPanel::set_fps(int fps) {
    QSignalBlocker b(fps_spin_);
    fps_spin_->setValue(fps);
}

void DisplayPanel::set_fps_limit(int limit) {
    QSignalBlocker bl(fps_limit_spin_);
    QSignalBlocker bf(fps_spin_);
    fps_limit_spin_->setValue(limit);
    fps_spin_->setMaximum(limit);
}

} // namespace gui
