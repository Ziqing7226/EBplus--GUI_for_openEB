// gui/panels/display_panel.cpp

#include "display_panel.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>

namespace gui {

DisplayPanel::DisplayPanel(QWidget* parent) : QWidget(parent) {
    auto* form = new QFormLayout(this);
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Accumulation time: 1.0 - 1000.0 ms (design default 33.3 ms).
    auto* accum_row = new QWidget(this);
    auto* accum_layout = new QHBoxLayout(accum_row);
    accum_layout->setContentsMargins(0, 0, 0, 0);
    accum_slider_ = new QSlider(Qt::Horizontal, accum_row);
    accum_slider_->setRange(10, 10000); // 1.0 - 1000.0 ms in 0.1 ms steps
    accum_slider_->setValue(330);       // 33.0 ms
    accum_spin_ = new QDoubleSpinBox(accum_row);
    accum_spin_->setRange(1.0, 1000.0);
    accum_spin_->setSingleStep(0.1);
    accum_spin_->setSuffix(" ms");
    accum_spin_->setValue(33.0);
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

    mode_combo_ = new QComboBox(this);
    mode_combo_->addItem(tr("Diff Frame"));
    mode_combo_->addItem(tr("Integration"));
    mode_combo_->addItem(tr("Time Decay"));
    mode_combo_->setToolTip(tr("Selects event accumulation mode."));
    form->addRow(tr("Frame mode"), mode_combo_);
    connect(mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DisplayPanel::frame_mode_changed);

    // Wire slider <-> spinbox.
    connect(accum_slider_, &QSlider::valueChanged, this,
            [this](int v) { accum_spin_->setValue(v / 10.0); });
    connect(accum_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double v) {
                QSignalBlocker b(accum_slider_);
                accum_slider_->setValue(static_cast<int>(v * 10));
            });
    connect(accum_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double v) { emit accumulation_time_changed_us(static_cast<int>(v * 1000)); });

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
    return static_cast<int>(accum_spin_->value() * 1000.0);
}

int DisplayPanel::color_palette_index() const {
    return palette_combo_->currentIndex();
}

int DisplayPanel::frame_mode_index() const {
    return mode_combo_->currentIndex();
}

int DisplayPanel::fps() const {
    return fps_spin_->value();
}

int DisplayPanel::fps_limit() const {
    return fps_limit_spin_->value();
}

void DisplayPanel::set_frame_mode(int index) {
    if (index >= 0 && index < mode_combo_->count()) {
        mode_combo_->setCurrentIndex(index);
    }
}

void DisplayPanel::set_accumulation_time_ms(double ms) {
    QSignalBlocker bs(accum_slider_);
    QSignalBlocker bp(accum_spin_);
    accum_spin_->setValue(ms);
    accum_slider_->setValue(static_cast<int>(ms * 10));
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
