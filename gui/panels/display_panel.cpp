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

    // Frame mode: which algorithm generates the displayed frame. Integration
    // keeps the classic accumulation frame; the other modes route events
    // through the frame-mode renderer (live: timer at fps; file: per window).
    frame_mode_combo_ = new QComboBox(this);
    frame_mode_combo_->addItem(tr("Integration"), static_cast<int>(FrameMode::Integration));
    frame_mode_combo_->addItem(tr("Contrast Map"), static_cast<int>(FrameMode::ContrastMap));
    frame_mode_combo_->addItem(tr("Histogram"), static_cast<int>(FrameMode::Histo));
    frame_mode_combo_->addItem(tr("Diff"), static_cast<int>(FrameMode::Diff));
    frame_mode_combo_->addItem(tr("Time Decay"), static_cast<int>(FrameMode::TimeDecay));
    frame_mode_combo_->addItem(tr("Events Integration"), static_cast<int>(FrameMode::EventsIntegration));
    frame_mode_combo_->setToolTip(tr("Display frame generation mode. Integration "
        "shows the accumulation window; Contrast Map shows per-pixel ON−OFF "
        "contrast; Histogram shows positive (green) and negative (red) event "
        "counts; Diff shows the signed polarity sum; Time Decay and Events "
        "Integration render decayed intensities."));
    form->addRow(tr("Frame mode"), frame_mode_combo_);

    // Color theme: palette for the Integration and TimeDecay renders. The
    // other modes use fixed color mappings (grayscale / red-green), so the
    // row is hidden for them (update_frame_mode_rows).
    palette_combo_ = new QComboBox(this);
    palette_combo_->addItem(tr("Dark"), 0);
    palette_combo_->addItem(tr("Light"), 1);
    palette_combo_->addItem(tr("CoolWarm"), 2);
    palette_combo_->addItem(tr("Gray"), 3);
    form->addRow(tr("Color theme"), palette_combo_);

    // Decay time (µs) for TimeDecay / EventsIntegration. The row is shown
    // only for those two modes (update_frame_mode_rows).
    decay_spin_ = new QSpinBox(this);
    decay_spin_->setRange(1000, 10000000);
    decay_spin_->setSingleStep(1000);
    decay_spin_->setValue(100000);
    decay_spin_->setSuffix(tr(" us"));
    decay_spin_->setToolTip(tr("Characteristic decay time for the Time Decay "
        "and Events Integration frame modes. Longer values keep older events "
        "visible for longer."));
    form->addRow(tr("Decay time"), decay_spin_);
    form_ = form;

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

    // Frame mode -> signal + decay-time row visibility.
    connect(frame_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) {
                update_frame_mode_rows();
                emit frame_mode_changed(frame_mode());
            });
    connect(decay_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
            [this](int v) { emit decay_time_changed_us(v); });

    update_frame_mode_rows();
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

FrameMode DisplayPanel::frame_mode() const {
    if (!frame_mode_combo_) return FrameMode::Integration;
    return static_cast<FrameMode>(frame_mode_combo_->currentData().toInt());
}

int DisplayPanel::decay_time_us() const {
    return decay_spin_ ? decay_spin_->value() : 100000;
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

void DisplayPanel::set_frame_mode(FrameMode mode) {
    if (!frame_mode_combo_) return;
    QSignalBlocker b(frame_mode_combo_);
    const int idx = frame_mode_combo_->findData(static_cast<int>(mode));
    if (idx >= 0) frame_mode_combo_->setCurrentIndex(idx);
    update_frame_mode_rows();
}

void DisplayPanel::set_decay_time_us(int us) {
    if (!decay_spin_) return;
    QSignalBlocker b(decay_spin_);
    decay_spin_->setValue(us);
    update_frame_mode_rows();
}

void DisplayPanel::update_frame_mode_rows() {
    if (!frame_mode_combo_ || !form_) return;
    const FrameMode m = frame_mode();
    const bool uses_decay = m == FrameMode::TimeDecay ||
                            m == FrameMode::EventsIntegration;
    form_->setRowVisible(decay_spin_, uses_decay);
    decay_spin_->setEnabled(uses_decay);
    const bool uses_palette = m == FrameMode::Integration ||
                              m == FrameMode::TimeDecay;
    form_->setRowVisible(palette_combo_, uses_palette);
    palette_combo_->setEnabled(uses_palette);
}

} // namespace gui
