// gui/recorder/playback_controls.cpp

#include "playback_controls.h"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>

#include <cmath>

#include "playback_controller.h"

namespace gui {

PlaybackControls::PlaybackControls(QWidget* parent) : QWidget(parent) {
    auto* lay = new QHBoxLayout(this);
    lay->setContentsMargins(6, 2, 6, 2);

    btn_play_ = new QPushButton(tr("Play"), this);
    btn_step_ = new QPushButton(tr("Step"), this);
    slider_   = new QSlider(Qt::Horizontal, this);
    lbl_cur_  = new QLabel("0.000 s", this);
    lbl_dur_  = new QLabel("0.000 s", this);
    chk_loop_ = new QCheckBox(tr("Loop"), this);

    // Linked playback-rate controls (design §3.3.2):
    //   multiplier = frame_rate * time_window_us / 1e6
    // Editing one field locks another and recomputes the third, mirroring the
    // PlaybackController's locking semantics:
    //   - edit time_window → frame_rate locked, multiplier recomputed
    //   - edit frame_rate  → time_window locked, multiplier recomputed
    //   - edit multiplier  → frame_rate locked, time_window recomputed
    spd_tw_ = new QSpinBox(this);
    spd_tw_->setRange(1, 1000000);      // 1 μs .. 1 s, integer only
    spd_tw_->setSingleStep(1000);
    spd_tw_->setValue(33000);
    spd_tw_->setSuffix(QStringLiteral(" us"));
    spd_tw_->setToolTip(tr("Event accumulation window per frame (microseconds, integer)."));

    spd_fps_ = new QSpinBox(this);
    spd_fps_->setRange(1, 60);          // upper bound = fps_limit (synced at runtime)
    spd_fps_->setValue(30);
    spd_fps_->setSuffix(tr(" fps"));
    spd_fps_->setToolTip(tr("Display frame rate. Clamped to the FPS limit."));

    spd_mult_ = new QDoubleSpinBox(this);
    spd_mult_->setRange(0.000001, 100.0);
    spd_mult_->setDecimals(6);
    spd_mult_->setSingleStep(0.1);
    spd_mult_->setValue(1.0);
    spd_mult_->setPrefix(QStringLiteral("x"));
    spd_mult_->setToolTip(tr("Playback multiplier = fps * time-window / 1e6. "
                             ">1 = fast-forward (max), <=1 = real-time / slow-motion."));

    slider_->setMinimum(0);
    slider_->setMaximum(1000);

    lay->addWidget(btn_play_);
    lay->addWidget(btn_step_);
    lay->addWidget(lbl_cur_, 0);
    lay->addWidget(slider_, 1);
    lay->addWidget(lbl_dur_, 0);
    lay->addSpacing(8);
    lay->addWidget(new QLabel(tr("Window"), this));
    lay->addWidget(spd_tw_);
    lay->addWidget(new QLabel(tr("Rate"), this));
    lay->addWidget(spd_fps_);
    lay->addWidget(spd_mult_);
    lay->addSpacing(8);
    lay->addWidget(chk_loop_);

    activate(false);

    connect(btn_play_, &QPushButton::clicked, this, [this]() {
        if (controller_) controller_->toggle_play_pause();
    });
    connect(btn_step_, &QPushButton::clicked, this, [this]() {
        if (!controller_) return;
        controller_->pause();
        // Advance by one accumulation window and render immediately.
        // seek_file() renders the frame synchronously, so no play-then-pause
        // hack is needed (unlike the CDFrameGenerator-based approach).
        const Metavision::timestamp step_us = controller_->time_window_us();
        const Metavision::timestamp next = controller_->position_us() + step_us;
        controller_->seek(next);
    });
    connect(slider_, &QSlider::sliderPressed,  this, [this]() { seeking_ = true;  });
    connect(slider_, &QSlider::sliderReleased, this, [this]() { seeking_ = false; });
    connect(slider_, &QSlider::sliderMoved,    this, &PlaybackControls::on_slider_moved);
    // Each field delegates to the controller, which is the single source of
    // truth. The controller delegates to FramePipeline, whose signals sync
    // both this panel and the DisplayPanel.
    connect(spd_tw_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v) {
        if (controller_) controller_->set_time_window_us(
            static_cast<Metavision::timestamp>(v));
    });
    connect(spd_fps_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int v) {
        if (controller_) controller_->set_frame_rate(static_cast<std::uint16_t>(v));
    });
    connect(spd_mult_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double v) {
        if (controller_) controller_->set_multiplier(v);
    });
    connect(chk_loop_, &QCheckBox::toggled, this, [this](bool on) {
        if (controller_) controller_->set_loop(on);
    });
}

void PlaybackControls::set_controller(PlaybackController* controller) {
    if (controller_ == controller) return;
    if (controller_) {
        disconnect(controller_, nullptr, this, nullptr);
    }
    controller_ = controller;
    if (controller_) {
        connect(controller_, &PlaybackController::state_changed,
                this, &PlaybackControls::on_state_changed);
        connect(controller_, &PlaybackController::position_changed,
                this, &PlaybackControls::on_position_changed);
        connect(controller_, &PlaybackController::opened,
                this, &PlaybackControls::on_opened);
        connect(controller_, &PlaybackController::multiplier_changed,
                this, &PlaybackControls::on_multiplier_changed);
        connect(controller_, &PlaybackController::loop_changed,
                this, &PlaybackControls::on_loop_changed);
    }
}

void PlaybackControls::activate(bool on) {
    setVisible(on);
    setEnabled(on);
}

void PlaybackControls::on_state_changed(bool playing) {
    btn_play_->setText(playing ? tr("Pause") : tr("Play"));
}

void PlaybackControls::on_opened(Metavision::timestamp dur) {
    lbl_dur_->setText(format_time(dur));
    slider_->setValue(0);
}

void PlaybackControls::on_position_changed(Metavision::timestamp pos, Metavision::timestamp dur) {
    lbl_cur_->setText(format_time(pos));
    if (seeking_) return;
    if (dur > 0) {
        slider_->blockSignals(true);
        slider_->setValue(static_cast<int>(pos * slider_->maximum() / dur));
        slider_->blockSignals(false);
    }
}

void PlaybackControls::on_slider_moved(int v) {
    if (!controller_) return;
    const Metavision::timestamp dur = controller_->duration_us();
    if (dur > 0) {
        controller_->seek(v * dur / slider_->maximum());
    }
}

void PlaybackControls::on_time_window_changed(Metavision::timestamp us) {
    // Sync the window field to the FramePipeline's actual accumulation value.
    // Blocked so valueChanged doesn't recurse into set_time_window_us.
    QSignalBlocker b(spd_tw_);
    spd_tw_->setValue(static_cast<int>(us));
}

void PlaybackControls::on_frame_rate_changed(unsigned fps) {
    QSignalBlocker b(spd_fps_);
    spd_fps_->setValue(static_cast<int>(fps));
}

void PlaybackControls::on_fps_limit_changed(unsigned limit) {
    // Update the fps spinbox range. If the current value exceeds the new
    // limit, QSpinBox::setMaximum will clamp it and emit valueChanged,
    // which flows through to the controller and FramePipeline.
    spd_fps_->setMaximum(static_cast<int>(limit));
}

void PlaybackControls::on_multiplier_changed(double m) {
    // Display the actually-applied multiplier, rounded to 6 decimals to
    // match the spinbox precision. The controller's multiplier() may differ
    // slightly from the user-typed value because accumulation_time_us is
    // rounded to an integer (μs).
    const double rounded = std::floor(m * 1e6 + 0.5) / 1e6;
    QSignalBlocker b(spd_mult_);
    spd_mult_->setValue(rounded);
}

void PlaybackControls::on_loop_changed(bool on) {
    chk_loop_->blockSignals(true);
    chk_loop_->setChecked(on);
    chk_loop_->blockSignals(false);
}

QString PlaybackControls::format_time(Metavision::timestamp us) {
    return QStringLiteral("%1 s").arg(us / 1.0e6, 0, 'f', 3);
}

} // namespace gui
