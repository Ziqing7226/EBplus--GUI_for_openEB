// gui/panels/roi_panel.cpp — see header (Phase 2.6 debug D-6 rewrite).

#include "roi_panel.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>

#include "app/camera_controller.h"

namespace gui {

RoiPanel::RoiPanel(QWidget* parent) : AbstractPanel(parent) {
    auto* form = new QFormLayout(this);
    form->setContentsMargins(8, 8, 8, 8);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Unified ROI (Phase 2.6 debug D-6): a single enable checkbox + a
    // settings button opening the modal UnifiedRoiDialog. Works on live
    // cameras (hardware I_ROI) and file playback (software crop) alike.
    enable_cb_ = new QCheckBox(tr("Enable ROI"), this);
    enable_cb_->setToolTip(
        tr("Single unified ROI: hardware ROI on a live camera, software crop "
           "on file playback. Rect/mode/drag settings live in the dialog."));
    form->addRow(enable_cb_);

    settings_btn_ = new QPushButton(tr("ROI Settings..."), this);
    form->addRow(QString(), settings_btn_);

    auto* hint = new QLabel(tr("Applies to display, recording and algorithm "
                               "inputs. Use the dialog for rect / RONI / drag."),
                            this);
    hint->setWordWrap(true);
    hint->setProperty("class", "hint");
    form->addRow(QString(), hint);

    connect(enable_cb_, &QCheckBox::toggled, this,
            [this](bool on) { emit roi_enable_toggled(on); });
    connect(settings_btn_, &QPushButton::clicked, this,
            [this]() { emit roi_settings_requested(); });
}

void RoiPanel::on_camera_connected(CameraController* /*controller*/) {
    // Nothing to populate: the ROI state lives in CameraController and the
    // checkbox is synced via set_roi_enabled (roi_state_changed).
}

void RoiPanel::on_camera_disconnected() {
    QSignalBlocker b(enable_cb_);
    enable_cb_->setChecked(false);
}

void RoiPanel::set_roi_enabled(bool enabled) {
    QSignalBlocker b(enable_cb_);
    enable_cb_->setChecked(enabled);
}

} // namespace gui
