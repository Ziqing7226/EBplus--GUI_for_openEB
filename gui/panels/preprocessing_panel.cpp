// gui/panels/preprocessing_panel.cpp

#include "preprocessing_panel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QVBoxLayout>

#include "algo_bridge/filter_chain.h"
#include "app/camera_controller.h"

namespace gui {

PreprocessingPanel::PreprocessingPanel(QWidget* parent) : AbstractPanel(parent) {
    build_ui();
    setEnabled(false);
}

void PreprocessingPanel::build_ui() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    group_ = new QGroupBox(tr("Preprocessing"), this);
    auto* form = new QFormLayout(group_);
    auto* hint = new QLabel(tr("Note: Noise/Hot-pixel filters are under the Algorithm menu."), group_);
    hint->setWordWrap(true);
    form->addRow(hint);

    auto make_row = [&](const QString& stage, const QString& label) {
        auto* cb = new QCheckBox(label, group_);
        enables_[stage] = cb;
        form->addRow(cb);
        connect(cb, &QCheckBox::toggled, this, [this, stage](bool) { apply_stage(stage); });
    };
    auto make_enum_row = [&](const QString& stage, const QString& label,
                             const QStringList& items, const QString& param) {
        auto* cb = new QCheckBox(label, group_);
        auto* cmb = new QComboBox(group_);
        cmb->addItems(items);
        cmb->setEnabled(false);
        combos_[stage + "|" + param] = cmb;
        auto* w = new QWidget;
        auto* hl = new QHBoxLayout(w);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->addWidget(cb);
        hl->addWidget(cmb, 1);
        form->addRow(w);
        enables_[stage] = cb;
        connect(cb, &QCheckBox::toggled, this, [this, stage, cmb](bool on) {
            cmb->setEnabled(on);
            apply_stage(stage);
        });
        connect(cmb, &QComboBox::currentTextChanged, this, [this, stage](const QString&) {
            apply_stage(stage);
        });
    };

    make_enum_row("polarity_filter", tr("Polarity Filter"),
                  {"OFF (0)", "ON (1)"}, "polarity");
    make_row("polarity_invert", tr("Polarity Invert"));
    make_row("flip_x", tr("Flip X"));
    make_row("flip_y", tr("Flip Y"));
    make_enum_row("rotate", tr("Rotate"),
                  {"0", "90", "180", "270"}, "rotation");
    make_row("transpose", tr("Transpose"));
    // Rescale needs two float inputs; expose as separate rows so the row
    // width fits the narrower sidebar (§13.3 — splitting wide rows).
    {
        auto* cb = new QCheckBox(tr("Rescale"), group_);
        form->addRow(cb);
        auto* sx = new QDoubleSpinBox(group_);
        sx->setRange(0.01, 10.0);
        sx->setValue(1.0);
        sx->setEnabled(false);
        auto* sy = new QDoubleSpinBox(group_);
        sy->setRange(0.01, 10.0);
        sy->setValue(1.0);
        sy->setEnabled(false);
        form->addRow(tr("  Scale X"), sx);
        form->addRow(tr("  Scale Y"), sy);
        enables_["rescale"] = cb;
        // Store pointers for apply_stage (typed correctly, no casts).
        double_spins_["rescale|scale_width"] = sx;
        double_spins_["rescale|scale_height"] = sy;
        connect(cb, &QCheckBox::toggled, this, [this, sx, sy](bool on) {
            sx->setEnabled(on); sy->setEnabled(on);
            apply_stage("rescale");
        });
        connect(sx, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            apply_stage("rescale");
        });
        connect(sy, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
            apply_stage("rescale");
        });
    }
    // ROI filter — each coordinate on its own row so the row width fits
    // the narrower sidebar (§13.3 — splitting wide rows).
    {
        auto* cb = new QCheckBox(tr("ROI Filter"), group_);
        form->addRow(cb);
        auto* x0 = new QSpinBox(group_); x0->setRange(0, 100000);
        auto* y0 = new QSpinBox(group_); y0->setRange(0, 100000);
        auto* x1 = new QSpinBox(group_); x1->setRange(0, 100000); x1->setValue(1279);
        auto* y1 = new QSpinBox(group_); y1->setRange(0, 100000); y1->setValue(719);
        for (auto* s : {x0, y0, x1, y1}) s->setEnabled(false);
        form->addRow(tr("  X0"), x0);
        form->addRow(tr("  Y0"), y0);
        form->addRow(tr("  X1"), x1);
        form->addRow(tr("  Y1"), y1);
        enables_["roi_filter"] = cb;
        spins_["roi_filter|x0"] = x0;
        spins_["roi_filter|y0"] = y0;
        spins_["roi_filter|x1"] = x1;
        spins_["roi_filter|y1"] = y1;
        connect(cb, &QCheckBox::toggled, this, [this, x0, y0, x1, y1](bool on) {
            x0->setEnabled(on); y0->setEnabled(on); x1->setEnabled(on); y1->setEnabled(on);
            apply_stage("roi_filter");
        });
        for (auto* s : {x0, y0, x1, y1}) {
            connect(s, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) {
                apply_stage("roi_filter");
            });
        }
    }

    outer->addWidget(group_);
}

void PreprocessingPanel::apply_stage(const QString& stage) {
    if (!camera_) return;
    auto* chain = camera_->filter_chain();
    auto* cb = enables_.value(stage);
    if (!cb) return;
    // Use the thread-safe locked wrappers. Calling chain->stage()->set_enabled
    // / set_param directly would race the SDK thread's process() call which
    // reads the same fields under chain_mutex().
    chain->set_stage_enabled(stage.toStdString(), cb->isChecked());
    if (stage == "polarity_filter") {
        auto* cmb = combos_.value("polarity_filter|polarity");
        if (cmb) chain->set_stage_param(stage.toStdString(), "polarity",
                                        std::to_string(cmb->currentIndex()));
    } else if (stage == "rotate") {
        auto* cmb = combos_.value("rotate|rotation");
        if (cmb) chain->set_stage_param(stage.toStdString(), "rotation",
                                        cmb->currentText().toStdString());
    } else if (stage == "rescale") {
        auto* sx = double_spins_.value("rescale|scale_width");
        auto* sy = double_spins_.value("rescale|scale_height");
        if (sx) chain->set_stage_param(stage.toStdString(), "scale_width",
                                       std::to_string(sx->value()));
        if (sy) chain->set_stage_param(stage.toStdString(), "scale_height",
                                       std::to_string(sy->value()));
    } else if (stage == "roi_filter") {
        auto* x0 = spins_.value("roi_filter|x0");
        auto* y0 = spins_.value("roi_filter|y0");
        auto* x1 = spins_.value("roi_filter|x1");
        auto* y1 = spins_.value("roi_filter|y1");
        if (x0) chain->set_stage_param(stage.toStdString(), "x0", std::to_string(x0->value()));
        if (y0) chain->set_stage_param(stage.toStdString(), "y0", std::to_string(y0->value()));
        if (x1) chain->set_stage_param(stage.toStdString(), "x1", std::to_string(x1->value()));
        if (y1) chain->set_stage_param(stage.toStdString(), "y1", std::to_string(y1->value()));
    }
    if (cb->isChecked()) {
        emit info_message(tr("Preprocess: %1 on").arg(stage));
    }
    emit stage_toggled(stage, cb->isChecked());
}

void PreprocessingPanel::on_camera_connected(CameraController* controller) {
    camera_ = controller;
    setEnabled(true);
    // Re-apply any stages that were enabled before the camera connected
    // (e.g. via the Preprocess menu while disconnected). Without this the
    // checkbox would show checked but the FilterChain stage stays disabled.
    for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
        if (it.value() && it.value()->isChecked()) {
            apply_stage(it.key());
        }
    }
}

void PreprocessingPanel::on_camera_disconnected() {
    // Disable every stage in the FilterChain BEFORE nullifying camera_.
    // CameraController is a long-lived MainWindow member (not destroyed on
    // disconnect) and CameraController::teardown() does not reset filter_chain_,
    // so previously-enabled stages would keep filtering events from the next
    // camera with no UI indication. apply_stage() early-returns once
    // camera_ is null, so we must drive the chain directly here.
    if (camera_) {
        auto* chain = camera_->filter_chain();
        for (auto it = enables_.constBegin(); it != enables_.constEnd(); ++it) {
            chain->set_stage_enabled(it.key().toStdString(), false);
        }
    }
    camera_ = nullptr;
    setEnabled(false);
    for (auto* cb : enables_) {
        cb->blockSignals(true);
        cb->setChecked(false);
        cb->blockSignals(false);
    }
}

void PreprocessingPanel::set_stage_enabled(const QString& stage, bool on) {
    auto* cb = enables_.value(stage);
    if (!cb) return;
    // Block the toggled signal so we don't re-enter apply_stage from the
    // checkbox; apply manually so the FilterChain is still updated. The
    // stage_toggled signal emitted by apply_stage syncs the menu (whose
    // actions use 'triggered', so setChecked won't re-enter this path).
    QSignalBlocker b(cb);
    cb->setChecked(on);
    apply_stage(stage);
}

bool PreprocessingPanel::is_stage_enabled(const QString& stage) const {
    auto* cb = enables_.value(stage);
    return cb ? cb->isChecked() : false;
}

} // namespace gui
