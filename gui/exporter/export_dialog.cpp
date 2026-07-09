// gui/exporter/export_dialog.cpp

#include "export_dialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace gui {

ExportDialog::ExportDialog(ExporterController* controller, QWidget* parent)
    : QDialog(parent), controller_(controller) {
    setWindowTitle(tr("Export Recording"));
    setMinimumWidth(460);

    auto* form = new QFormLayout();
    edt_source_ = new QLineEdit(this);
    auto* btn_src = new QPushButton(tr("..."), this);
    auto* hl = new QHBoxLayout;
    hl->addWidget(edt_source_);
    hl->addWidget(btn_src);
    form->addRow(tr("Source file:"), hl);
    connect(btn_src, &QPushButton::clicked, this, &ExportDialog::on_browse_source);

    edt_output_ = new QLineEdit(this);
    auto* btn_out = new QPushButton(tr("..."), this);
    auto* hl2 = new QHBoxLayout;
    hl2->addWidget(edt_output_);
    hl2->addWidget(btn_out);
    form->addRow(tr("Output file:"), hl2);
    connect(btn_out, &QPushButton::clicked, this, &ExportDialog::on_browse_output);

    cmb_format_ = new QComboBox(this);
    cmb_format_->addItem(tr("HDF5 (.h5)"), static_cast<int>(ExportParams::Format::HDF5));
    cmb_format_->addItem(tr("AVI (.avi)"),  static_cast<int>(ExportParams::Format::AVI));
    form->addRow(tr("Format:"), cmb_format_);
    connect(cmb_format_, qOverload<int>(&QComboBox::currentIndexChanged),
            this, &ExportDialog::on_format_changed);

    spn_fps_ = new QSpinBox(this);
    spn_fps_->setRange(1, 120);
    spn_fps_->setValue(30);
    form->addRow(tr("FPS (AVI):"), spn_fps_);

    spn_accum_ = new QDoubleSpinBox(this);
    spn_accum_->setRange(1.0, 1000.0);
    spn_accum_->setValue(33.0);
    spn_accum_->setSuffix(" ms");
    form->addRow(tr("Accumulation (AVI):"), spn_accum_);

    spn_quality_ = new QSpinBox(this);
    spn_quality_->setRange(1, 100);
    spn_quality_->setValue(90);
    // quality only selects the codec (>=50 -> H.264, <50 -> MJPG); it is not
    // forwarded to the encoder as a quantization parameter. The label and
    // tooltip make this explicit so users don't expect fine-grained quality
    // control that the CvVideoRecorder backend doesn't provide.
    spn_quality_->setToolTip(tr("Selects the AVI codec: values >= 50 use H.264, "
                                "values < 50 use MJPG. Does not control encoder "
                                "quantization — CvVideoRecorder exposes no "
                                "quality parameter."));
    form->addRow(tr("Codec select (AVI, ≥50=H.264):"), spn_quality_);

    chk_color_ = new QCheckBox(tr("Color"), this);
    chk_color_->setChecked(true);
    form->addRow(QString(), chk_color_);

    progress_ = new QProgressBar(this);
    progress_->setRange(0, 100);
    progress_->setValue(0);

    lbl_status_ = new QLabel(tr("Ready."), this);

    btn_start_ = new QPushButton(tr("Start"), this);
    btn_cancel_ = new QPushButton(tr("Cancel"), this);
    btn_cancel_->setEnabled(false);
    auto* btn_close = new QPushButton(tr("Close"), this);
    auto* btnbox = new QDialogButtonBox;
    btnbox->addButton(btn_start_, QDialogButtonBox::ActionRole);
    btnbox->addButton(btn_cancel_, QDialogButtonBox::ActionRole);
    btnbox->addButton(btn_close, QDialogButtonBox::RejectRole);
    connect(btn_start_, &QPushButton::clicked, this, &ExportDialog::on_start);
    connect(btn_cancel_, &QPushButton::clicked, this, [this]() {
        controller_->cancel();
        lbl_status_->setText(tr("Cancelling..."));
        btn_cancel_->setEnabled(false);
    });
    connect(btn_close, &QPushButton::clicked, this, &QDialog::reject);

    auto* lay = new QVBoxLayout(this);
    lay->addLayout(form);
    lay->addWidget(progress_);
    lay->addWidget(lbl_status_);
    lay->addWidget(btnbox);

    on_format_changed(0);

    connect(controller_, &ExporterController::completed, this, &ExportDialog::on_completed);
    connect(controller_, &ExporterController::failed, this, &ExportDialog::on_failed);
    connect(controller_, &ExporterController::progress, this, &ExportDialog::on_progress);
}

void ExportDialog::set_source(const QString& path) {
    edt_source_->setText(path);
}

void ExportDialog::on_browse_source() {
    const QString p = QFileDialog::getOpenFileName(
        this, tr("Select source"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (!p.isEmpty()) edt_source_->setText(p);
}

void ExportDialog::on_browse_output() {
    const QString filt = (cmb_format_->currentData().toInt() == static_cast<int>(ExportParams::Format::HDF5))
                             ? tr("HDF5 (*.h5);;All files (*)")
                             : tr("AVI (*.avi);;All files (*)");
    const QString p = QFileDialog::getSaveFileName(this, tr("Select output"), QString(), filt);
    if (!p.isEmpty()) edt_output_->setText(p);
}

void ExportDialog::on_format_changed(int idx) {
    const bool avi = (cmb_format_->itemData(idx).toInt() == static_cast<int>(ExportParams::Format::AVI));
    spn_fps_->setEnabled(avi);
    spn_accum_->setEnabled(avi);
    spn_quality_->setEnabled(avi);
    chk_color_->setEnabled(avi);
}

void ExportDialog::on_start() {
    if (edt_source_->text().isEmpty() || edt_output_->text().isEmpty()) {
        lbl_status_->setText(tr("Source and output paths are required."));
        return;
    }
    ExportParams p;
    p.source_path = edt_source_->text();
    p.output_path = edt_output_->text();
    p.format = static_cast<ExportParams::Format>(cmb_format_->currentData().toInt());
    p.fps = spn_fps_->value();
    p.accumulation_ms = spn_accum_->value();
    p.quality = spn_quality_->value();
    p.color = chk_color_->isChecked();
    progress_->setValue(0);
    lbl_status_->setText(tr("Exporting..."));
    btn_start_->setEnabled(false);
    btn_cancel_->setEnabled(true);
    // start() returns false if an export is already running — without this
    // check the dialog would stay in the "Exporting…" state forever because
    // no completed/failed signal would ever arrive.
    if (!controller_->start(p)) {
        btn_start_->setEnabled(true);
        btn_cancel_->setEnabled(false);
        lbl_status_->setText(tr("An export is already running. Wait for it to finish or cancel it."));
    }
}

void ExportDialog::on_completed(const QString& out) {
    btn_start_->setEnabled(true);
    btn_cancel_->setEnabled(false);
    lbl_status_->setText(tr("Done: %1").arg(out));
}

void ExportDialog::on_failed(const QString& msg) {
    btn_start_->setEnabled(true);
    btn_cancel_->setEnabled(false);
    lbl_status_->setText(tr("Failed: %1").arg(msg));
}

void ExportDialog::on_progress(double r) {
    progress_->setValue(static_cast<int>(r * 100));
}

} // namespace gui
