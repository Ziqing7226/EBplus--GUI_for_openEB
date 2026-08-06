// gui/recorder/record_dialog.cpp

#include "record_dialog.h"

#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStandardPaths>

namespace gui {

RecordDialog::RecordDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Start Recording"));

    auto* form = new QFormLayout(this);

    auto* row = new QHBoxLayout;
    edt_output_ = new QLineEdit(this);
    // Timestamped default so consecutive recordings don't collide.
    const QString dir =
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation) +
        QStringLiteral("/EBplus/recordings");
    edt_output_->setText(
        dir + QStringLiteral("/rec_") +
        QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")) +
        QStringLiteral(".raw"));
    btn_browse_ = new QPushButton(tr("Browse..."), this);
    row->addWidget(edt_output_);
    row->addWidget(btn_browse_);
    form->addRow(tr("Output:"), row);

    chk_biases_ = new QCheckBox(tr("Save biases alongside (.bias)"), this);
    chk_biases_->setChecked(true);
    chk_biases_->setToolTip(
        tr("Stores the camera's current bias configuration next to the RAW file "
           "so the recording is reproducible (best-effort, like Metavision Viewer)."));
    form->addRow(QString(), chk_biases_);

    lbl_status_ = new QLabel(tr("Recording starts the live camera's RAW event log."), this);
    lbl_status_->setWordWrap(true);
    lbl_status_->setProperty("class", "hint");
    form->addRow(lbl_status_);

    auto* btn_row = new QHBoxLayout;
    btn_start_ = new QPushButton(tr("Start"), this);
    btn_close_ = new QPushButton(tr("Close"), this);
    btn_row->addWidget(btn_start_);
    btn_row->addWidget(btn_close_);
    form->addRow(btn_row);

    connect(btn_browse_, &QPushButton::clicked, this, &RecordDialog::on_browse);
    connect(btn_start_, &QPushButton::clicked, this, &RecordDialog::on_start);
    connect(btn_close_, &QPushButton::clicked, this, &QDialog::reject);
}

void RecordDialog::on_browse() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Record to file"), edt_output_->text(),
        tr("RAW files (*.raw);;All files (*)"));
    if (!path.isEmpty()) edt_output_->setText(path);
}

void RecordDialog::on_start() {
    QString path = edt_output_->text();
    if (path.isEmpty()) {
        lbl_status_->setText(tr("An output path is required."));
        return;
    }
    // Ensure the .raw extension is present so downstream tools and the SDK
    // can identify the file format.
    if (!path.endsWith(".raw", Qt::CaseInsensitive)) path += ".raw";
    edt_output_->setText(path);
    // Create the output directory if needed (the timestamped default lives
    // in ~/Documents/EBplus/recordings which may not exist yet).
    QDir().mkpath(QFileInfo(path).absolutePath());
    emit start_recording(path, chk_biases_->isChecked());
    accept();
}

} // namespace gui
