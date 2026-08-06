// gui/panels/file_tools_panel.cpp

#include "file_tools_panel.h"

#include <QCursor>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "app/file_converter.h"

namespace gui {

namespace {
// True when src and dst resolve to the same file (audit §六-E2: converting/
// cutting onto the source would overwrite the very file being read).
bool same_file(const QString& src, const QString& dst) {
    const QFileInfo s(src);
    const QString s_canon = s.canonicalFilePath();
    if (s_canon.isEmpty()) return false;
    const QFileInfo d(dst);
    QString d_canon = d.canonicalFilePath();
    if (d_canon.isEmpty()) {
        // dst doesn't exist yet — canonicalize its directory instead.
        const QString cdir = d.dir().canonicalPath();
        if (cdir.isEmpty()) return false;
        d_canon = cdir + QLatin1Char('/') + d.fileName();
    }
    return s_canon == d_canon;
}
} // namespace

FileToolsPanel::FileToolsPanel(FileConverter* converter, QWidget* parent)
    : AbstractPanel(parent), converter_(converter) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto* gb = new QGroupBox(tr("File Tools"), this);
    auto* lay = new QVBoxLayout(gb);

    btn_hdf5_ = new QPushButton(tr("Convert to HDF5..."), gb);
    btn_csv_  = new QPushButton(tr("Convert to CSV..."), gb);
    btn_cut_  = new QPushButton(tr("File Cutter..."), gb);
    btn_info_ = new QPushButton(tr("File Info..."), gb);
    progress_ = new QProgressBar(gb);
    progress_->setRange(0, 100);
    // Hide the progress bar when idle so it doesn't look like an empty box
    // (§14.6). It's shown only during conversion/cutting operations.
    progress_->setVisible(false);
    lbl_status_ = new QLabel(tr("Ready."), gb);

    // Recording + Export controls (moved from File menu / toolbar — §14.5).
    btn_record_ = new QPushButton(tr("Start Recording..."), gb);
    btn_record_->setShortcut(QKeySequence("R"));
    btn_stop_   = new QPushButton(tr("Stop Recording"), gb);
    btn_export_ = new QPushButton(tr("Export..."), gb);
    btn_stop_->setEnabled(false);
    btn_record_->setEnabled(false);
    btn_export_->setEnabled(false);

    lay->addWidget(btn_hdf5_);
    lay->addWidget(btn_csv_);
    lay->addWidget(btn_cut_);
    lay->addWidget(btn_info_);
    lay->addWidget(progress_);
    lay->addWidget(lbl_status_);
    lay->addSpacing(8);
    lay->addWidget(btn_record_);
    lay->addWidget(btn_stop_);
    lay->addWidget(btn_export_);

    outer->addWidget(gb);

    connect(btn_hdf5_, &QPushButton::clicked, this, &FileToolsPanel::on_convert_hdf5);
    connect(btn_csv_,  &QPushButton::clicked, this, &FileToolsPanel::on_convert_csv);
    connect(btn_cut_,  &QPushButton::clicked, this, &FileToolsPanel::on_cutter);
    connect(btn_info_, &QPushButton::clicked, this, &FileToolsPanel::on_info);
    connect(btn_record_, &QPushButton::clicked, this, &FileToolsPanel::record_start_requested);
    connect(btn_stop_,   &QPushButton::clicked, this, &FileToolsPanel::record_stop_requested);
    connect(btn_export_, &QPushButton::clicked, this, &FileToolsPanel::export_requested);

    if (converter_) {
        connect(converter_, &FileConverter::completed, this, &FileToolsPanel::on_completed);
        connect(converter_, &FileConverter::failed, this, &FileToolsPanel::on_failed);
        connect(converter_, &FileConverter::progress, this, [this](double r) {
            progress_->setValue(static_cast<int>(r * 100));
        });
    }
}

void FileToolsPanel::set_buttons_enabled(bool enabled) {
    btn_hdf5_->setEnabled(enabled);
    btn_csv_->setEnabled(enabled);
    btn_cut_->setEnabled(enabled);
    btn_info_->setEnabled(enabled);
}

void FileToolsPanel::set_record_enabled(bool enabled) {
    btn_record_->setEnabled(enabled);
}

void FileToolsPanel::set_stop_enabled(bool enabled) {
    btn_stop_->setEnabled(enabled);
}

void FileToolsPanel::set_export_enabled(bool enabled) {
    btn_export_->setEnabled(enabled);
}

void FileToolsPanel::on_convert_hdf5() {
    if (!converter_) return;
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (src.isEmpty()) return;
    const QString dst = QFileDialog::getSaveFileName(
        this, tr("Output HDF5"), QString(), tr("HDF5 (*.h5);;All files (*)"));
    if (dst.isEmpty()) return;
    // Ensure .h5 extension so HDF5EventFileWriter can identify the format.
    QString final_dst = dst;
    if (!final_dst.endsWith(".h5", Qt::CaseInsensitive))
        final_dst += ".h5";
    if (same_file(src, final_dst)) {
        QMessageBox::warning(this, tr("Convert to HDF5"),
            tr("Output path must differ from the source file (it would be overwritten)."));
        return;
    }
    progress_->setVisible(true);
    progress_->setValue(0);
    lbl_status_->setText(tr("Converting to HDF5..."));
    set_buttons_enabled(false);
    converter_->convert(src, final_dst, FileConverter::Format::HDF5);
}

void FileToolsPanel::on_convert_csv() {
    if (!converter_) return;
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (src.isEmpty()) return;
    const QString dst = QFileDialog::getSaveFileName(
        this, tr("Output CSV"), QString(), tr("CSV (*.csv);;All files (*)"));
    if (dst.isEmpty()) return;
    QString final_dst = dst;
    if (!final_dst.endsWith(".csv", Qt::CaseInsensitive))
        final_dst += ".csv";
    if (same_file(src, final_dst)) {
        QMessageBox::warning(this, tr("Convert to CSV"),
            tr("Output path must differ from the source file (it would be overwritten)."));
        return;
    }
    progress_->setVisible(true);
    progress_->setValue(0);
    lbl_status_->setText(tr("Converting to CSV..."));
    set_buttons_enabled(false);
    converter_->convert(src, final_dst, FileConverter::Format::CSV);
}

void FileToolsPanel::on_cutter() {
    if (!converter_) return;
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (src.isEmpty()) return;
    // Simple cut dialog: ask start/end in seconds.
    QDialog dlg(this);
    dlg.setWindowTitle(tr("File Cutter"));
    auto* form = new QFormLayout(&dlg);
    auto* spStart = new QDoubleSpinBox(&dlg); spStart->setRange(0, 1e6); spStart->setSuffix(" s");
    auto* spEnd = new QDoubleSpinBox(&dlg); spEnd->setRange(0, 1e6); spEnd->setSuffix(" s");
    form->addRow(tr("Start:"), spStart);
    form->addRow(tr("End:"), spEnd);
    auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(bb);
    connect(bb, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(bb, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    if (dlg.exec() != QDialog::Accepted) return;
    if (spEnd->value() <= spStart->value()) {
        QMessageBox::warning(this, tr("File Cutter"),
            tr("End time must be greater than start time."));
        return;
    }
    const QString dst = QFileDialog::getSaveFileName(
        this, tr("Output RAW"), QString(), tr("RAW (*.raw);;All files (*)"));
    if (dst.isEmpty()) return;
    QString final_dst = dst;
    if (!final_dst.endsWith(".raw", Qt::CaseInsensitive))
        final_dst += ".raw";
    if (same_file(src, final_dst)) {
        QMessageBox::warning(this, tr("File Cutter"),
            tr("Output path must differ from the source file (it would be overwritten)."));
        return;
    }
    const auto start_us = static_cast<Metavision::timestamp>(spStart->value() * 1e6);
    const auto end_us = static_cast<Metavision::timestamp>(spEnd->value() * 1e6);
    progress_->setVisible(true);
    progress_->setValue(0);
    lbl_status_->setText(tr("Cutting..."));
    set_buttons_enabled(false);
    converter_->cut(src, final_dst, start_us, end_us);
}

void FileToolsPanel::on_info() {
    if (!converter_) return;
    const QString src = QFileDialog::getOpenFileName(
        this, tr("Source file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (src.isEmpty()) return;
    // info() opens the file synchronously on the GUI thread — for a large
    // file without a .tmp_index this can take seconds. Show a busy cursor
    // and disable the button so the stall is visible and can't be
    // re-triggered (audit §六-U6; not moving threads this round).
    btn_info_->setEnabled(false);
    QGuiApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QString text;
    QString err;
    try {
        const auto fi = converter_->info(src);
        text = tr("Path: %1\nIntegrator: %2\nSerial: %3\nPlugin: %4\nEncoding: %5\n"
                  "Geometry: %6 x %7\nDuration: %8 s")
                   .arg(fi.path, fi.integrator, fi.serial, fi.plugin, fi.encoding)
                   .arg(fi.width).arg(fi.height)
                   .arg(fi.duration_us / 1.0e6, 0, 'f', 3);
    } catch (const std::exception& e) {
        err = tr("Failed to read file info:\n%1").arg(QString::fromUtf8(e.what()));
    } catch (...) {
        err = tr("Failed to read file info.");
    }
    QGuiApplication::restoreOverrideCursor();
    btn_info_->setEnabled(true);
    if (err.isEmpty()) {
        QMessageBox::information(this, tr("File Info"), text);
    } else {
        QMessageBox::warning(this, tr("File Info"), err);
    }
}

void FileToolsPanel::on_completed(const QString& out) {
    progress_->setValue(100);
    progress_->setVisible(false);
    lbl_status_->setText(tr("Done: %1").arg(out));
    set_buttons_enabled(true);
}

void FileToolsPanel::on_failed(const QString& msg) {
    progress_->reset();
    progress_->setVisible(false);
    lbl_status_->setText(tr("Failed: %1").arg(msg));
    set_buttons_enabled(true);
}

} // namespace gui
