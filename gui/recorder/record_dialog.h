// gui/recorder/record_dialog.h — recording setup dialog (ExportDialog-style).
//
// Collects the output path (auto .raw suffix, timestamped default) and the
// "save biases alongside" option, then asks MainWindow to start the actual
// recording (the recorder itself lives in RecorderController).

#ifndef GUI_RECORDER_RECORD_DIALOG_H
#define GUI_RECORDER_RECORD_DIALOG_H

#include <QDialog>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace gui {

class RecordDialog : public QDialog {
    Q_OBJECT
public:
    explicit RecordDialog(QWidget* parent = nullptr);

signals:
    /// @brief Emitted when the user confirms: record to @p path, optionally
    /// saving the current biases alongside as <base>.bias.
    void start_recording(const QString& path, bool save_biases);

private slots:
    void on_browse();
    void on_start();

private:
    QLineEdit* edt_output_{nullptr};
    QPushButton* btn_browse_{nullptr};
    QCheckBox* chk_biases_{nullptr};
    QLabel* lbl_status_{nullptr};
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_close_{nullptr};
};

} // namespace gui

#endif // GUI_RECORDER_RECORD_DIALOG_H
