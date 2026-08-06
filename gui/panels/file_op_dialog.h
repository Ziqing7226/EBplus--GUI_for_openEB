// gui/panels/file_op_dialog.h — unified file-operation dialog (Convert/Cut).
//
// Mirrors the ExportDialog UX: source path (prefilled with the currently
// open file), output path with automatic extension, operation parameters,
// integrated progress bar + cancel. Replaces the ad-hoc sequence of
// QFileDialog prompts in FileToolsPanel.

#ifndef GUI_PANELS_FILE_OP_DIALOG_H
#define GUI_PANELS_FILE_OP_DIALOG_H

#include <QDialog>

#include <metavision/sdk/base/utils/timestamp.h>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;

namespace gui {

class FileConverter;

class FileOpDialog : public QDialog {
    Q_OBJECT
public:
    enum class Mode { ConvertHdf5, ConvertCsv, Cut };

    FileOpDialog(Mode mode, FileConverter* converter, QWidget* parent = nullptr);

    /// @brief Pre-fills the source path (e.g. the currently open file).
    void set_source(const QString& path);

private slots:
    void on_browse_source();
    void on_browse_output();
    void on_start();
    void on_cancel();
    void on_completed(const QString& out);
    void on_failed(const QString& msg);
    void on_progress(double r);

private:
    bool validate(QString& err) const;
    void set_running(bool on);

    Mode mode_;
    FileConverter* converter_;
    bool running_{false};

    QLineEdit* edt_source_{nullptr};
    QLineEdit* edt_output_{nullptr};
    QPushButton* btn_browse_src_{nullptr};
    QPushButton* btn_browse_dst_{nullptr};
    // Cut-mode fields (hidden in Convert modes).
    QLabel* lbl_start_{nullptr};
    QLabel* lbl_end_{nullptr};
    QDoubleSpinBox* spn_start_us_{nullptr};
    QDoubleSpinBox* spn_end_us_{nullptr};
    QLabel* lbl_duration_hint_{nullptr};
    // Convert-CSV warning.
    QLabel* lbl_csv_warn_{nullptr};

    QProgressBar* progress_{nullptr};
    QLabel* lbl_status_{nullptr};
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_cancel_{nullptr};
    QPushButton* btn_close_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_FILE_OP_DIALOG_H
