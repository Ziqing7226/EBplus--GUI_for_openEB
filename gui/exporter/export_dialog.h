// gui/exporter/export_dialog.h — modal dialog for configuring an export job.

#ifndef GUI_EXPORTER_EXPORT_DIALOG_H
#define GUI_EXPORTER_EXPORT_DIALOG_H

#include <QDialog>
#include <functional>
#include "exporter_controller.h"

class QLineEdit;
class QComboBox;
class QSpinBox;
class QCheckBox;
class QProgressBar;
class QPushButton;
class QLabel;

namespace gui {

class ExportDialog : public QDialog {
    Q_OBJECT
public:
    explicit ExportDialog(ExporterController* controller, QWidget* parent = nullptr);

    /// @brief Pre-fills the source path (e.g. the currently open file).
    void set_source(const QString& path);

    /// @brief Provides a known total duration (us) for a source path, or 0
    /// when unknown. Set by MainWindow so that exporting the currently-open
    /// (fully buffered) file skips the blocking OSC duration query, which
    /// would otherwise freeze progress reporting (see query_duration_async).
    void set_duration_provider(
        std::function<Metavision::timestamp(const QString&)> provider) {
        duration_provider_ = std::move(provider);
    }

private slots:
    void on_browse_source();
    void on_browse_output();
    void on_format_changed(int idx);
    void on_start();
    void on_completed(const QString& out);
    void on_failed(const QString& msg);
    void on_progress(double r);

private:
    ExporterController* controller_;
    std::function<Metavision::timestamp(const QString&)> duration_provider_;
    QLineEdit* edt_source_{nullptr};
    QLineEdit* edt_output_{nullptr};
    QComboBox* cmb_format_{nullptr};
    QSpinBox* spn_fps_{nullptr};
    QSpinBox* spn_accum_{nullptr};
    QSpinBox* spn_quality_{nullptr};
    QCheckBox* chk_color_{nullptr};
    QProgressBar* progress_{nullptr};
    QPushButton* btn_start_{nullptr};
    QPushButton* btn_cancel_{nullptr};
    QLabel* lbl_status_{nullptr};
};

} // namespace gui

#endif // GUI_EXPORTER_EXPORT_DIALOG_H
