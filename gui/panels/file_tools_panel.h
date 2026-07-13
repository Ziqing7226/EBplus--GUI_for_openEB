// gui/panels/file_tools_panel.h — offline file conversion tools UI (design §1.5.7).
//
// Buttons for: Convert to HDF5, Convert to CSV, File Cutter, File Info.
// Delegates all work to FileConverter.

#ifndef GUI_PANELS_FILE_TOOLS_PANEL_H
#define GUI_PANELS_FILE_TOOLS_PANEL_H

#include <QWidget>

#include "abstract_panel.h"

class QProgressBar;
class QLabel;
class QPushButton;

namespace gui {

class FileConverter;

class FileToolsPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit FileToolsPanel(FileConverter* converter, QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("file_tools"); }
    QString panel_title() const override { return tr("File Tools"); }
    QString panel_group() const override { return QStringLiteral("Tools"); }

    private slots:
    void on_convert_hdf5();
    void on_convert_csv();
    void on_cutter();
    void on_info();
    void on_completed(const QString& out);
    void on_failed(const QString& msg);

private:
    FileConverter* converter_;
    QPushButton* btn_hdf5_{nullptr};
    QPushButton* btn_csv_{nullptr};
    QPushButton* btn_cut_{nullptr};
    QPushButton* btn_info_{nullptr};
    QProgressBar* progress_{nullptr};
    QLabel* lbl_status_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_FILE_TOOLS_PANEL_H
