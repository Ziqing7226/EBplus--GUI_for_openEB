// gui/panels/information_panel.h — sensor metadata display.

#ifndef GUI_PANELS_INFORMATION_PANEL_H
#define GUI_PANELS_INFORMATION_PANEL_H

#include <QWidget>
#include "../app/camera_controller.h"
#include "abstract_panel.h"

class QLabel;
class QFormLayout;

namespace gui {

class InformationPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit InformationPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("information"); }
    QString panel_title() const override { return tr("Information"); }
    QString panel_group() const override { return QStringLiteral("Camera"); }

    public slots:
    void set_info(const SensorInfo& info);
    void clear();

private:
    QLabel* value_model_{nullptr};
    QLabel* value_resolution_{nullptr};
    QLabel* value_serial_{nullptr};
    QLabel* value_integrator_{nullptr};
    QLabel* value_plugin_{nullptr};
    QLabel* value_encoding_{nullptr};
    QLabel* value_firmware_{nullptr};
    QLabel* value_source_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_INFORMATION_PANEL_H
