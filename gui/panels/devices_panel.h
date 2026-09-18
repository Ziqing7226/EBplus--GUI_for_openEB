// gui/panels/devices_panel.h — live camera discovery + connect controls.

#ifndef GUI_PANELS_DEVICES_PANEL_H
#define GUI_PANELS_DEVICES_PANEL_H

#include <QWidget>
#include <QString>

#include "abstract_panel.h"

class QCheckBox;
class QListWidget;
class QPushButton;
class QWidget;

namespace gui {

class DevicesPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit DevicesPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("devices"); }
    QString panel_title() const override { return tr("Devices"); }
    QString panel_group() const override { return QStringLiteral("Camera"); }

    public slots:
    void refresh_sources(const std::vector<std::pair<QString, QString>>& sources);
    void set_connected(bool connected);
    /// Phase 2/3 capability rows: appear only for sources that provide the
    /// inivation IMU / APS streams. Hiding also unchecks.
    void set_imu_available(bool available);
    void set_imu_checked(bool on);
    void set_aps_available(bool available);
    void set_aps_checked(bool on);

signals:
    void refresh_requested();
    void connect_first_requested();
    void connect_serial_requested(const QString& serial);
    void disconnect_requested();
    void self_test_requested();
    void imu_stream_toggled(bool on);
    void aps_stream_toggled(bool on);

private:
    QListWidget* list_{nullptr};
    QPushButton* btn_refresh_{nullptr};
    QPushButton* btn_connect_first_{nullptr};
    QPushButton* btn_connect_selected_{nullptr};
    QPushButton* btn_disconnect_{nullptr};
    QPushButton* btn_self_test_{nullptr};
    QWidget* imu_row_{nullptr};
    QCheckBox* imu_check_{nullptr};
    QWidget* aps_row_{nullptr};
    QCheckBox* aps_check_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_DEVICES_PANEL_H
