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
    /// Phase 2 capability: the IMU row appears only for sources that
    /// provide the inivation IMU stream. Hiding also unchecks it.
    void set_imu_available(bool available);
    /// Programmatic check state (no toggled signal re-emission).
    void set_imu_checked(bool on);

signals:
    void refresh_requested();
    void connect_first_requested();
    void connect_serial_requested(const QString& serial);
    void disconnect_requested();
    void self_test_requested();
    void imu_stream_toggled(bool on);

private:
    QListWidget* list_{nullptr};
    QPushButton* btn_refresh_{nullptr};
    QPushButton* btn_connect_first_{nullptr};
    QPushButton* btn_connect_selected_{nullptr};
    QPushButton* btn_disconnect_{nullptr};
    QPushButton* btn_self_test_{nullptr};
    QWidget* imu_row_{nullptr};
    QCheckBox* imu_check_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_DEVICES_PANEL_H
