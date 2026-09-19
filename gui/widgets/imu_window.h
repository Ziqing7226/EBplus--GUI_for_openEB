// gui/widgets/imu_window.h — DV-style live IMU visualization: the camera is
// drawn as a cuboid whose 3D pose follows the gyro-integrated IMU output,
// with numeric accel/gyro/temperature readouts.
//
// IP note: fresh implementation. iniVation's dv-gui ships a custom
// (non-standard) license — none of its code is used or referenced; only the
// generic concept (a 3D box posed by the IMU) is shared, which is not
// protectable expression. The pose math below is written from scratch on
// standard quaternion kinematics.

#ifndef GUI_WIDGETS_IMU_WINDOW_H
#define GUI_WIDGETS_IMU_WINDOW_H

#include <QElapsedTimer>
#include <QLabel>
#include <QWidget>

#include "davis/imu_types.h"

class QTimer;

namespace gui {

class CameraController;

class ImuWindow : public QWidget {
    Q_OBJECT
public:
    explicit ImuWindow(CameraController* controller, QWidget* parent = nullptr);

signals:
    /// Emitted when the user closes the window (the host unchecks the
    /// Devices-panel toggle and stops the stream).
    void window_closed();

protected:
    void closeEvent(QCloseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private:
    void refresh();
    /// Integrates one gyro sample into the pose quaternion (body-rate
    /// kinematics). A timestamp jump (stream restart) resets the pose.
    void integrate_imu(const davis::ImuSample& s);
    /// Draws the camera cuboid + body axes in the current pose.
    void draw_pose(QPainter& p, const QRectF& r);

    CameraController* controller_;
    QTimer* timer_;
    QLabel* status_label_;

    /// Camera pose as a unit quaternion (world <- body rotation), integrated
    /// from the gyro rates; identity = axes aligned with the world frame.
    double qw_{1}, qx_{0}, qy_{0}, qz_{0};
    std::int64_t prev_t_{-1};

    std::int64_t imu_cursor_{0};
    long last_count_{0};
    long rate_accum_events_{0};
    double rate_accum_time_{0};
    double smoothed_rate_{0};
    QElapsedTimer rate_clock_;
};

} // namespace gui

#endif // GUI_WIDGETS_IMU_WINDOW_H
