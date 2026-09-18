// gui/widgets/imu_window.h — standalone live readout for the inivation IMU
// stream (Phase 2). Algorithm-window style: an independent top-level window
// fed from CameraController's IMU telemetry via a 30 Hz pull timer (no
// per-sample signals across threads).

#ifndef GUI_WIDGETS_IMU_WINDOW_H
#define GUI_WIDGETS_IMU_WINDOW_H

#include <QElapsedTimer>
#include <QLabel>
#include <QWidget>

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

private:
    void refresh();

    CameraController* controller_;
    QTimer* timer_;
    QLabel* accel_label_;
    QLabel* gyro_label_;
    QLabel* temp_label_;
    QLabel* status_label_;
    QElapsedTimer rate_clock_;
    long last_count_{0};
    double smoothed_rate_{0};
};

} // namespace gui

#endif // GUI_WIDGETS_IMU_WINDOW_H
