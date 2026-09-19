// gui/widgets/imu_window.h — DV-style live IMU visualization (Phase 2
// visualization pass): three stacked scrolling strip charts (accelerometer,
// gyroscope, temperature) drawn with plain QPainter.
//
// IP note: this is a FRESH implementation of a generic scientific
// time-series plot — no code or assets are taken from iniVation's DV GUI
// (whose custom license we deliberately do not rely on). Only the generic
// concept (rolling multi-axis curves) is shared, which is not protectable
// expression.

#ifndef GUI_WIDGETS_IMU_WINDOW_H
#define GUI_WIDGETS_IMU_WINDOW_H

#include <QElapsedTimer>
#include <QLabel>
#include <QWidget>

#include <deque>

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

    CameraController* controller_;
    QTimer* timer_;
    QLabel* status_label_;
    /// Scrolling history for the strip charts (time-ordered, trimmed to the
    /// plot window; cleared on a stream restart).
    std::deque<davis::ImuSample> history_;
    std::int64_t imu_cursor_{0};
    long last_count_{0};
    double smoothed_rate_{0};
    QElapsedTimer rate_clock_;
};

} // namespace gui

#endif // GUI_WIDGETS_IMU_WINDOW_H
