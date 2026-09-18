// gui/widgets/aps_window.h — standalone live preview for the DAVIS APS
// frame stream (Phase 3). Algorithm-window style: an independent top-level
// window pulling the latest frame from CameraController at 30 Hz.

#ifndef GUI_WIDGETS_APS_WINDOW_H
#define GUI_WIDGETS_APS_WINDOW_H

#include <QElapsedTimer>
#include <QLabel>
#include <QWidget>

class QTimer;

namespace gui {

class CameraController;

class ApsWindow : public QWidget {
    Q_OBJECT
public:
    explicit ApsWindow(CameraController* controller, QWidget* parent = nullptr);

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
    QLabel* image_label_;
    QLabel* status_label_;
    QElapsedTimer rate_clock_;
    long last_count_{0};
    double smoothed_rate_{0};
};

} // namespace gui

#endif // GUI_WIDGETS_APS_WINDOW_H
