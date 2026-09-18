// gui/widgets/imu_window.cpp — see imu_window.h.

#include "imu_window.h"

#include <QTimer>
#include <QVBoxLayout>

#include "app/camera_controller.h"

namespace gui {

ImuWindow::ImuWindow(CameraController* controller, QWidget* parent)
    : QWidget(parent, Qt::Window), controller_(controller) {
    setWindowTitle(tr("IMU Stream"));
    setAttribute(Qt::WA_DeleteOnClose);
    setMinimumSize(320, 220);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(10);

    QFont mono(QStringLiteral("Monospace"));
    mono.setStyleHint(QFont::TypeWriter);
    mono.setPointSize(11);

    // Accelerometer (g) — one label per axis triple, monospace grid.
    accel_label_ = new QLabel(this);
    accel_label_->setFont(mono);
    gyro_label_ = new QLabel(this);
    gyro_label_->setFont(mono);
    temp_label_ = new QLabel(this);
    temp_label_->setFont(mono);
    status_label_ = new QLabel(this);
    status_label_->setFont(mono);

    layout->addWidget(accel_label_);
    layout->addWidget(gyro_label_);
    layout->addWidget(temp_label_);
    layout->addSpacing(6);
    layout->addWidget(status_label_);
    layout->addStretch(1);

    // 30 Hz pull — the controller aggregates samples from the USB thread;
    // the window only reads the latest state.
    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &ImuWindow::refresh);
    timer_->start();
    rate_clock_.start();
    refresh();
}

void ImuWindow::refresh() {
    const auto sample = controller_->latest_imu();
    const long count = controller_->imu_sample_count();

    accel_label_->setText(tr("Accelerometer (g)\n  X: %1   Y: %2   Z: %3")
                              .arg(sample.accel_x, 8, 'f', 3)
                              .arg(sample.accel_y, 8, 'f', 3)
                              .arg(sample.accel_z, 8, 'f', 3));
    gyro_label_->setText(tr("Gyroscope (°/s)\n  X: %1   Y: %2   Z: %3")
                             .arg(sample.gyro_x, 8, 'f', 2)
                             .arg(sample.gyro_y, 8, 'f', 2)
                             .arg(sample.gyro_z, 8, 'f', 2));
    temp_label_->setText(tr("Temperature: %1 °C").arg(sample.temperature, 6, 'f', 2));

    // Exponential-smoothed sample rate from the counter delta.
    const double elapsed_s = rate_clock_.restart() / 1000.0;
    if (elapsed_s > 0.05) {
        const double inst = static_cast<double>(count - last_count_) / elapsed_s;
        smoothed_rate_ = smoothed_rate_ > 0 ? (0.7 * smoothed_rate_ + 0.3 * inst) : inst;
    }
    last_count_ = count;

    if (count == 0) {
        status_label_->setText(tr("Waiting for samples…\n(Stream runs only while the camera streams)"));
    } else {
        status_label_->setText(tr("Samples: %1   Rate: %2 Hz")
                                   .arg(count)
                                   .arg(smoothed_rate_, 5, 'f', 1));
    }
}

void ImuWindow::closeEvent(QCloseEvent* event) {
    emit window_closed();
    QWidget::closeEvent(event);
}

} // namespace gui
