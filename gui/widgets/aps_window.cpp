// gui/widgets/aps_window.cpp — see aps_window.h.

#include "aps_window.h"

#include <QImage>
#include <QPixmap>
#include <QTimer>
#include <QVBoxLayout>

#include "app/camera_controller.h"

namespace gui {

ApsWindow::ApsWindow(CameraController* controller, QWidget* parent)
    : QWidget(parent, Qt::Window), controller_(controller) {
    setWindowTitle(tr("APS Frames"));
    setAttribute(Qt::WA_DeleteOnClose);
    setMinimumSize(360, 300);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);

    // Grayscale preview, scaled to fit while preserving aspect ratio.
    image_label_ = new QLabel(this);
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setMinimumSize(340, 240);
    image_label_->setStyleSheet(QStringLiteral("background: black;"));
    layout->addWidget(image_label_, 1);

    status_label_ = new QLabel(this);
    layout->addWidget(status_label_);

    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &ApsWindow::refresh);
    timer_->start();
    rate_clock_.start();
    refresh();
}

void ApsWindow::refresh() {
    const auto frame = controller_->latest_aps_frame();
    const long count = controller_->aps_frame_count();

    if (frame.valid && !frame.image.empty()) {
        // The decoded image is CV_8UC1 grayscale — wrap without copying and
        // let Qt scale to the label size (smooth for the preview only; the
        // underlying data is untouched).
        const QImage img(frame.image.data, frame.image.cols, frame.image.rows,
            static_cast<qsizetype>(frame.image.step), QImage::Format_Grayscale8);
        image_label_->setPixmap(QPixmap::fromImage(img).scaled(
            image_label_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }

    const double elapsed_s = rate_clock_.restart() / 1000.0;
    if (elapsed_s > 0.05) {
        const double inst = static_cast<double>(count - last_count_) / elapsed_s;
        smoothed_rate_ = smoothed_rate_ > 0 ? (0.7 * smoothed_rate_ + 0.3 * inst) : inst;
    }
    last_count_ = count;

    if (count == 0) {
        status_label_->setText(tr("Waiting for frames…\n(Stream runs only while the camera streams)"));
    } else {
        status_label_->setText(tr("Frames: %1   Rate: %2 Hz   %3×%4")
                                   .arg(count)
                                   .arg(smoothed_rate_, 5, 'f', 1)
                                   .arg(frame.image.cols)
                                   .arg(frame.image.rows));
    }
}

void ApsWindow::closeEvent(QCloseEvent* event) {
    emit window_closed();
    QWidget::closeEvent(event);
}

} // namespace gui
