// gui/widgets/imu_window.cpp — see imu_window.h.
//
// Visualization: three stacked strip charts over a rolling 5 s window —
// accelerometer (g), gyroscope (°/s), temperature (°C) — per-axis colors
// (X red / Y green / Z blue), auto-scaled ranges with a zero line and
// in-plot legends carrying the latest values. Everything is drawn with
// plain QPainter (fresh implementation; see the IP note in the header).

#include "imu_window.h"

#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "app/camera_controller.h"

namespace gui {
namespace {

constexpr int kPlotWindowMs = 5000;    // Rolling window shown in the charts.
constexpr qreal kAccelHeadroom = 0.3;  // Accel range headroom (g).
constexpr qreal kGyroHeadroom = 25.0;  // Gyro minimum range headroom (deg/s).

const QColor kAxisColors[3] = {
    QColor(0xff, 0x55, 0x55),  // X — red
    QColor(0x50, 0xfa, 0x7b),  // Y — green
    QColor(0x8b, 0xe9, 0xfd),  // Z — blue
};
const QColor kPanelColor(20, 20, 24);
struct PlotSpec {
    const char* title;
    const char* unit;
};
const PlotSpec kPlots[3] = {
    {"Accelerometer", "g"},
    {"Gyroscope", "deg/s"},
    {"Temperature", "C"},
};
const QColor kFrameColor(70, 70, 78);
const QColor kTextColor(200, 200, 205);

int series_count(int plot) {
    return plot == 2 ? 1 : 3;  // temperature is a single curve
}

qreal axis_value(const davis::ImuSample& s, int plot, int axis) {
    switch (plot) {
        case 0: return axis == 0 ? s.accel_x : axis == 1 ? s.accel_y : s.accel_z;
        case 1: return axis == 0 ? s.gyro_x : axis == 1 ? s.gyro_y : s.gyro_z;
        default: return s.temperature;
    }
}

QString axis_name(int plot, int axis) {
    if (plot == 2) return QString();
    return axis == 0 ? QStringLiteral("X")
                     : axis == 1 ? QStringLiteral("Y") : QStringLiteral("Z");
}

} // namespace

ImuWindow::ImuWindow(CameraController* controller, QWidget* parent)
    : QWidget(parent, Qt::Window), controller_(controller) {
    setWindowTitle(tr("IMU Stream"));
    setAttribute(Qt::WA_DeleteOnClose);
    setMinimumSize(480, 560);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    status_label_ = new QLabel(this);
    layout->addWidget(status_label_);

    // 30 Hz pull: drain new samples from the controller ring (thread-safe)
    // and repaint the strip charts.
    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &ImuWindow::refresh);
    timer_->start();
    rate_clock_.start();
    refresh();
}

void ImuWindow::refresh() {
    auto fresh = controller_->drain_imu(imu_cursor_);
    if (!fresh.empty()) {
        // A stream restart rebases the timestamps — start a fresh plot.
        if (!history_.empty() && fresh.front().t + 2000000 < history_.back().t) {
            history_.clear();
        }
        for (const auto& s : fresh) history_.push_back(s);
        const auto newest = history_.back().t;
        while (!history_.empty() && history_.front().t + kPlotWindowMs + 500 < newest) {
            history_.pop_front();
        }
    }

    const long count = controller_->imu_sample_count();
    const double elapsed_s = rate_clock_.restart() / 1000.0;
    if (elapsed_s > 0.05) {
        const double inst = static_cast<double>(count - last_count_) / elapsed_s;
        smoothed_rate_ = smoothed_rate_ > 0 ? (0.7 * smoothed_rate_ + 0.3 * inst) : inst;
    }
    last_count_ = count;

    if (count == 0) {
        status_label_->setText(
            tr("Waiting for samples…\n(Stream runs only while the camera streams)"));
    } else {
        status_label_->setText(tr("Samples: %1   Rate: %2 Hz")
                                   .arg(count)
                                   .arg(smoothed_rate_, 5, 'f', 1));
    }
    update();  // repaint the strip charts
}

void ImuWindow::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(12, 12, 14));

    const QRectF charts = rect().adjusted(8, 8, -8, -(status_label_->height() + 10));
    const qreal plot_h = charts.height() / 3.0;

    const qint64 t_max = history_.empty() ? 0 : history_.back().t;
    const qint64 t_min = t_max - kPlotWindowMs;
    // Timestamp → horizontal fraction within the rolling window.
    auto x_frac = [&](qint64 t) {
        return (t_max == t_min)
                   ? 1.0
                   : static_cast<qreal>(t - t_min) / static_cast<qreal>(t_max - t_min);
    };

    for (int plot = 0; plot < 3; ++plot) {
        const QRectF r(charts.left(), charts.top() + plot * plot_h,
                       charts.width(), plot_h - 6);
        p.fillRect(r, kPanelColor);
        p.setPen(kFrameColor);
        p.drawRect(r);

        // Value range over the visible samples.
        qreal lo = 0, hi = 0;
        bool have = false;
        for (const auto& s : history_) {
            for (int a = 0; a < series_count(plot); ++a) {
                const qreal v = axis_value(s, plot, a);
                lo = have ? std::min(lo, v) : v;
                hi = have ? std::max(hi, v) : v;
                have = true;
            }
        }
        if (plot == 0) {
            // Accel: fixed headroom so a still sensor keeps a usable scale.
            lo = std::min(lo, -kAccelHeadroom);
            hi = std::max(hi, kAccelHeadroom);
        } else if (plot == 1) {
            // Gyro: minimum ± range so a still sensor doesn't zoom into
            // quantization noise.
            lo = std::min(lo, -kGyroHeadroom);
            hi = std::max(hi, kGyroHeadroom);
        } else if (have) {
            lo -= 1.0;  // temperature: ±1 °C breathing room
            hi += 1.0;
        }
        if (!have) { lo = -1; hi = 1; }

        // Zero line.
        if (lo < 0 && hi > 0) {
            const qreal zero_y = r.bottom() - (-lo / (hi - lo)) * r.height();
            p.setPen(QPen(QColor(90, 90, 96), 1, Qt::DashLine));
            p.drawLine(QPointF(r.left(), zero_y), QPointF(r.right(), zero_y));
        }

        // Curves.
        if (!history_.empty()) {
            for (int a = 0; a < series_count(plot); ++a) {
                QPolygonF line;
                for (const auto& s : history_) {
                    const qreal v = axis_value(s, plot, a);
                    const qreal y_frac = (v - lo) / (hi - lo);
                    line.append(QPointF(r.left() + x_frac(s.t) * r.width(),
                                        r.bottom() - y_frac * r.height()));
                }
                p.setPen(QPen(a < 3 ? kAxisColors[a] : QColor(255, 165, 0), 1.6));
                p.drawPolyline(line);
            }
        }

        // Legend: title + latest per-axis values.
        const davis::ImuSample latest =
            history_.empty() ? davis::ImuSample{} : history_.back();
        QString legend = QStringLiteral("%1 (%2)")
                             .arg(QString::fromUtf8(kPlots[plot].title),
                                  QString::fromUtf8(kPlots[plot].unit));
        if (plot == 2) {
            legend += QStringLiteral("   %1 °C").arg(latest.temperature, 0, 'f', 1);
        } else {
            for (int a = 0; a < 3; ++a) {
                legend += QStringLiteral("  %1 %2")
                              .arg(axis_name(plot, a))
                              .arg(axis_value(latest, plot, a), 0, 'f', 2);
            }
        }
        p.setPen(kTextColor);
        p.drawText(r.adjusted(6, 4, -6, -4), Qt::AlignLeft | Qt::AlignTop, legend);
    }

    if (controller_->imu_sample_count() == 0) {
        p.setPen(kTextColor);
        p.drawText(rect(), Qt::AlignCenter, tr("Waiting for samples…"));
    }
}

void ImuWindow::closeEvent(QCloseEvent* event) {
    emit window_closed();
    QWidget::closeEvent(event);
}

} // namespace gui
