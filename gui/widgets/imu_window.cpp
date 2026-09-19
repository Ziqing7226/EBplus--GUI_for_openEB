// gui/widgets/imu_window.cpp — see imu_window.h.
//
// Visualization: the camera drawn as a cuboid whose orientation follows the
// gyro-integrated IMU pose (quaternion kinematics, body-rate integration),
// rendered with a perspective projection in plain QPainter — plus numeric
// accel/gyro/temperature readouts and the sample counter / rate line.
// Fresh implementation (see the IP note in the header).

#include "imu_window.h"

#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

#include "app/camera_controller.h"

namespace gui {
namespace {

constexpr double kDeg2Rad = M_PI / 180.0;
/// Perspective focal distance (world units).
constexpr double kFocal = 3.5;
/// Camera-body half extents (a cuboid, not a cube — camera-shaped).
constexpr double kHalfX = 0.8, kHalfY = 1.2, kHalfZ = 0.5;

struct Q4 {
    double w, x, y, z;
};

Q4 qnormalized(const Q4& q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12) return {1, 0, 0, 0};
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

Q4 qmul(const Q4& a, const Q4& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

/// Rotates (x, y, z) by q: v' = q * v * q*.
void qrotate(const Q4& q, double& x, double& y, double& z) {
    const double tx = 2.0 * (q.y * z - q.z * y);
    const double ty = 2.0 * (q.z * x - q.x * z);
    const double tz = 2.0 * (q.x * y - q.y * x);
    const double nx = x + q.w * tx + (q.y * tz - q.z * ty);
    const double ny = y + q.w * ty + (q.z * tx - q.x * tz);
    const double nz = z + q.w * tz + (q.x * ty - q.y * tx);
    x = nx;
    y = ny;
    z = nz;
}

} // namespace

ImuWindow::ImuWindow(CameraController* controller, QWidget* parent)
    : QWidget(parent, Qt::Window), controller_(controller) {
    setWindowTitle(tr("IMU Stream"));
    setAttribute(Qt::WA_DeleteOnClose);
    setMinimumSize(520, 620);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    status_label_ = new QLabel(this);
    layout->addWidget(status_label_);
    layout->addStretch(1);  // the pose panel below is hand-painted

    // 30 Hz pull: drain new samples from the controller ring (thread-safe),
    // integrate the pose, repaint.
    timer_ = new QTimer(this);
    timer_->setInterval(33);
    connect(timer_, &QTimer::timeout, this, &ImuWindow::refresh);
    timer_->start();
    rate_clock_.start();
    refresh();
}

void ImuWindow::integrate_imu(const davis::ImuSample& s) {
    if (prev_t_ < 0) {  // first sample establishes the time base
        prev_t_ = s.t;
        return;
    }
    const double dt = static_cast<double>(s.t - prev_t_) / 1e6;
    prev_t_ = s.t;
    if (dt <= 0 || dt > 0.2) return;  // restart/gap: hold the pose

    const double wx = s.gyro_x * kDeg2Rad;
    const double wy = s.gyro_y * kDeg2Rad;
    const double wz = s.gyro_z * kDeg2Rad;
    const double norm = std::sqrt(wx * wx + wy * wy + wz * wz);
    if (norm < 1e-9) return;
    const double half = dt * norm / 2.0;
    const Q4 dq{std::cos(half), wx / norm * std::sin(half),
                wy / norm * std::sin(half), wz / norm * std::sin(half)};

    const Q4 q = qnormalized(qmul(qnormalized({qw_, qx_, qy_, qz_}), dq));
    qw_ = q.w;
    qx_ = q.x;
    qy_ = q.y;
    qz_ = q.z;
}

void ImuWindow::refresh() {
    const auto fresh = controller_->drain_imu(imu_cursor_);
    for (const auto& s : fresh) integrate_imu(s);

    const long count = controller_->imu_sample_count();
    const double elapsed_s = rate_clock_.restart() / 1000.0;
    rate_accum_time_ += elapsed_s;
    rate_accum_events_ += count - last_count_;
    last_count_ = count;
    if (rate_accum_time_ >= 0.5) {
        const double inst =
            static_cast<double>(rate_accum_events_) / rate_accum_time_;
        smoothed_rate_ =
            smoothed_rate_ > 0 ? (0.7 * smoothed_rate_ + 0.3 * inst) : inst;
        rate_accum_events_ = 0;
        rate_accum_time_ = 0;
    }

    if (count == 0) {
        status_label_->setText(
            tr("Waiting for samples…\n(Stream runs only while the camera streams)"));
    } else {
        status_label_->setText(tr("Samples: %1   Rate: %2 Hz")
                                   .arg(count)
                                   .arg(smoothed_rate_, 5, 'f', 1));
    }
    update();
}

void ImuWindow::draw_pose(QPainter& p, const QRectF& r) {
    p.fillRect(r, QColor(16, 18, 22));
    p.setPen(QColor(70, 70, 78));
    p.drawRect(r);

    const Q4 q{qw_, qx_, qy_, qz_};

    // Cuboid corners (body frame), rotated into the world frame.
    double corners[8][3];
    const double hx[2] = {-kHalfX, kHalfX};
    const double hy[2] = {-kHalfY, kHalfY};
    const double hz[2] = {-kHalfZ, kHalfZ};
    for (int i = 0; i < 8; ++i) {
        double x = hx[i & 1], y = hy[(i >> 1) & 1], z = hz[(i >> 2) & 1];
        qrotate(q, x, y, z);
        corners[i][0] = x;
        corners[i][1] = y;
        corners[i][2] = z;
    }

    // Faces (corner indices, body-frame outward normals).
    struct Face {
        int idx[4];
        double nx, ny, nz;
    };
    const Face faces[6] = {
        {{4, 5, 7, 6}, 0, 0, 1},   // +Z (top)
        {{0, 1, 3, 2}, 0, 0, -1},  // -Z (bottom)
        {{2, 3, 7, 6}, 0, 1, 0},   // +Y (front)
        {{0, 1, 5, 4}, 0, -1, 0},  // -Y (back)
        {{1, 3, 7, 5}, 1, 0, 0},   // +X
        {{0, 2, 6, 4}, -1, 0, 0},  // -X
    };

    // Perspective projection: viewer at y = -kFocal looking toward +y.
    const qreal base = std::min(r.width(), r.height()) / 2.2;
    const qreal cx = r.center().x(), cy = r.center().y();
    auto project = [&](const double c[3], qreal* sx, qreal* sy) {
        const double persp = kFocal / (kFocal + c[1] + kHalfY);
        *sx = cx + c[0] * base * persp;
        *sy = cy - c[2] * base * persp;
    };

    // Painter's algorithm: farthest face first (viewer looks toward +y, so
    // larger mean-y = nearer).
    int order[6] = {0, 1, 2, 3, 4, 5};
    double face_depth[6];
    for (int f = 0; f < 6; ++f) {
        face_depth[f] = (corners[faces[f].idx[0]][1] + corners[faces[f].idx[1]][1] +
                         corners[faces[f].idx[2]][1] + corners[faces[f].idx[3]][1]) /
                        4.0;
    }
    for (int a = 0; a < 5; ++a)
        for (int b = a + 1; b < 6; ++b)
            if (face_depth[order[b]] > face_depth[order[a]])
                std::swap(order[a], order[b]);

    const double light[3] = {0.3, -0.8, 0.5};
    for (int fi = 0; fi < 6; ++fi) {
        const int f = order[fi];
        QPointF poly[4];
        for (int k = 0; k < 4; ++k) {
            qreal sx, sy;
            const double c[3] = {corners[faces[f].idx[k]][0],
                                 corners[faces[f].idx[k]][1],
                                 corners[faces[f].idx[k]][2]};
            project(c, &sx, &sy);
            poly[k] = QPointF(sx, sy);
        }
        double nx = faces[f].nx, ny = faces[f].ny, nz = faces[f].nz;
        qrotate(q, nx, ny, nz);
        const double shade =
            0.4 + 0.45 * std::max(0.0, nx * light[0] + ny * light[1] + nz * light[2]);
        p.setPen(QPen(QColor(25, 40, 60), 1));
        p.setBrush(QColor(static_cast<int>(40 * shade),
                          static_cast<int>(70 * shade),
                          static_cast<int>(120 * shade)));
        p.drawPolygon(poly, 4);
    }

    // Body axes triad from the cuboid center (X red, Y green, Z blue).
    const double axes[3][3] = {{2.0, 0, 0}, {0, 2.0, 0}, {0, 0, 2.0}};
    const QColor axis_colors[3] = {QColor(255, 80, 80), QColor(80, 220, 120),
                                   QColor(110, 160, 255)};
    const char* axis_labels[3] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; ++a) {
        double x = axes[a][0], y = axes[a][1], z = axes[a][2];
        qrotate(q, x, y, z);
        const qreal persp = kFocal / (kFocal + y + kHalfY);
        p.setPen(QPen(axis_colors[a], 2));
        p.drawLine(QPointF(cx, cy),
                   QPointF(cx + x * base * persp, cy - z * base * persp));
        p.setPen(axis_colors[a]);
        p.drawText(QPointF(cx + x * base * persp + 4, cy - z * base * persp - 4),
                   QString(axis_labels[a]));
    }

    // Numeric readout (latest sample).
    const auto latest = controller_->latest_imu();
    p.setPen(QColor(200, 200, 205));
    p.drawText(r.adjusted(8, 6, -8, 0), Qt::AlignLeft | Qt::AlignTop,
               QStringLiteral("Acc %1, %2, %3 g   Gyro %4, %5, %6 dps   %7 °C")
                   .arg(latest.accel_x, 0, 'f', 2)
                   .arg(latest.accel_y, 0, 'f', 2)
                   .arg(latest.accel_z, 0, 'f', 2)
                   .arg(latest.gyro_x, 0, 'f', 1)
                   .arg(latest.gyro_y, 0, 'f', 1)
                   .arg(latest.gyro_z, 0, 'f', 1)
                   .arg(latest.temperature, 0, 'f', 1));
}

void ImuWindow::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(12, 12, 14));

    // The status label occupies the top strip (layout-managed); everything
    // below is hand-painted.
    const qreal paint_top = status_label_->geometry().bottom() + 6.0;
    const QRectF pose(rect().left() + 8, paint_top, rect().width() - 16,
                      std::max(120.0, rect().bottom() - 10.0 - paint_top));
    draw_pose(p, pose);
}

void ImuWindow::closeEvent(QCloseEvent* event) {
    emit window_closed();
    QWidget::closeEvent(event);
}

} // namespace gui
