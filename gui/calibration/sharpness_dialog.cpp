// gui/calibration/sharpness_dialog.cpp

#include "sharpness_dialog.h"

#include <algorithm>

#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <opencv2/core.hpp>

#include "app/camera_controller.h"
#include "calibration/sharpness_metrics.h"

namespace gui {

namespace {
// Poll interval — 10 Hz. The timer only re-builds the count image and
// recomputes metrics; event accumulation happens continuously in the SDK
// callback thread, so a slower timer never loses events.
constexpr int kPollIntervalMs = 100;
} // namespace

// ---------------------------------------------------------------------------
// SharpnessChart
// ---------------------------------------------------------------------------

SharpnessChart::SharpnessChart(const QString& title, const QColor& color,
                               QWidget* parent)
    : QWidget(parent), title_(title), color_(color) {
    setMinimumSize(300, 120);
    setAutoFillBackground(true);
    // Dark background like an oscilloscope screen.
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(24, 24, 28));
    setPalette(pal);
}

void SharpnessChart::set_format(char fmt, int precision) {
    fmt_ = fmt;
    precision_ = precision;
}

void SharpnessChart::add_value(double value) {
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    samples_.push_back({now, value});
    // Prune samples older than the window.
    const qint64 cutoff = now - kWindowMs;
    while (!samples_.empty() && samples_.front().t_ms < cutoff) {
        samples_.erase(samples_.begin());
    }
    update();
}

void SharpnessChart::clear() {
    samples_.clear();
    update();
}

void SharpnessChart::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF r = rect().adjusted(0, 0, -1, -1);

    // Background.
    p.fillRect(r, QColor(24, 24, 28));

    // Chart area with margins for labels.
    const int ml = 8, mt = 18, mr = 8, mb = 8;
    const QRectF chart_r(r.left() + ml, r.top() + mt,
                         r.width() - ml - mr, r.height() - mt - mb);

    // Title (top-left).
    p.setPen(QColor(180, 180, 185));
    QFont tf = p.font();
    tf.setPointSize(9);
    p.setFont(tf);
    p.drawText(QRectF(chart_r.left(), r.top(), chart_r.width() * 0.5, mt),
               Qt::AlignLeft | Qt::AlignVCenter, title_);

    // Grid — 4 horizontal lines.
    p.setPen(QPen(QColor(50, 50, 55), 1, Qt::DotLine));
    for (int i = 1; i < 4; ++i) {
        const qreal y = chart_r.top() + chart_r.height() * i / 4.0;
        p.drawLine(QPointF(chart_r.left(), y), QPointF(chart_r.right(), y));
    }

    if (samples_.empty()) {
        p.setPen(QColor(130, 130, 135));
        QFont f = p.font();
        f.setPointSize(11);
        p.setFont(f);
        p.drawText(chart_r, Qt::AlignCenter, tr("Waiting for data..."));
        return;
    }

    // Adaptive Y-axis (audit §9.3 S6): both metrics are >= 0, so pin the
    // bottom at 0 and set the ceiling to the 95th percentile of the visible
    // samples + 10 % headroom. The percentile (not the max) keeps single
    // spikes from collapsing the scale.
    std::vector<double> values;
    values.reserve(samples_.size());
    for (const auto& s : samples_) values.push_back(s.value);
    std::sort(values.begin(), values.end());
    const double q95 = values[static_cast<std::size_t>(
        0.95 * static_cast<double>(values.size() - 1))];
    const double y_min = 0.0;
    const double y_max = (q95 > 0.0) ? q95 * 1.1 : 1.0;

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const qint64 t_min = now - kWindowMs;

    // Map (t, value) -> (x, y) in chart coordinates.
    auto map_pt = [&](const Sample& s) -> QPointF {
        const double x_frac = static_cast<double>(s.t_ms - t_min) /
                              static_cast<double>(kWindowMs);
        double y_frac = (s.value - y_min) / (y_max - y_min);
        if (y_frac > 1.0) y_frac = 1.0;  // clip spikes above the ceiling
        const qreal x = chart_r.left() + x_frac * chart_r.width();
        const qreal y = chart_r.bottom() - y_frac * chart_r.height();
        return QPointF(x, y);
    };

    // Draw the polyline.
    p.setPen(QPen(color_, 2));
    QPainterPath path;
    bool first = true;
    for (const auto& s : samples_) {
        const QPointF pt = map_pt(s);
        if (first) { path.moveTo(pt); first = false; }
        else       { path.lineTo(pt); }
    }
    p.drawPath(path);

    // Draw sample dots.
    p.setBrush(color_);
    p.setPen(Qt::NoPen);
    for (const auto& s : samples_) {
        p.drawEllipse(map_pt(s), 2.5, 2.5);
    }

    // Current value (top-right).
    const double current = samples_.back().value;
    p.setPen(QColor(235, 235, 240));
    QFont f = p.font();
    f.setPointSize(11);
    f.setBold(true);
    p.setFont(f);
    p.drawText(QRectF(chart_r.right() - chart_r.width() * 0.5, r.top(),
                      chart_r.width() * 0.5, mt),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(current, fmt_, precision_));
}

// ---------------------------------------------------------------------------
// SharpnessDialog
// ---------------------------------------------------------------------------

SharpnessDialog::SharpnessDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(tr("Sharpness Meter"));
    setMinimumSize(360, 420);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);

    // Top row: accumulation window selector + event rate (reference only).
    auto* top_row = new QHBoxLayout();
    top_row->addWidget(new QLabel(tr("Window:"), this));
    window_combo_ = new QComboBox(this);
    window_combo_->addItem(tr("50 ms"), 50);
    window_combo_->addItem(tr("100 ms"), 100);
    window_combo_->addItem(tr("200 ms"), 200);
    window_combo_->setCurrentIndex(1);  // default 100 ms
    connect(window_combo_, QOverload<int>::of(&QComboBox::activated),
            this, &SharpnessDialog::reset_accumulation);
    top_row->addWidget(window_combo_);
    top_row->addStretch(1);
    rate_label_ = new QLabel(tr("Event rate: —"), this);
    top_row->addWidget(rate_label_);
    layout->addLayout(top_row);

    // Big current-value readouts.
    auto* values = new QFormLayout();
    contrast_value_ = new QLabel(tr("—"), this);
    width_value_ = new QLabel(tr("—"), this);
    QFont big = font();
    big.setPointSize(14);
    big.setBold(true);
    contrast_value_->setFont(big);
    width_value_->setFont(big);
    values->addRow(tr("Contrast (higher = sharper):"), contrast_value_);
    values->addRow(tr("Line width px (lower = sharper):"), width_value_);
    layout->addLayout(values);

    contrast_chart_ = new SharpnessChart(tr("Contrast σ²/μ²"),
                                         QColor(76, 200, 130), this);
    contrast_chart_->set_format('f', 1);
    layout->addWidget(contrast_chart_, 1);

    width_chart_ = new SharpnessChart(tr("Mean line width (px)"),
                                      QColor(90, 160, 240), this);
    width_chart_->set_format('f', 2);
    layout->addWidget(width_chart_, 1);

    hint_label_ = new QLabel(
        tr("Focus: smaller line width is better. "
           "Bias tuning: higher contrast is better. "
           "If noise is high, reduce noise first."),
        this);
    hint_label_->setWordWrap(true);
    hint_label_->setAlignment(Qt::AlignCenter);
    hint_label_->setStyleSheet(QStringLiteral("color: gray;"));
    layout->addWidget(hint_label_);

    timer_ = new QTimer(this);
    timer_->setTimerType(Qt::CoarseTimer);
    timer_->setInterval(kPollIntervalMs);
    connect(timer_, &QTimer::timeout, this, &SharpnessDialog::on_tick);
    timer_->start();

    // §11.4-P0-3 B1: start the metrics worker. It sleeps on work_cv_ until
    // on_tick hands it a count image, so the idle cost is negligible.
    worker_ = std::thread([this]() { worker_loop(); });
}

SharpnessDialog::~SharpnessDialog() {
    if (timer_) timer_->stop();
    // Stop the worker FIRST, before touching camera_ / buffer_: on_tick (GUI
    // thread) and worker_loop both touch pending_count_image_ / latest_result_,
    // and worker_loop calls compute_sharpness_metrics which has no GUI deps.
    // After join, no thread touches the worker slots.
    stop_and_join_worker();
    // Note: cd_broadcast is a single boolean shared with the calibration
    // wizard. Like the wizard, this dialog flips it off on teardown without
    // tracking which consumer enabled it — if both tools are open at once,
    // closing one stops the other's broadcast until it is reopened. This is
    // a known, accepted limitation of the shared-flag design.
    if (camera_) {
        disconnect(camera_, &CameraController::cd_events_ready,
                   this, &SharpnessDialog::on_events_ready);
        camera_->set_cd_broadcast(false);
    }
    // Lock once so any in-progress on_events_ready() call (running on the
    // SDK thread via DirectConnection) finishes before buffer_ is destroyed.
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.clear();
}

void SharpnessDialog::worker_loop() {
    // Worker thread: no Qt calls, no GUI object access. Only
    // compute_sharpness_metrics (pure OpenCV) + mutex-protected slot I/O.
    while (!worker_stop_.load(std::memory_order_acquire)) {
        cv::Mat img;
        double win = 0.0;
        {
            std::unique_lock<std::mutex> lk(work_mutex_);
            work_cv_.wait(lk, [this] {
                return worker_stop_.load(std::memory_order_acquire) ||
                       !pending_count_image_.empty();
            });
            if (worker_stop_.load(std::memory_order_acquire)) break;
            // Move the pending image out so the GUI thread can immediately
            // fill the slot again while we compute. If a newer image arrives
            // during computation, it overwrites the (now empty) slot and we
            // pick it up on the next iteration — natural frame dropping.
            img = std::move(pending_count_image_);
            pending_count_image_ = cv::Mat();
            win = pending_window_s_;
        }
        // Heavy O(W×H) computation OUTSIDE the lock — GUI thread can hand
        // the next image without waiting for us.
        const SharpnessMetrics m = compute_sharpness_metrics(img, win);
        {
            std::lock_guard<std::mutex> rlk(result_mutex_);
            latest_result_ = m;
            has_new_result_ = true;
        }
    }
}

void SharpnessDialog::stop_and_join_worker() {
    worker_stop_.store(true, std::memory_order_release);
    work_cv_.notify_one();
    if (worker_.joinable()) worker_.join();
}

void SharpnessDialog::set_camera(CameraController* camera) {
    if (camera_ == camera) return;
    if (camera_) {
        disconnect(camera_, &CameraController::cd_events_ready,
                   this, &SharpnessDialog::on_events_ready);
    }
    camera_ = camera;
    reset_accumulation();
    if (!camera_) return;

    {
        const SensorInfo& info = camera_->sensor_info();
        std::lock_guard<std::mutex> lk(mutex_);
        sensor_width_ = info.width;
        sensor_height_ = info.height;
    }

    // DirectConnection: on_events_ready() runs on the SDK streaming thread,
    // NOT the GUI thread (same rationale as CalibrationEventTap — a queued
    // connection would pile batches into the GUI event queue). The slot only
    // appends under mutex_. UniqueConnection guards against duplicate
    // connections if set_camera() is called repeatedly with the same camera.
    connect(camera_, &CameraController::cd_events_ready,
            this, &SharpnessDialog::on_events_ready,
            static_cast<Qt::ConnectionType>(Qt::DirectConnection |
                                            Qt::UniqueConnection));

    // If the dialog is already visible, start the broadcast immediately;
    // otherwise showEvent() will.
    if (isVisible()) camera_->set_cd_broadcast(true);
}

void SharpnessDialog::set_display(EventDisplayWidget* display) {
    // Kept for source compatibility with callers that have no camera handle.
    // The display frame is never polled anymore (audit §9.3 R3); without a
    // camera the dialog shows a "no data source" placeholder.
    display_ = display;
}

void SharpnessDialog::showEvent(QShowEvent* event) {
    QDialog::showEvent(event);
    // Enable the CD broadcast only while the dialog is shown so the SDK
    // thread pays the per-batch copy cost only when a consumer is listening.
    if (camera_) camera_->set_cd_broadcast(true);
}

void SharpnessDialog::closeEvent(QCloseEvent* event) {
    if (camera_) camera_->set_cd_broadcast(false);
    QDialog::closeEvent(event);
}

int SharpnessDialog::window_ms() const {
    return window_combo_->currentData().toInt();
}

void SharpnessDialog::reset_accumulation() {
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.clear();
}

void SharpnessDialog::show_no_data(const QString& reason) {
    contrast_value_->setText(tr("—"));
    width_value_->setText(tr("—"));
    rate_label_->setText(tr("Event rate: —"));
    hint_label_->setText(reason);
}

void SharpnessDialog::on_events_ready(
    std::shared_ptr<std::vector<Metavision::EventCD>> events) {
    // Runs on the SDK streaming thread (DirectConnection). Mutex only, no GUI.
    if (!events || events->empty()) return;
    std::lock_guard<std::mutex> lk(mutex_);
    buffer_.insert(buffer_.end(), events->begin(), events->end());
    // Cap: if the GUI thread stalls, keep only the most recent events.
    if (buffer_.size() > kMaxBufferEvents) {
        std::vector<Metavision::EventCD> trimmed(
            buffer_.end() - kMaxBufferEvents, buffer_.end());
        buffer_ = std::move(trimmed);
    }
}

void SharpnessDialog::on_tick() {
    if (!camera_) {
        contrast_chart_->clear();
        width_chart_->clear();
        show_no_data(tr("No data source — connect a camera."));
        return;
    }
    hint_label_->setText(
        tr("Focus: smaller line width is better. "
           "Bias tuning: higher contrast is better. "
           "If noise is high, reduce noise first."));

    // Build the count image under the lock (copy-swap pattern: the lock only
    // covers buffer trimming + image construction). Events arrive SDK-sorted
    // by timestamp, so the window is the newest window_ms() of the buffer.
    cv::Mat count_image;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (buffer_.empty()) {
            show_no_data(tr("No events in window — point the camera at a "
                            "textured scene."));
            return;
        }
        const Metavision::timestamp t_end = buffer_.back().t;
        const Metavision::timestamp cutoff =
            t_end - static_cast<Metavision::timestamp>(window_ms()) * 1000;
        auto first = std::lower_bound(
            buffer_.begin(), buffer_.end(), cutoff,
            [](const Metavision::EventCD& e, Metavision::timestamp t) {
                return e.t < t;
            });
        buffer_.erase(buffer_.begin(), first);

        // Sensor dimensions come from SensorInfo; fall back to the max event
        // coordinates if they are unavailable.
        int w = sensor_width_;
        int h = sensor_height_;
        if (w <= 0 || h <= 0) {
            for (const auto& e : buffer_) {
                if (e.x + 1 > w) w = e.x + 1;
                if (e.y + 1 > h) h = e.y + 1;
            }
            if (w <= 0 || h <= 0) return;
        }

        count_image = cv::Mat::zeros(h, w, CV_32F);
        for (const auto& e : buffer_) {
            if (e.x < w && e.y < h) {
                count_image.at<float>(e.y, e.x) += 1.0f;
            }
        }
    }

    // §11.4-P0-3 B1: hand the count image to the worker thread and read back
    // the LATEST computed result. The GUI thread never calls
    // compute_sharpness_metrics — it only does light buffer work + UI updates.
    // UI therefore lags data by ≤ 1 tick (100 ms), which is imperceptible.
    {
        std::lock_guard<std::mutex> lk(work_mutex_);
        pending_count_image_ = std::move(count_image);
        pending_window_s_ = window_ms() / 1000.0;
    }
    work_cv_.notify_one();

    // Read the latest result. has_new_result_ distinguishes "worker hasn't
    // produced anything yet" (first tick after open) from "result unchanged".
    SharpnessMetrics m;
    bool has_result;
    {
        std::lock_guard<std::mutex> rlk(result_mutex_);
        m = latest_result_;
        has_result = has_new_result_;
        has_new_result_ = false;
    }

    if (!has_result) {
        // First tick: worker just received the first count image and hasn't
        // finished yet. Leave the UI at its default ("—" / "Waiting for
        // data...") — the next tick will pick up the result.
        return;
    }
    if (!m.valid) {
        show_no_data(tr("No events in window — point the camera at a "
                        "textured scene."));
        return;
    }

    contrast_value_->setText(QString::number(m.contrast, 'f', 1));
    width_value_->setText(QString::number(m.mean_line_width, 'f', 2));

    QString rate_text;
    if (m.event_rate >= 1.0e6) {
        rate_text = tr("Event rate: %1 Mev/s").arg(m.event_rate / 1.0e6, 0, 'f', 2);
    } else if (m.event_rate >= 1.0e3) {
        rate_text = tr("Event rate: %1 kev/s").arg(m.event_rate / 1.0e3, 0, 'f', 1);
    } else {
        rate_text = tr("Event rate: %1 ev/s").arg(m.event_rate, 0, 'f', 0);
    }
    rate_label_->setText(rate_text);

    contrast_chart_->add_value(m.contrast);
    width_chart_->add_value(m.mean_line_width);
}

} // namespace gui
