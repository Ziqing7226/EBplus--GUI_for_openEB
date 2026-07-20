// gui/calibration/sharpness_dialog.h — live sharpness meter (count-image
// based, systematic_audit §9.3 rewrite).
//
// Primary data source (S1): CameraController::cd_events_ready. The dialog
// accumulates raw CD events in a rolling 50/100/200 ms window, builds a
// per-pixel count image (polarity ignored, no palette / overlay / 8-bit
// saturation contamination — fixes R3), and computes normalized contrast
// (S3, higher = sharper) and mean line width (S4, lower = sharper) via
// sharpness_metrics.h. This replaces the old 10 Hz poll of
// EventDisplayWidget::current_frame() + variance-of-Laplacian, which measured
// noise density and event density rather than sharpness (R1/R2).
//
// Threading: the cd_events_ready connection is Qt::DirectConnection (same
// pattern as CalibrationEventTap), so on_events_ready() runs on the SDK
// streaming thread and only appends to a mutex-protected buffer — never
// touches the GUI. The 10 Hz QTimer (GUI thread) trims the buffer to the
// window, builds the count image under the lock, and runs the metric
// computation after unlocking.

#ifndef GUI_CALIBRATION_SHARPNESS_DIALOG_H
#define GUI_CALIBRATION_SHARPNESS_DIALOG_H

#include <QDialog>
#include <QWidget>

#include <memory>
#include <mutex>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>

class QComboBox;
class QLabel;
class QTimer;

namespace gui {

class CameraController;
class EventDisplayWidget;

/// @brief Rolling line chart with an adaptive Y-axis (95th percentile of the
/// visible samples + 10 % headroom; audit §9.3 S6 — no fixed/theoretical
/// ceiling). One chart per metric; the dialog stacks two of them.
class SharpnessChart : public QWidget {
    Q_OBJECT
public:
    explicit SharpnessChart(const QString& title, const QColor& color,
                            QWidget* parent = nullptr);

    /// @brief Appends a sample tagged with the current wall-clock time.
    /// Samples older than the rolling window are pruned.
    void add_value(double value);

    /// @brief Clears all samples.
    void clear();

    /// @brief Number format for the current-value readout.
    void set_format(char fmt, int precision);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    struct Sample {
        qint64 t_ms;
        double value;
    };
    QString title_;
    QColor color_;
    std::vector<Sample> samples_;
    char fmt_{'f'};
    int precision_{2};
    static constexpr qint64 kWindowMs = 5000;  ///< Rolling window width.
};

/// @brief Live sharpness meter on the raw CD event stream.
class SharpnessDialog : public QDialog {
    Q_OBJECT
public:
    explicit SharpnessDialog(QWidget* parent = nullptr);
    ~SharpnessDialog();

    /// @brief Sets the camera as the data source: subscribes to
    /// cd_events_ready (DirectConnection) and reads sensor dimensions.
    /// Safe to call with nullptr. Repeated calls re-connect (UniqueConnection).
    void set_camera(CameraController* camera);

    /// @brief Degraded path kept for callers without a camera: the dialog
    /// shows a "no data source" placeholder instead of metrics.
    void set_display(EventDisplayWidget* display);

protected:
    void showEvent(QShowEvent* event) override;
    void closeEvent(QCloseEvent* event) override;

private slots:
    void on_tick();
    void on_events_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events);

private:
    int window_ms() const;
    void reset_accumulation();
    void show_no_data(const QString& reason);

    CameraController* camera_{nullptr};
    EventDisplayWidget* display_{nullptr};  // fallback marker only, never polled
    QTimer* timer_{nullptr};
    QComboBox* window_combo_{nullptr};
    SharpnessChart* contrast_chart_{nullptr};
    SharpnessChart* width_chart_{nullptr};
    QLabel* contrast_value_{nullptr};
    QLabel* width_value_{nullptr};
    QLabel* rate_label_{nullptr};
    QLabel* hint_label_{nullptr};

    // --- Accumulation state, shared between the SDK callback thread
    // (on_events_ready, DirectConnection) and the GUI thread (on_tick).
    // Guarded by mutex_; never touch GUI objects under this lock. ---
    std::mutex mutex_;
    std::vector<Metavision::EventCD> buffer_;
    int sensor_width_{0};
    int sensor_height_{0};

    /// Cap on buffer_.size() in case the GUI thread stalls (same rationale
    /// and scale as CalibrationEventTap::kMaxBufferEvents).
    static constexpr std::size_t kMaxBufferEvents = 2'000'000;
};

} // namespace gui

#endif // GUI_CALIBRATION_SHARPNESS_DIALOG_H
