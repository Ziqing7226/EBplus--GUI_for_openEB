// gui/calibration/calibration_wizard.h — intrinsic calibration UI (Phase 4).
//
// Redesigned around a STATIC asymmetric circle grid + Space-key capture. The
// layout is a top row of three equal columns — parameters | live camera
// aim-view | captured-frame preview — above a large full-width circle-grid
// pattern so the user can aim the camera at it. On Space, the wizard takes the
// last 5000 µs of CD events (polarity ignored), renders a full-resolution
// binary frame, and submits it to a CalibrationWorker that runs
// cv::findCirclesGrid with CALIB_CB_ASYMMETRIC_GRID on a background thread.
// cv::calibrateCamera also runs on the worker; the result is exported to YAML
// (auto-mkdir). Detection is capture-triggered only (never per-frame); the
// capture button is serialized — disabled while a frame is being judged and
// re-enabled once the accept/reject verdict returns.
//
// Zhou's Circle Grid: the on-screen pattern uses a waffle texture — each
// circle is a white edge ring (dot_size px thick) plus an interior grid of
// white dots — instead of a solid disc, producing far more brightness
// transitions per circle and improving findCirclesGrid detection rate. The
// dot size (1/2/3, default 2) is user-configurable via a dropdown.
//
// Phase 4 bug-absorption (see devlog/v2_audit_and_plan.md §6 Phase 4):
//  - tap attach() disconnects first (no duplicate Connection);
//  - no QScreen::physicalDotsPerInch() — cell spacing mm is user-input;
//  - no raw QScreen* held (the pattern is embedded; no screen tracking, so
//    the hot-plug dangling-pointer concern is eliminated by design).

#ifndef GUI_CALIBRATION_CALIBRATION_WIZARD_H
#define GUI_CALIBRATION_CALIBRATION_WIZARD_H

#include <QDialog>
#include <QImage>
#include <QPointer>
#include <QString>
#include <QWidget>

#include <opencv2/core.hpp>
#include <metavision/sdk/base/events/event_cd.h>

#include "calibration_event_tap.h"

class QComboBox;
class QDoubleSpinBox;
class QEvent;
class QLabel;
class QPaintEvent;
class QProgressBar;
class QPushButton;
class QScrollArea;
class QShowEvent;
class QSpinBox;
class QThread;
class QTimer;

namespace gui {

class CameraController;
class CircleGridDisplay;
class CalibrationWorker;
class EventDisplayWidget;

/// @brief Lightweight live-camera preview widget. Stores a QImage and paints
/// it via QPainter::drawImage in paintEvent — same approach as
/// FocusCameraView. Also handles the "no camera" / "not running" text states.
class CalibrationCameraView : public QWidget {
public:
    explicit CalibrationCameraView(QWidget* parent = nullptr);

    /// @brief Shows @p frame (scaled to fit). Clears any prior message.
    void set_frame(const QImage& frame);

    /// @brief Shows @p msg centered (no frame). Clears any prior frame.
    void set_message(const QString& msg);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage frame_;
    QString message_;
};

/// @brief Dialog hosting the intrinsic calibration workflow.
class CalibrationWizard : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationWizard(QWidget* parent = nullptr);
    ~CalibrationWizard();

    /// @brief Provides the live camera so the wizard can tap CD events.
    /// Safe to call with nullptr (capture stays disabled).
    void set_camera(CameraController* controller);

    /// @brief Provides the event display whose current frame is polled for the
    /// side-by-side aim view.
    void set_display(EventDisplayWidget* display);

public slots:
    void show_intrinsic();

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void hideEvent(QHideEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void changeEvent(QEvent* event) override;

signals:
    /// @brief Cross-thread: reconfigure the worker's board geometry.
    void configure_requested(int cols, int rows, double square_mm, int target);
    /// @brief Cross-thread: submit a rendered capture frame for detection.
    void submit_frame(const cv::Mat& frame);
    /// @brief Cross-thread: run cv::calibrateCamera on the worker.
    void run_calibration_requested();
    /// @brief Cross-thread: write the calibration YAML on the worker.
    void export_requested(const QString& path);

private slots:
    void on_camera_tick();
    void on_intrinsic_reset();
    void on_capture_pressed();
    void on_frame_accepted(QImage annotated, std::size_t accepted, std::size_t target);
    void on_frame_rejected(QString reason);
    void on_capture_complete(std::size_t accepted);
    void on_calibration_done(bool ok, double rms, int frames_used, QString error);
    void on_export_pressed();
    void on_export_done(bool ok, QString message);
    void on_config_changed();

private:
    void build_ui();
    void set_status(const QString& text);
    void apply_pattern_to_display();
    void configure_worker();
    void enable_capture(bool on);
    cv::Mat render_event_frame(const std::vector<Metavision::EventCD>& evs,
                               int sensor_w, int sensor_h);
    void teardown_worker();
    QString default_export_path() const;

    // Configuration.
    QSpinBox*       cols_{nullptr};
    QSpinBox*       rows_{nullptr};
    QComboBox*      dot_size_{nullptr};
    QDoubleSpinBox* square_mm_{nullptr};
    QSpinBox*       target_frames_{nullptr};
    // Last valid (non-square) cols/rows — used to revert a spinbox change
    // that would make cols == rows (invalid for ASYMMETRIC_GRID).
    int prev_cols_{6};
    int prev_rows_{5};

    // Top row: params | live camera aim-view | captured preview; bottom: pattern.
    CircleGridDisplay* pattern_{nullptr};
    CalibrationCameraView* camera_view_{nullptr};

    // Progress / preview / status.
    QLabel*       hint_{nullptr};
    QLabel*       spacing_note_{nullptr};  ///< Circle-spacing measurement instruction.
    QLabel*       preview_label_{nullptr};
    QScrollArea*  preview_area_{nullptr};
    QProgressBar* progress_{nullptr};
    QLabel*       status_{nullptr};

    QPushButton* capture_btn_{nullptr};
    QPushButton* reset_btn_{nullptr};
    QPushButton* export_btn_{nullptr};

    QTimer* camera_timer_{nullptr};

    // Event capture + worker.
    CalibrationEventTap tap_;
    CalibrationWorker* worker_{nullptr};
    QThread* worker_thread_{nullptr};
    bool capture_in_flight_{false};   ///< Ignore Space while a frame is processing.
    bool capture_done_{false};        ///< Target reached; further captures blocked until reset.

    CameraController* camera_{nullptr};
    QPointer<EventDisplayWidget> display_;
};

} // namespace gui

#endif // GUI_CALIBRATION_CALIBRATION_WIZARD_H
