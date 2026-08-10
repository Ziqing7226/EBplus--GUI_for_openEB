// gui/calibration/calibration_wizard.h — intrinsic calibration UI (Zhou's Method).
//
// Redesigned around a STATIC asymmetric screw-head grid + Space-key capture. The
// layout is a top row of three equal columns — parameters | live camera
// aim-view | captured-frame preview — above a large full-width pattern so the
// user can aim the camera at it. On Space, the wizard takes the last
// (user-tunable 100–1000 µs, default 500 µs) of CD events, renders a
// three-valued colour frame (black/gold=ON/white=OFF, blend where both fired),
// and submits it to a CalibrationWorker that runs screw-head detection
// (cross-centre + ring polarity verification) on a background thread.
// cv::calibrateCamera also runs on the worker; the result is exported to YAML
// (auto-mkdir). Detection is capture-triggered only (never per-frame); the
// capture button is serialized — disabled while a frame is being judged and
// re-enabled once the accept/reject verdict returns.
//
// Zhou's screw-head grid: each marker is a dashed cross (pins the centre)
// inside a solid thin ring (supplies the polarity signal), white on black.
// Marker centres sit on an asymmetric 6×5 layout whose row offset gives 8-fold
// orientation disambiguation (no missing corners). dot_gap (1/2/3, default 2)
// is user-configurable.
//
// Bug-absorption (see devlog/v2_audit_and_plan.md §6 Phase 4):
//  - tap attach() disconnects first (no duplicate Connection);
//  - no QScreen::physicalDotsPerInch() — marker spacing mm is user-input;
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
    /// @brief Cross-thread: reconfigure the worker's board scale + target + dot
    /// gap. The grid is fixed at asymmetric 6×5.
    void configure_requested(double square_mm, int target, int dot_gap);
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

    // Configuration. The grid is fixed at asymmetric 6×5 (no cols/rows spinbox):
    // the row offset gives 8-fold orientation disambiguation, so no square-grid
    // rejection is needed.
    QComboBox*      dot_gap_{nullptr};
    QSpinBox*       capture_window_{nullptr};
    QDoubleSpinBox* square_mm_{nullptr};
    QSpinBox*       target_frames_{nullptr};

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
