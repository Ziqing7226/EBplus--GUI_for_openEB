// gui/calibration/calibration_wizard.h — intrinsic calibration UI (blinking
// chessboard method).
//
// Layout: a top row of three equal columns — parameters | live camera aim-view
// | captured-frame preview — above a large full-width BLINKING CHESSBOARD
// pattern (9×6 inner corners, alternating with a blank frame at 10 ms) so the
// user can aim the camera at it. On Space, the wizard takes the last
// (user-tunable 200–200000 µs, default 100000 µs) of CD events, accumulates the
// per-polarity masks, and submits them to a CalibrationWorker that builds the
// binary blink frame and runs chessboard detection on a background thread.
// There is NO capture-review dialog: a successful detection flows straight
// through the coverage + duplicate checks and is accumulated (progress bar
// advances); a failure is reported on the status line. The two-pass
// cv::calibrateCamera also runs on the worker; the result is exported to YAML
// (auto-mkdir). Detection is capture-triggered only (never per-frame); the
// capture button is serialized — disabled while a frame is processed and
// re-enabled once the accept/reject verdict returns.
//
// Blinking chessboard: black squares toggle dark↔light with the blank frame and
// fire both polarities, forming a filled checkerboard in the blink frame that
// cv::findChessboardCorners consumes. The physical square size (mm) is supplied
// by the user — screen DPI is deliberately not used (unreliable on X11).
//
// Bug-absorption notes:
//  - tap attach() disconnects first (no duplicate Connection);
//  - no QScreen::physicalDotsPerInch() — square size mm is user-input;
//  - no raw QScreen* held (the pattern is embedded; no screen tracking, so
//    the hot-plug dangling-pointer concern is eliminated by design).

#ifndef GUI_CALIBRATION_CALIBRATION_WIZARD_H
#define GUI_CALIBRATION_CALIBRATION_WIZARD_H

#include <QDialog>
#include <QImage>
#include <QPointer>
#include <QString>
#include <QWidget>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <metavision/sdk/base/events/event_cd.h>

#include "calibration_event_tap.h"

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
class BlinkingChessboardDisplay;
class CalibrationWorker;
class EventDisplayWidget;
struct BlinkCapture;

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
    /// @brief Cross-thread: reconfigure the worker's board scale + target.
    /// The grid is fixed at 9×6 inner corners.
    void configure_requested(double square_mm, int target);
    /// @brief Cross-thread: submit a capture's polarity masks for the worker to
    /// build the blink frame, detect and — on success — accumulate directly
    /// (no review dialog); the verdict arrives via frame_accepted/rejected.
    void process_capture_requested(const gui::BlinkCapture& capture);
    /// @brief Cross-thread: remove the most recently accepted observation
    /// from the worker's accumulator. The wizard gates this so at most one
    /// delete follows each accept — a second consecutive delete (which would
    /// drop the prior capture) is blocked in the GUI before this fires.
    void delete_last_capture_requested();
    /// @brief Cross-thread: run the two-pass calibration on the worker.
    void run_calibration_requested();
    /// @brief Cross-thread: write the calibration YAML on the worker.
    void export_requested(const QString& path);

    /// @brief Emitted after the wizard overrides or restores the diff_on /
    /// diff_off biases out-of-band, so the Biases panel can re-read the
    /// hardware values (BiasesPanel::refresh_row_values).
    void biases_changed_externally();

private slots:
    void on_camera_tick();
    void on_intrinsic_reset();
    void on_capture_pressed();
    void on_delete_capture();
    void on_frame_accepted(QImage annotated, std::size_t accepted, std::size_t target);
    void on_frame_rejected(QString reason);
    void on_frame_deleted(std::size_t remaining);
    void on_capture_complete(std::size_t accepted);
    void on_calibration_done(bool ok, double rms, int frames_used, int removed_frames,
                             QString error);
    void on_export_pressed();
    void on_export_done(bool ok, QString message);
    void on_config_changed();

private:
    void build_ui();
    void set_status(const QString& text);
    void apply_pattern_to_display();
    void configure_worker();
    void enable_capture(bool on);
    /// @brief Refreshes the "Delete this capture" button enabled state from the
    /// current capture/delete/done flags. The button is enabled only when the
    /// last accept has not yet been deleted (can_delete_last_), no capture is
    /// in flight, and the target has not been reached — enforcing the
    /// "one delete per capture, before the final set is locked" contract.
    void update_delete_enabled();
    /// @brief Renders the top of captured_previews_ into the preview label
    /// (scaled to the scroll area width), or the placeholder text when the
    /// stack is empty. Called after each accept and after each delete so the
    /// preview always reflects the current last-accepted frame.
    void show_last_preview();
    void teardown_worker();
    QString default_export_path() const;
    /// @brief Snapshots the diff_on/diff_off biases and sets both to their
    /// hardware maximum (blinking-LCD PWM noise-floor suppression — see the
    /// .cpp). No-op when the camera/bias facility is unavailable or an
    /// override is already active.
    void apply_diff_bias_override();
    /// @brief Restores the snapshot taken by apply_diff_bias_override().
    /// No-op when no override is active.
    void restore_diff_bias_override();

    // Configuration. The grid is fixed at 9×6 inner corners (no cols/rows
    // spinbox): the asymmetric board gives the checkerboard a unique
    // orientation, so no square-grid rejection is needed.
    QSpinBox*       capture_window_{nullptr};
    QDoubleSpinBox* square_mm_{nullptr};
    QSpinBox*       target_frames_{nullptr};

    // Top row: params | live camera aim-view | captured preview; bottom: pattern.
    BlinkingChessboardDisplay* pattern_{nullptr};
    CalibrationCameraView* camera_view_{nullptr};

    // Progress / preview / status.
    QLabel*       hint_{nullptr};
    QLabel*       spacing_note_{nullptr};  ///< Square-size measurement instruction.
    QLabel*       preview_label_{nullptr};
    QScrollArea*  preview_area_{nullptr};
    QProgressBar* progress_{nullptr};
    QLabel*       status_{nullptr};

    QPushButton* capture_btn_{nullptr};
    QPushButton* delete_capture_btn_{nullptr};  ///< Discards the last accepted capture.
    QPushButton* reset_btn_{nullptr};
    QPushButton* export_btn_{nullptr};

    QTimer* camera_timer_{nullptr};

    // Event capture + worker.
    CalibrationEventTap tap_;
    CalibrationWorker* worker_{nullptr};
    QThread* worker_thread_{nullptr};
    bool capture_in_flight_{false};   ///< Ignore Space while a frame is processing.
    bool capture_done_{false};        ///< Target reached; further captures blocked until reset.
    /// @brief True only between a successful accept and the delete of that same
    /// frame. Reset to false by delete / reset / config-change / capture-complete,
    /// so a second consecutive delete (which would drop the prior capture) is
    /// blocked. Set true again by the next on_frame_accepted.
    bool can_delete_last_{false};
    /// @brief Stack of accepted-frame annotated previews, parallel to the
    /// worker's image_points_. on_frame_accepted pushes; on_delete_capture pops
    /// so the preview reverts to the now-last capture (or the placeholder when
    /// empty). Lets the user see what remains after discarding a bad frame.
    std::vector<QImage> captured_previews_;

    CameraController* camera_{nullptr};
    QPointer<EventDisplayWidget> display_;

    /// Bias snapshot taken by apply_diff_bias_override() (bias name → value
    /// before the override). Non-empty while an override is active; restored
    /// on hide/close. Keys are the hardware's own bias names (matched by
    /// substring, e.g. "bias_diff_on").
    std::map<std::string, int> saved_diff_biases_;
};

} // namespace gui

#endif // GUI_CALIBRATION_CALIBRATION_WIZARD_H
