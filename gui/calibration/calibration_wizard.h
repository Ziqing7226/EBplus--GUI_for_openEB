// gui/calibration/calibration_wizard.h — intrinsic calibration UI.
//
// Phase 9 (design §4.5, §5.3), rebuilt around a flashing on-screen chessboard
// + auto-capture workflow suited to event cameras: the chessboard inverts at
// 20 Hz, each flip generates a burst of events along the edges, and a 1 ms
// accumulation window aligned with the flip captures the pattern "for free".
// The wizard picks the max-event 1 ms window out of every 50 ms cycle,
// detects the chessboard, rejects duplicates via corner displacement
// (audit §9.2-D — the old MSE check was defeated by the 20 Hz inversion:
// consecutive accepted frames are opposite phases of the same pattern, so
// MSE always looked "different" and a stationary camera could fill the
// quota), and auto-ends after a configurable frame count (default 30) —
// then runs cv::calibrateCamera and exports the result to YAML.
//
// Thread model (audit §9.2-B / §六-B3/B4): the 50 ms capture timer tick on
// the GUI thread only drains the tap and enqueues the 1 ms event window.
// All heavy work — render_event_window, medianBlur, corner detection,
// duplicate check, intrinsic_->run() — runs on a std::thread worker (same
// pattern as ExporterController). The IntrinsicCalibration object is touched
// ONLY by the worker thread (plus on_intrinsic_save() after the worker is
// joined). Results return to the GUI thread via
// QMetaObject::invokeMethod(this, lambda, Qt::QueuedConnection); posted
// functors are auto-discarded if the wizard is destroyed first.

#ifndef GUI_CALIBRATION_CALIBRATION_WIZARD_H
#define GUI_CALIBRATION_CALIBRATION_WIZARD_H

#include <QDialog>
#include <QImage>
#include <QMetaObject>
#include <QPointer>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "algo/calibration/intrinsic.h"
#include "calibration_event_tap.h"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QProgressBar;
class QPushButton;
class QScrollArea;
class QSpinBox;
class QTabWidget;
class QTimer;

namespace gui {

class CameraController;
class ChessboardDisplay;
class EventDisplayWidget;

/// @brief Dialog hosting the intrinsic calibration wizard.
class CalibrationWizard : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationWizard(QWidget* parent = nullptr);
    ~CalibrationWizard();

    /// @brief Provides the live camera so the wizard can tap CD events.
    /// Safe to call with nullptr (capture buttons disabled until set).
    void set_camera(CameraController* controller);

    /// @brief Provides the event display (unused by the auto-capture loop
    /// but kept for the manual fallback and for sensor geometry lookup).
    void set_display(EventDisplayWidget* display);

public slots:
    void show_intrinsic();

private slots:
    // Intrinsic tab.
    void on_show_chessboard();
    void on_start_capture();
    void on_stop_capture();
    void on_intrinsic_reset();
    void on_intrinsic_save();
    void on_capture_tick();

private:
    /// @brief Plain-data snapshot of the capture configuration, written by
    ///        the GUI thread before the worker starts and read only by the
    ///        worker afterwards (no locking needed — happens-before via
    ///        std::thread construction).
    struct CaptureConfig {
        int cols{9};
        int rows{6};
        float square_mm{25.0f};
        int target_frames{30};
        double min_move_px{2.0};
        int sensor_w{0};
        int sensor_h{0};
    };

    void build_intrinsic_tab();
    void update_intrinsic_preview(const QImage& img);
    void set_status(const QString& text);
    void rebuild_screen_combo();
    QString default_export_path() const;
    static QImage cv_to_qimage(const cv::Mat& mat);
    /// @brief Renders a 1 ms event window to a single-channel cv::Mat at
    ///        sensor resolution (background grey, ON events white, OFF black).
    ///        Worker thread only.
    static cv::Mat render_event_window(const std::vector<Metavision::EventCD>& evs,
                                       int sensor_w, int sensor_h);

    /// @brief Worker thread body: drain queue → render → medianBlur →
    ///        corner-displacement duplicate check → intrinsic_->add_frame →
    ///        auto-run calibration at the target frame count.
    void worker_loop();
    /// @brief Signals the worker to exit and joins it. GUI thread only.
    void stop_and_join_worker();
    /// @brief Joins the worker if it is joinable (reap after self-exit).
    void join_worker();

    /// @brief Posts @p fn to the GUI thread. Qt discards the posted functor
    ///        automatically if this wizard is destroyed before delivery, so
    ///        lambdas may capture `this` safely.
    template <typename F>
    void post_to_gui(F&& fn) {
        QMetaObject::invokeMethod(this, std::forward<F>(fn), Qt::QueuedConnection);
    }

    // Shared.
    QTabWidget* tabs_{nullptr};

    // Intrinsic controls.
    QComboBox*  in_screen_{nullptr};
    QSpinBox*   in_cols_{nullptr};
    QSpinBox*   in_rows_{nullptr};
    QSpinBox*   in_target_frames_{nullptr};
    QDoubleSpinBox* in_min_move_px_{nullptr};  ///< §9.2-D duplicate threshold.
    QDoubleSpinBox* in_square_mm_{nullptr};    ///< §9.2-G editable square size.
    QLabel*     in_status_{nullptr};
    QLabel*     in_preview_label_{nullptr};
    QScrollArea* in_preview_area_{nullptr};
    QProgressBar* in_progress_{nullptr};
    QPushButton* in_show_board_btn_{nullptr};
    QPushButton* in_start_btn_{nullptr};
    QPushButton* in_stop_btn_{nullptr};
    QPushButton* in_reset_btn_{nullptr};
    QPushButton* in_save_btn_{nullptr};

    std::unique_ptr<gui_algo::IntrinsicCalibration> intrinsic_;
    gui_algo::IntrinsicResult intrinsic_result_;
    QImage in_last_preview_;

    // Auto-capture state.
    CalibrationEventTap tap_;
    QTimer* capture_timer_{nullptr};
    bool capturing_{false};

    // Worker (std::thread + queue, ExporterController pattern).
    CaptureConfig capture_cfg_;
    std::thread worker_;
    std::mutex work_mutex_;
    std::condition_variable work_cv_;
    std::deque<std::shared_ptr<std::vector<Metavision::EventCD>>> work_queue_;
    std::atomic<bool> worker_stop_{false};

    CameraController* camera_{nullptr};
    QPointer<EventDisplayWidget> display_;
    QPointer<ChessboardDisplay> chessboard_;
};

} // namespace gui

#endif // GUI_CALIBRATION_CALIBRATION_WIZARD_H
