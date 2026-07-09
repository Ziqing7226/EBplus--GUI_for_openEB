// gui/main_window.h — top-level QMainWindow.
//
// Layout (design §5.1):
//   - central:  EventDisplayWidget (OpenGL)
//   - right dock: SettingsPanel (two tabs: 基础功能 / 算法模块)
//   - bottom (status bar): connection | event rate | timestamp | recording state
//   - bottom (above status bar): PlaybackControls (file playback only)
//   - menu bar: File | View | Camera | Calibration | Tools | Help
//
// The Algorithm menu has been removed — all algorithm configuration lives
// in the sidebar's "算法模块" tab. The sidebar can be hidden via the
// View menu (Ctrl+Shift+S) to maximize the display area.
//
// Phase 1-2: live camera + bias/roi/esp/trigger control.
// Phase 3:   recorder + playback controls.
// Phase 4:   exporter dialog + config manager.
// Phase 5:   preprocessing panel, algorithms panel, file tools panel.
// Phase 9:   calibration wizard.
// Phase 10:  multi-window layout, i18n, layout save/restore.

#ifndef GUI_MAIN_WINDOW_H
#define GUI_MAIN_WINDOW_H

#include <QHash>
#include <QMainWindow>
#include <QPointer>
#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <metavision/sdk/base/utils/callback_id.h>
#include <metavision/sdk/base/events/event_cd.h>

#include "algo_bridge/algo_bridge.h"
#include "app/camera_controller.h"
#include "app/file_converter.h"
#include "app/theme_controller.h"
#include "config/config_manager.h"
#include "config/layout_manager.h"
#include "display/event_display_widget.h"
#include "display/frame_annotator.h"
#include "exporter/exporter_controller.h"
#include "panels/settings_panel.h"
#include "recorder/playback_controller.h"
#include "recorder/recorder_controller.h"
#include "display/space_time_display.h"
#include "widgets/algo_window.h"
#include "widgets/custom_title_bar.h"
#include "widgets/multi_window_manager.h"

class QLabel;
class QAction;
class QMenu;
class QToolBar;
class QDockWidget;

namespace gui {

class PlaybackControls;
class ExportDialog;
class CalibrationWizard;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* event) override;

signals:
    /// Emitted from the frame_ready handler after process_algo_results() has
    /// applied overlays/replacements. Multi-window child displays connect to
    /// this instead of the raw FramePipeline signal so they see the same
    /// annotated frame as the main display.
    void annotated_frame_ready(QImage frame);

private slots:
    void on_open_file();
    void on_connect_first();
    void on_disconnect();
    void on_refresh_devices();
    void on_about();
    void on_save_config();
    void on_load_config();
    void on_save_biases();
    void on_load_biases();
    void on_toggle_roi_drag(bool on);

    // Phase 3 — recording / playback.
    void on_record_start();
    void on_record_stop();
    void on_record_elapsed(std::chrono::seconds s);

    // Phase 4 — export / presets.
    void on_export_dialog();
    void on_apply_preset(int index);

    // Phase 9 — calibration.
    void on_intrinsic_wizard();

    // Phase 10 — multi-window / layout / standalone algorithm views.
    void on_open_xyt_view();
    void on_open_algo_window(const std::string& algo_name);
    void on_add_display_window();
    void on_tile_windows();
    void on_save_layout();
    void on_load_layout();
    void on_reset_layout();

private:
    void build_menus();
    void build_toolbar();
    void build_status_bar();
    void build_title_bar_controls();
    void wire_signals();
    void update_palettes(int index);
    void forward_panel_message(const QString& msg, bool isError);

    /// Shows/hides the right-edge sidebar tab based on dock visibility.
    /// Called whenever the settings dock is toggled.
    void update_sidebar_tab_visibility();

    void on_file_opened_for_playback(const QString& path);

    // Algorithm event/result pipeline — pushes CD events to all live
    // AlgoInstances and pulls results for overlay/replace/standalone display.
    void install_algo_callback();
    void remove_algo_callback();
    void process_algo_results(QImage& frame);

    /// File playback: feeds the events in the current accumulation window
    /// (emitted by FileFrameGenerator) to all live AlgoInstances and to the
    /// XYT 3D display, synchronously with the displayed frame. This replaces
    /// the SDK CD callback path for file sources, where all events arrive in
    /// ~10ms and would otherwise be processed before the first frame is shown.
    void on_events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                                Metavision::timestamp ts);

    EventDisplayWidget* display_{nullptr};
    SettingsPanel* settings_{nullptr};
    QDockWidget* settings_dock_{nullptr};  ///< Right-dock wrapper, for hide/show.
    PlaybackControls* playback_controls_{nullptr};

    QLabel* status_conn_{nullptr};
    QLabel* status_rate_{nullptr};
    QLabel* status_ts_{nullptr};
    QLabel* status_rec_{nullptr};

    // File menu actions.
    QAction* a_save_cfg_{nullptr};
    QAction* a_load_cfg_{nullptr};
    QAction* a_save_biases_{nullptr};
    QAction* a_load_biases_{nullptr};
    QAction* a_export_{nullptr};
    QAction* a_record_start_{nullptr};
    QAction* a_record_stop_{nullptr};

    /// "Recent Files" submenu — persisted via QSettings so the list survives
    /// restarts. Most recent first, capped at 10 entries.
    QMenu* m_recent_files_{nullptr};
    void build_recent_files_menu();
    void add_recent_file(const QString& path);
    void on_open_recent_file(const QString& path);

    // Camera menu actions.
    QAction* a_roi_drag_{nullptr};

    // Calibration menu actions.
    QMenu* m_calibration_{nullptr};

    // Tools menu.
    QMenu* m_tools_{nullptr};

    CameraController camera_;
    AlgoBridge algo_bridge_;

    // Phase 3.
    RecorderController recorder_;
    PlaybackController playback_;

    // Phase 4.
    ExporterController exporter_;
    ConfigManager config_;
    ExportDialog* export_dialog_{nullptr};

    // Phase 5.
    FileConverter file_converter_;

    // Phase 9 — owned lazily; built when the wizard is first opened.
    CalibrationWizard* calibration_wizard_{nullptr};

    // Phase 10 — layout manager is owned from construction; multi-window
    // manager is owned lazily.
    std::unique_ptr<MultiWindowManager> multi_window_;
    std::unique_ptr<LayoutManager> layout_manager_;

    // Standalone algorithm windows (design §5.6.1 / §5.6.6).
    // xyt_visualizer keeps a dedicated SpaceTimeDisplay (QOpenGLWidget with
    // 3D rendering); all other algorithms use the generic AlgoWindow
    // (algo_windows_) for both parameter control and result display.
    QPointer<SpaceTimeDisplay> xyt_display_;
    std::shared_ptr<AlgoInstance> xyt_algo_;

    /// Generic AlgoWindow instances keyed by algo name (design §5.6.6).
    /// Every self-developed algorithm gets an AlgoWindow when enabled, so the
    /// user can adjust any parameter (including the 5 ROI params) at runtime.
    QHash<std::string, QPointer<AlgoWindow>> algo_windows_;

    // Algorithm event/result pipeline.
    std::optional<Metavision::CallbackId> algo_cd_cb_id_;
    std::atomic<Metavision::timestamp> algo_last_xyt_post_us_{0};
    FrameAnnotator annotator_;
    Metavision::timestamp prev_frame_ts_{0};

    /// View menu action to toggle the sidebar (settings dock) visibility.
    QAction* a_toggle_sidebar_{nullptr};

    /// Main toolbar (top) with prominent toggle buttons for sidebar,
    /// playback panel, and layout actions.
    QToolBar* main_toolbar_{nullptr};

    /// Thin vertical toolbar on the right edge — visible only when the
    /// sidebar is hidden, so the user always has a way to bring it back.
    /// Acts as the "collapsed marker" requested in the UX pass.
    QToolBar* sidebar_tab_{nullptr};
    QAction* a_show_sidebar_{nullptr};  ///< Action inside sidebar_tab_.

    /// Draws the ROI rectangle of any enabled self-developed algorithm
    /// (design §5.6.6: all self-developed algos support ROI) on the main
    /// display frame so the user can see which region is being processed.
    /// Called from process_algo_results().
    void draw_roi_overlays(QImage& frame);

    /// Application theme controller (background color + light/dark mode).
    /// Owned by MainWindow; the SettingsPanel sidebar exposes its UI.
    ThemeController theme_;

    /// Edge/corner resize handles for the frameless window.
    std::vector<ResizeGrip*> resize_grips_;
};

} // namespace gui

#endif // GUI_MAIN_WINDOW_H
