// gui/main_window.cpp

#include "main_window.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QSet>
#include <QStatusBar>
#include <QVBoxLayout>

#include <vector>

#include <metavision/sdk/core/utils/colors.h>

#include <opencv2/imgproc.hpp>

#include "panels/algorithms_panel.h"
#include "panels/biases_panel.h"
#include "panels/devices_panel.h"
#include "panels/display_panel.h"
#include "panels/esp_panel.h"
#include "panels/information_panel.h"
#include "panels/preprocessing_panel.h"
#include "panels/roi_panel.h"
#include "panels/statistics_panel.h"
#include "panels/trigger_panel.h"
#include "recorder/playback_controls.h"
#include "exporter/export_dialog.h"
#include "calibration/calibration_wizard.h"

namespace gui {

namespace {

/// Converts a cv::Mat (1-channel gray or 3-channel BGR) to a deep-copied
/// RGB888 QImage suitable for display.
QImage mat_to_qimage(const cv::Mat& mat) {
    if (mat.empty()) return QImage();
    cv::Mat rgb;
    if (mat.channels() == 1) {
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    } else if (mat.channels() == 3) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    } else {
        return QImage();
    }
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
}

} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      camera_(nullptr),
      algo_bridge_(),
      recorder_(nullptr),
      playback_(nullptr),
      exporter_(nullptr),
      file_converter_(nullptr) {
    setWindowTitle(tr("GUI for openEB"));
    resize(1280, 720);

    display_ = new EventDisplayWidget(this);
    setCentralWidget(display_);

    settings_ = new SettingsPanel(&algo_bridge_, &file_converter_, this);
    auto* dock = new QDockWidget(tr("Settings"), this);
    dock->setObjectName("SettingsDock");
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dock->setWidget(settings_);
    dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable);
    addDockWidget(Qt::RightDockWidgetArea, dock);
    dock->setMinimumWidth(320);

    // Phase 3: bottom-bar playback transport (hidden until a file is opened).
    playback_controls_ = new PlaybackControls(this);
    playback_controls_->set_controller(&playback_);
    auto* pb_dock = new QDockWidget(tr("Playback"), this);
    pb_dock->setObjectName("PlaybackDock");
    pb_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    pb_dock->setWidget(playback_controls_);
    pb_dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable);
    addDockWidget(Qt::BottomDockWidgetArea, pb_dock);
    pb_dock->setVisible(false); // shown only when a file is opened

    // Phase 4: export dialog (lazy-shown via menu).
    export_dialog_ = new ExportDialog(&exporter_, this);

    playback_.set_camera(&camera_);

    // Phase 10: layout manager (saves/restores dock geometry).
    layout_manager_ = std::make_unique<LayoutManager>(this, this);

    build_menus();
    build_status_bar();
    wire_signals();

    // Capture the factory layout (all docks in their default positions) so
    // reset_layout() can restore it. Must be called before load_default()
    // which may overlay a user-customized layout.
    layout_manager_->capture_default();

    // Try restoring the previous layout (silent on failure).
    layout_manager_->load_default();

    // Populate the device list on startup.
    on_refresh_devices();
}

MainWindow::~MainWindow() = default;

void MainWindow::closeEvent(QCloseEvent* event) {
    if (layout_manager_) layout_manager_->save_default();
    // Delete lazily-created child windows before our members are destroyed.
    // Their destroyed() handlers access MainWindow members (e.g. camera_,
    // temporal_cd_cb_id_) that would already be gone by the time ~QWidget
    // runs its automatic child cleanup.
    if (temporal_plot_) {
        delete temporal_plot_.data();
        // QPointer is auto-nulled by the destroyed signal.
    }
    if (calibration_wizard_) {
        delete calibration_wizard_;
        // calibration_wizard_ is nulled by the destroyed signal handler.
    }
    // Standalone algorithm windows — same reason as above: their
    // destroyed() handlers reset AlgoInstance shared_ptrs and QPointer
    // members that reference MainWindow state.
    if (time_surface_window_) { delete time_surface_window_.data(); }
    if (event_to_video_window_) { delete event_to_video_window_.data(); }
    if (freq_detector_window_) { delete freq_detector_window_.data(); }
    if (xyt_display_) { delete xyt_display_.data(); }
    // Pre-delete child widgets that hold raw pointers to MainWindow members.
    // Without this, ~QObject child cleanup runs after member destructors,
    // causing use-after-free in widget destructors.
    if (export_dialog_) { delete export_dialog_; export_dialog_ = nullptr; }
    recorder_.stop();
    camera_.disconnect();
    event->accept();
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void MainWindow::build_menus() {
    auto* mb = menuBar();

    // File
    auto* m_file = mb->addMenu(tr("&File"));
    m_file->addAction(tr("&Open File..."), this, &MainWindow::on_open_file,
                      QKeySequence::Open);
    a_save_cfg_ = m_file->addAction(tr("Save Camera Config..."), this, &MainWindow::on_save_config);
    a_load_cfg_ = m_file->addAction(tr("Load Camera Config..."), this, &MainWindow::on_load_config);
    a_save_cfg_->setEnabled(false);
    a_load_cfg_->setEnabled(false);
    m_file->addSeparator();
    a_save_biases_ = m_file->addAction(tr("Save Biases..."), this, &MainWindow::on_save_biases);
    a_load_biases_ = m_file->addAction(tr("Load Biases..."), this, &MainWindow::on_load_biases);
    a_save_biases_->setEnabled(false);
    a_load_biases_->setEnabled(false);
    m_file->addSeparator();
    // Phase 3 — recording actions.
    a_record_start_ = m_file->addAction(tr("Start &Recording..."), this,
                                        &MainWindow::on_record_start,
                                        QKeySequence("R"));
    a_record_stop_  = m_file->addAction(tr("Sto&p Recording"), this,
                                        &MainWindow::on_record_stop);
    a_record_start_->setEnabled(false);
    a_record_stop_->setEnabled(false);
    m_file->addSeparator();
    // Phase 4 — export.
    a_export_ = m_file->addAction(tr("&Export..."), this, &MainWindow::on_export_dialog);
    a_export_->setEnabled(false);
    m_file->addSeparator();
    // Phase 10 — algorithm parameter save/load.
    m_file->addAction(tr("Save Algo Params..."), this, [this]() {
        const QString path = QFileDialog::getSaveFileName(
            this, tr("Save Algorithm Parameters"), {},
            tr("JSON (*.json);;All Files (*)"));
        if (path.isEmpty()) return;
        QString err;
        if (config_.save_algo_params_to_file(&algo_bridge_, path, err)) {
            statusBar()->showMessage(tr("Algorithm params saved to %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Save failed"), err);
        }
    });
    m_file->addAction(tr("Load Algo Params..."), this, [this]() {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Load Algorithm Parameters"), {},
            tr("JSON (*.json);;All Files (*)"));
        if (path.isEmpty()) return;
        QString err;
        if (config_.load_algo_params_from_file(&algo_bridge_, path, err)) {
            statusBar()->showMessage(tr("Algorithm params loaded from %1").arg(path), 3000);
        } else if (!err.isEmpty()) {
            QMessageBox::warning(this, tr("Load failed"), err);
        }
    });
    m_file->addSeparator();
    m_file->addAction(tr("E&xit"), this, &QWidget::close, QKeySequence::Quit);

    // View
    auto* m_view = mb->addMenu(tr("&View"));
    auto* dock_toggle = m_view->addAction(tr("Toggle Settings Panel"));
    dock_toggle->setCheckable(true);
    dock_toggle->setChecked(true);
    connect(dock_toggle, &QAction::toggled, this, [this](bool on) {
        auto* dock = findChild<QDockWidget*>("SettingsDock");
        if (dock) dock->setVisible(on);
    });
    auto* pb_toggle = m_view->addAction(tr("Toggle Playback Panel"));
    pb_toggle->setCheckable(true);
    pb_toggle->setChecked(false);
    connect(pb_toggle, &QAction::toggled, this, [this](bool on) {
        auto* dock = findChild<QDockWidget*>("PlaybackDock");
        if (dock) dock->setVisible(on);
    });
    m_view->addAction(tr("Reset Layout"), this, &MainWindow::on_reset_layout);
    m_view->addAction(tr("Save Layout..."), this, &MainWindow::on_save_layout);
    m_view->addAction(tr("Load Layout..."), this, &MainWindow::on_load_layout);
    m_view->addSeparator();
    m_view->addAction(tr("&Fullscreen"), this,
                      [this]() { showFullScreen(); }, QKeySequence("F11"));

    // Camera
    auto* m_cam = mb->addMenu(tr("&Camera"));
    m_cam->addAction(tr("&Connect First Available"), this,
                     &MainWindow::on_connect_first, QKeySequence("Ctrl+Shift+C"));
    m_cam->addAction(tr("&Disconnect"), this, &MainWindow::on_disconnect);
    m_cam->addSeparator();
    m_cam->addAction(tr("Refresh Device &List"), this, &MainWindow::on_refresh_devices);
    m_cam->addAction(tr("&Device List..."), this, [this]() {
        auto* dock = findChild<QDockWidget*>("SettingsDock");
        if (dock) { dock->show(); dock->raise(); }
    });
    m_cam->addSeparator();
    a_roi_drag_ = m_cam->addAction(tr("&ROI Drag Mode"), this, &MainWindow::on_toggle_roi_drag);
    a_roi_drag_->setCheckable(true);
    a_roi_drag_->setChecked(false);
    a_roi_drag_->setShortcut(QKeySequence("Ctrl+R"));
    a_roi_drag_->setEnabled(false);
    m_cam->addSeparator();
    // Built-in config presets (Phase 4).
    auto* m_presets = m_cam->addMenu(tr("&Presets"));
    const auto preset_names = config_.preset_names();
    for (int i = 0; i < preset_names.size(); ++i) {
        m_presets->addAction(preset_names[i], this, [this, i]() { on_apply_preset(i); });
    }

    // Preprocess (Phase 5) — checkable actions mirrored to the PreprocessingPanel.
    m_preprocess_ = mb->addMenu(tr("&Preprocess"));
    {
        struct StageEntry { QString id; QString label; };
        const std::vector<StageEntry> stages = {
            {"roi_filter",      tr("ROI Filter")},
            {"polarity_filter", tr("Polarity Filter")},
            {"polarity_invert", tr("Polarity Invert")},
            {"flip_x",          tr("Flip X")},
            {"flip_y",          tr("Flip Y")},
            {"rotate",          tr("Rotate (0/90/180/270)")},
            {"transpose",       tr("Transpose")},
            {"rescale",         tr("Rescale")},
        };
        for (const auto& s : stages) {
            auto* a = m_preprocess_->addAction(s.label);
            a->setCheckable(true);
            a->setChecked(false);
            connect(a, &QAction::toggled, this, [this, id = s.id](bool on) {
                settings_->preprocessing_panel()->set_stage_enabled(id, on);
            });
        }
        m_preprocess_->addSeparator();
        m_preprocess_->addAction(tr("Open Preprocessing Panel..."), this, [this]() {
            auto* dock = findChild<QDockWidget*>("SettingsDock");
            if (dock) { dock->show(); dock->raise(); }
        });
    }

    // Frame Mode (Phase 5) — controls event accumulation time per frame.
    // Feasible modes adjust the accumulation window; unimplemented modes
    // are disabled.
    auto* m_frame = mb->addMenu(tr("Frame &Mode"));
    {
        const QStringList modes = {
            tr("Diff (10ms)"), tr("Integration (50ms)"), tr("Histogram"),
            tr("Time Decay (30ms)"), tr("Contrast Map"), tr("Periodic"), tr("On-Demand")
        };
        // Modes that are not yet implemented (disabled).
        const QSet<int> disabled = {2, 4, 5, 6};
        auto* grp = new QActionGroup(this);
        for (int i = 0; i < modes.size(); ++i) {
            auto* a = m_frame->addAction(modes[i]);
            a->setCheckable(true);
            a->setEnabled(!disabled.contains(i));
            if (i == 0) a->setChecked(true);
            grp->addAction(a);
            connect(a, &QAction::triggered, this, [this, i]() {
                settings_->display_panel()->set_frame_mode(i);
            });
        }
    }

    // Algorithm (design §5.4) — 30 self-developed modules.
    auto* m_algo = mb->addMenu(tr("&Algorithm"));
    {
        // Maps menu display name → algo_bridge name.
        const std::vector<std::pair<QString, QString>> algos = {
            {tr("Noise Filter"), "noise_filter"},
            {tr("Hot Pixel Filter"), "hot_pixel_filter"},
            {tr("Orientation Filter"), "orientation_filter"},
            {tr("Direction Selective"), "direction_selective"},
            {tr("Optical Flow (Sparse)"), "sparse_optical_flow"},
            {tr("Blob Detect"), "blob_detector"},
            {tr("Object Tracker"), "object_tracker"},
            {tr("Corner Detect"), "corner_detector"},
            {tr("Line Segment (ELiSeD)"), "line_segment"},
            {tr("Hough Line"), "hough_line"},
            {tr("Hough Circle"), "hough_circle"},
            {tr("Orientation Cluster"), "orientation_cluster"},
            {tr("Cluster LIF"), "cluster_lif"},
            {tr("Background Mask"), "background_mask"},
            {tr("Perspective Undistort"), "perspective_undistort"},
            {tr("Trigger Synced"), "trigger_synced"},
            {tr("Bandpass Filter"), "bandpass_filter"},
            {tr("EIS (Optical Gyro)"), "optical_gyro"},
            {tr("Ultra Slow Motion"), "ultra_slow_motion"},
            {tr("XYT 3D"), "xyt_visualizer"},
            {tr("Time Surface"), "time_surface"},
            {tr("Active Marker"), "active_marker"},
            {tr("Event -> Video"), "event_to_video"},
            {tr("Flow Statistics"), "flow_statistics"},
            {tr("ISI Analyzer"), "isi_analyzer"},
            {tr("Particle Counter"), "particle_counter"},
            {tr("Auto Bias"), "auto_bias"},
            {tr("Freq Detector"), "freq_detector"},
            {tr("Overlay"), "overlay"},
        };
        for (const auto& [label, name] : algos) {
            m_algo->addAction(label, this, [this, name, label]() {
                const auto key = name.toStdString();
                auto inst = algo_bridge_.find_live(key);
                if (!inst) {
                    inst = algo_bridge_.create(key);
                }
                if (inst) {
                    bool now_on = !inst->is_enabled();
                    inst->set_enabled(now_on);
                    statusBar()->showMessage(
                        tr("%1: %2").arg(label).arg(now_on ? "ON" : "OFF"), 3000);
                }
            });
        }
    }

    // Calibration (Phase 9) — launches the wizard lazily.
    m_calibration_ = mb->addMenu(tr("&Calibration"));
    m_calibration_->addAction(tr("&Intrinsic Wizard..."), this, &MainWindow::on_intrinsic_wizard);

    // Tools (design §5.5).
    m_tools_ = mb->addMenu(tr("&Tools"));
    // Standalone algorithm windows (design §5.6.1).
    m_tools_->addAction(tr("&Time Surface..."), this, &MainWindow::on_open_time_surface,
                        QKeySequence("Ctrl+Shift+T"));
    m_tools_->addAction(tr("&XYT 3D View..."), this, &MainWindow::on_open_xyt_view,
                        QKeySequence("Ctrl+Shift+X"));
    m_tools_->addAction(tr("&Event -> Video..."), this, &MainWindow::on_open_event_to_video,
                        QKeySequence("Ctrl+Shift+E"));
    m_tools_->addAction(tr("&Frequency Detector..."), this, &MainWindow::on_open_freq_detector,
                        QKeySequence("Ctrl+Shift+F"));
    m_tools_->addSeparator();
    m_tools_->addAction(tr("&Temporal Plot (legacy)..."), this, &MainWindow::on_open_temporal_plot);
    m_tools_->addAction(tr("&Add Display Window..."), this, &MainWindow::on_add_display_window);
    m_tools_->addAction(tr("&Tile Windows"), this, &MainWindow::on_tile_windows);
    m_tools_->addAction(tr("&Cascade Windows"), this, [this]() {
        if (multi_window_) multi_window_->cascade();
        else statusBar()->showMessage(tr("No display windows open."), 3000);
    });
    m_tools_->addAction(tr("&Close All Windows"), this, [this]() {
        if (multi_window_) multi_window_->close_all();
        else statusBar()->showMessage(tr("No display windows open."), 3000);
    });
    m_tools_->addSeparator();
    auto* a_fc = m_tools_->addAction(tr("&Frame Composer..."));
    a_fc->setEnabled(false);
    a_fc->setToolTip(tr("Not yet implemented (planned for a future phase)."));
    auto* a_ds = m_tools_->addAction(tr("&Data Synchronizer..."));
    a_ds->setEnabled(false);
    a_ds->setToolTip(tr("Not yet implemented (planned for a future phase)."));
    auto* a_tp = m_tools_->addAction(tr("&Timing Profiler..."));
    a_tp->setEnabled(false);
    a_tp->setToolTip(tr("Not yet implemented (planned for a future phase)."));

    // Help
    auto* m_help = mb->addMenu(tr("&Help"));
    m_help->addAction(tr("&About"), this, &MainWindow::on_about);
    m_help->addAction(tr("About &Qt"), this, &QApplication::aboutQt);
}

void MainWindow::build_status_bar() {
    auto* sb = statusBar();
    status_conn_ = new QLabel(tr("Disconnected"), this);
    status_rate_ = new QLabel(tr("— ev/s"), this);
    status_ts_   = new QLabel(tr("t: —"), this);
    status_rec_  = new QLabel(tr("Idle"), this);
    sb->addWidget(status_conn_);
    sb->addWidget(status_rate_);
    sb->addWidget(status_ts_);
    sb->addPermanentWidget(status_rec_);
}

void MainWindow::wire_signals() {
    // Devices panel <-> controller
    auto* dp = settings_->devices_panel();
    connect(dp, &DevicesPanel::refresh_requested, this, &MainWindow::on_refresh_devices);
    connect(dp, &DevicesPanel::connect_first_requested, this, &MainWindow::on_connect_first);
    connect(dp, &DevicesPanel::connect_serial_requested, this,
            [this](const QString& serial) { camera_.connect_serial(serial.toStdString()); });
    connect(dp, &DevicesPanel::disconnect_requested, this, &MainWindow::on_disconnect);

    // Controller -> UI
    connect(&camera_, &CameraController::connected, this, [this](const SensorInfo& info) {
        settings_->information_panel()->set_info(info);
        settings_->devices_panel()->set_connected(true);
        // Propagate actual sensor dimensions to the algorithm bridge so
        // algorithm backends are created with the correct width/height.
        algo_bridge_.set_sensor_dimensions(info.width, info.height);
        status_conn_->setText(tr("Connected: %1").arg(info.generation_name.isEmpty()
                                                           ? info.integrator
                                                           : info.generation_name));
        // Phase 2 panels: populate from the now-connected camera.
        settings_->biases_panel()->on_camera_connected(&camera_);
        settings_->roi_panel()->on_camera_connected(&camera_);
        settings_->esp_panel()->on_camera_connected(&camera_);
        settings_->trigger_panel()->on_camera_connected(&camera_);
        // Phase 5 panels.
        settings_->preprocessing_panel()->on_camera_connected(&camera_);
        // Enable config / bias file actions on live cameras only.
        const bool live = !info.is_file;
        a_save_cfg_->setEnabled(live);
        a_load_cfg_->setEnabled(live);
        a_save_biases_->setEnabled(live);
        a_load_biases_->setEnabled(live);
        a_roi_drag_->setEnabled(live);
        a_record_start_->setEnabled(live);
        a_record_stop_->setEnabled(false);
        a_export_->setEnabled(true);
        // Phase 3: show/hide playback bar.
        if (auto* pb = findChild<QDockWidget*>("PlaybackDock")) {
            pb->setVisible(info.is_file);
        }
        // Phase 10: re-install the temporal plot callback on the new camera.
        // The previous camera (if any) was destroyed in teardown(), which
        // invalidated the old CallbackId.
        temporal_cd_cb_id_.reset();
        if (temporal_plot_) {
            temporal_plot_->set_sensor_geometry(info.width, info.height);
        }
        install_temporal_callback();
        install_algo_callback();
        camera_.start();
    });
    connect(&camera_, &CameraController::disconnected, this, [this]() {
        // Phase 10: the camera was torn down in teardown(); the CD callback
        // was removed by the SDK. Just clear our ID so a fresh one can be
        // registered on the next connect.
        temporal_cd_cb_id_.reset();
        algo_cd_cb_id_.reset();
        prev_frame_ts_ = 0;
        settings_->information_panel()->clear();
        settings_->statistics_panel()->clear();
        settings_->devices_panel()->set_connected(false);
        settings_->biases_panel()->on_camera_disconnected();
        settings_->roi_panel()->on_camera_disconnected();
        settings_->esp_panel()->on_camera_disconnected();
        settings_->trigger_panel()->on_camera_disconnected();
        settings_->preprocessing_panel()->on_camera_disconnected();
        status_conn_->setText(tr("Disconnected"));
        status_rate_->setText(tr("— ev/s"));
        status_ts_->setText(tr("t: —"));
        a_save_cfg_->setEnabled(false);
        a_load_cfg_->setEnabled(false);
        a_save_biases_->setEnabled(false);
        a_load_biases_->setEnabled(false);
        a_roi_drag_->setEnabled(false);
        a_roi_drag_->setChecked(false);
        a_record_start_->setEnabled(false);
        a_record_stop_->setEnabled(false);
        a_export_->setEnabled(false);
        display_->set_roi_drag_mode(false);
        display_->clear();
        if (auto* pb = findChild<QDockWidget*>("PlaybackDock")) {
            pb->setVisible(false);
        }
        playback_controls_->activate(false);
    });
    connect(&camera_, &CameraController::started, this, [this]() {
        status_rec_->setText(tr("Streaming"));
    });
    connect(&camera_, &CameraController::stopped, this, [this]() {
        if (!recorder_.is_recording()) {
            status_rec_->setText(tr("Idle"));
        }
    });
    connect(&camera_, &CameraController::error, this, [this](const QString& msg) {
        QMessageBox::warning(this, tr("Camera error"), msg);
    });
    connect(&camera_, &CameraController::runtime_warning, this, [this](const QString& msg) {
        status_rec_->setText(tr("Stopped"));
        statusBar()->showMessage(msg, 5000);
    });

    // Frame pipeline -> display
    connect(camera_.frame_pipeline(), &FramePipeline::frame_ready, this,
            [this](QImage frame, Metavision::timestamp ts) {
                process_algo_results(frame);
                display_->set_frame(frame);
                settings_->statistics_panel()->set_timestamp(ts);
                status_ts_->setText(QStringLiteral("t: %1 s").arg(ts / 1.0e6, 0, 'f', 3));
                if (prev_frame_ts_ > 0 && ts > prev_frame_ts_) {
                    const double fps = 1.0e6 / static_cast<double>(ts - prev_frame_ts_);
                    settings_->statistics_panel()->set_fps(fps);
                }
                prev_frame_ts_ = ts;
            });

    // Statistics controller -> panel + status bar
    connect(camera_.statistics(), &StatisticsController::rate_updated, this,
            [this](double rate, double peak, Metavision::timestamp t) {
                settings_->statistics_panel()->set_rate(rate, peak, t);
                if (rate >= 1.0e6) {
                    status_rate_->setText(QStringLiteral("%1 Mev/s")
                                              .arg(rate / 1.0e6, 0, 'f', 2));
                } else if (rate >= 1.0e3) {
                    status_rate_->setText(QStringLiteral("%1 kev/s")
                                              .arg(rate / 1.0e3, 0, 'f', 2));
                } else {
                    status_rate_->setText(QStringLiteral("%1 ev/s").arg(rate, 0, 'f', 0));
                }
            });
    connect(camera_.statistics(), &StatisticsController::on_off_updated,
            settings_->statistics_panel(), &StatisticsPanel::set_on_off);

    // Display panel -> pipeline
    connect(settings_->display_panel(), &DisplayPanel::color_palette_changed, this,
            &MainWindow::update_palettes);
    connect(settings_->display_panel(), &DisplayPanel::frame_mode_changed, this,
            [this](int idx) {
                // Map frame mode to accumulation time. Only feasible modes
                // (Diff/Integration/Time Decay) are enabled in the menu; the
                // others are disabled and won't reach here.
                const int us[] = {10000, 50000, 30000, 30000, 10000, 10000, 10000};
                const int accumulation = (idx >= 0 && idx < 7) ? us[idx] : 10000;
                camera_.frame_pipeline()->set_accumulation_time_us(accumulation);
                statusBar()->showMessage(
                    tr("Frame mode %1 (accumulation %2 us)").arg(idx).arg(accumulation), 3000);
            });
    connect(settings_->display_panel(), &DisplayPanel::accumulation_time_changed_us,
            this, [this](int us) {
                camera_.frame_pipeline()->set_accumulation_time_us(us);
            });

    // ROI panel <-> display widget (Phase 2)
    auto* roi = settings_->roi_panel();
    connect(display_, &EventDisplayWidget::roi_dragged,
            roi, &RoiPanel::set_roi_from_drag);
    connect(roi, &RoiPanel::roi_applied, display_, &EventDisplayWidget::set_roi_overlay);

    // Phase 3 — recorder.
    connect(&recorder_, &RecorderController::recording_started, this,
            [this](const QString& path) {
                status_rec_->setText(tr("REC: %1").arg(QFileInfo(path).fileName()));
                a_record_start_->setEnabled(false);
                a_record_stop_->setEnabled(true);
            });
    connect(&recorder_, &RecorderController::recording_stopped, this,
            [this](const QString& path) {
                status_rec_->setText(tr("Idle"));
                a_record_start_->setEnabled(camera_.is_connected() && !camera_.is_file_source());
                a_record_stop_->setEnabled(false);
                statusBar()->showMessage(tr("Recording saved: %1").arg(path), 5000);
            });
    connect(&recorder_, &RecorderController::elapsed, this, &MainWindow::on_record_elapsed);
    connect(&recorder_, &RecorderController::error, this, [this](const QString& msg) {
        QMessageBox::warning(this, tr("Recording"), msg);
    });

    // Phase 3 — playback.
    connect(&playback_, &PlaybackController::opened, this,
            [this](Metavision::timestamp) {
                playback_controls_->activate(true);
            });
    connect(&playback_, &PlaybackController::error, this, [this](const QString& msg) {
        statusBar()->showMessage(msg, 4000);
    });

    // Phase 5 — file converter signals are wired directly inside FileToolsPanel.

    // Panel info/error messages -> status bar / message box.
    auto forward = [this](const QString& msg, bool isError) {
        forward_panel_message(msg, isError);
    };
    connect(settings_->biases_panel(),  &BiasesPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->biases_panel(),  &BiasesPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(roi, &RoiPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(roi, &RoiPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(settings_->esp_panel(),     &EspPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->esp_panel(),     &EspPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    connect(settings_->trigger_panel(), &TriggerPanel::info_message,  this,
            [forward](const QString& m) { forward(m, false); });
    connect(settings_->trigger_panel(), &TriggerPanel::error_message, this,
            [forward](const QString& m) { forward(m, true); });
    if (auto* pp = settings_->preprocessing_panel()) {
        connect(pp, &PreprocessingPanel::info_message, this,
                [forward](const QString& m) { forward(m, false); });
        connect(pp, &PreprocessingPanel::error_message, this,
                [forward](const QString& m) { forward(m, true); });
    }
    if (auto* ap = settings_->algorithms_panel()) {
        connect(ap, &AlgorithmsPanel::info_message, this,
                [forward](const QString& m) { forward(m, false); });
        connect(ap, &AlgorithmsPanel::algorithm_toggled, this,
                [this](const QString& name, bool on) {
                    statusBar()->showMessage(
                        tr("%1: %2").arg(name).arg(on ? tr("enabled") : tr("disabled")), 3000);
                });
    }
}

void MainWindow::update_palettes(int index) {
    Metavision::ColorPalette p = Metavision::ColorPalette::Dark;
    switch (index) {
        case 1: p = Metavision::ColorPalette::Light; break;
        case 2: p = Metavision::ColorPalette::CoolWarm; break;
        case 3: p = Metavision::ColorPalette::Gray; break;
        default: break;
    }
    camera_.frame_pipeline()->set_color_palette(p);
}

// ---------------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------------

void MainWindow::on_open_file() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open event file"), QString(),
        tr("Event files (*.raw *.hdf5 *.h5 *.dat);;All files (*)"));
    if (path.isEmpty()) {
        return;
    }
    on_file_opened_for_playback(path);
}

void MainWindow::on_file_opened_for_playback(const QString& path) {
    // Route through the playback controller so it can capture duration and
    // start the position probe timer.
    if (!playback_.open_file(path, 1.0)) {
        QMessageBox::warning(this, tr("Open file"),
                             tr("Failed to open event file:\n%1").arg(path));
    }
}

void MainWindow::on_connect_first() {
    if (!camera_.connect_first_available()) {
        QMessageBox::information(this, tr("Connect"),
                                 tr("No live camera available."));
    }
}

void MainWindow::on_disconnect() {
    camera_.disconnect();
}

void MainWindow::on_save_config() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save camera config"), QString(),
        tr("Camera config (*.json);;All files (*)"));
    if (path.isEmpty()) return;
    QString err;
    if (!config_.save_to_file(&camera_, path, err)) {
        QMessageBox::warning(this, tr("Save config"),
                             err.isEmpty() ? tr("Failed to save camera config to:\n%1").arg(path)
                                           : err);
    } else {
        statusBar()->showMessage(tr("Config saved: %1").arg(path), 4000);
    }
}

void MainWindow::on_load_config() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load camera config"), QString(),
        tr("Camera config (*.json);;All files (*)"));
    if (path.isEmpty()) return;
    QString err;
    if (!config_.load_from_file(&camera_, path, err)) {
        QMessageBox::warning(this, tr("Load config"),
                             err.isEmpty() ? tr("Failed to load camera config from:\n%1").arg(path)
                                           : err);
        return;
    }
    // Refresh the panels so they reflect the newly-applied state.
    settings_->biases_panel()->on_camera_connected(&camera_);
    settings_->roi_panel()->on_camera_connected(&camera_);
    settings_->esp_panel()->on_camera_connected(&camera_);
    settings_->trigger_panel()->on_camera_connected(&camera_);
    statusBar()->showMessage(tr("Config loaded: %1").arg(path), 4000);
}

void MainWindow::on_save_biases() {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save biases"), QString(),
        tr("Bias files (*.bias);;All files (*)"));
    if (path.isEmpty()) return;
    settings_->biases_panel()->save_to_file(path);
}

void MainWindow::on_load_biases() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load biases"), QString(),
        tr("Bias files (*.bias);;All files (*)"));
    if (path.isEmpty()) return;
    settings_->biases_panel()->load_from_file(path);
}

void MainWindow::on_toggle_roi_drag(bool on) {
    display_->set_roi_drag_mode(on);
    statusBar()->showMessage(on ? tr("ROI drag mode on — draw a rectangle on the display.")
                                : tr("ROI drag mode off."), 3000);
}

void MainWindow::on_record_start() {
    if (recorder_.is_recording()) return;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Record to file"), QString(),
        tr("RAW files (*.raw);;All files (*)"));
    if (path.isEmpty()) return;
    recorder_.start(&camera_, path);
}

void MainWindow::on_record_stop() {
    recorder_.stop();
}

void MainWindow::on_record_elapsed(std::chrono::seconds s) {
    const auto hrs = std::chrono::duration_cast<std::chrono::hours>(s).count();
    const auto mins = std::chrono::duration_cast<std::chrono::minutes>(s).count() % 60;
    const auto secs = s.count() % 60;
    const QString base = tr("REC");
    status_rec_->setText(QStringLiteral("%1 %2:%3:%4")
                             .arg(base)
                             .arg(hrs, 2, 10, QLatin1Char('0'))
                             .arg(mins, 2, 10, QLatin1Char('0'))
                             .arg(secs, 2, 10, QLatin1Char('0')));
}

void MainWindow::on_export_dialog() {
    if (export_dialog_) {
        export_dialog_->show();
        export_dialog_->raise();
        export_dialog_->activateWindow();
    }
}

void MainWindow::on_apply_preset(int index) {
    QString err;
    if (!config_.apply_preset(&camera_, index, err)) {
        QMessageBox::warning(this, tr("Apply preset"), err);
        return;
    }
    // Refresh Phase 2 panels.
    settings_->biases_panel()->on_camera_connected(&camera_);
    settings_->roi_panel()->on_camera_connected(&camera_);
    settings_->esp_panel()->on_camera_connected(&camera_);
    settings_->trigger_panel()->on_camera_connected(&camera_);
    statusBar()->showMessage(tr("Preset applied."), 3000);
}

void MainWindow::on_open_temporal_plot() {
    if (!temporal_plot_) {
        temporal_plot_ = new TemporalPlotWindow(this);
        temporal_plot_->setAttribute(Qt::WA_DeleteOnClose);
        connect(temporal_plot_, &QObject::destroyed, this, [this]() {
            // Remove the SDK callback so the SDK thread stops copying events
            // for a window that no longer exists. The QPointer itself is
            // auto-nulled by Qt after this slot returns.
            remove_temporal_callback();
        });
        // Pass the current sensor geometry so axis_position is clamped to a
        // valid range and the Y-axis can be drawn to scale.
        if (camera_.is_connected()) {
            const auto& info = camera_.sensor_info();
            temporal_plot_->set_sensor_geometry(info.width, info.height);
        }
        // Install the CD callback if a camera is already connected. If not,
        // it will be installed when CameraController::connected fires.
        install_temporal_callback();
    }
    temporal_plot_->show();
    temporal_plot_->raise();
    temporal_plot_->activateWindow();
}

void MainWindow::install_temporal_callback() {
    if (temporal_cd_cb_id_) return;          // already installed on this camera
    if (!temporal_plot_) return;             // no window to feed
    auto* cam = camera_.camera_handle();
    if (!cam) return;                        // no live camera
    temporal_cd_cb_id_ = cam->cd().add_callback(
        [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (b == e) return;
            // Throttle: limit to ~20 posts/second to avoid flooding the GUI
            // event loop at high event rates (10-100 Mev/s). Without this,
            // the queued invocations accumulate faster than the GUI thread
            // can process them, making the entire application unresponsive.
            const Metavision::timestamp cur_ts = (e - 1)->t;
            const Metavision::timestamp last = temporal_last_post_us_.load(std::memory_order_relaxed);
            if (cur_ts - last < 50000) return;  // 50ms minimum interval
            temporal_last_post_us_.store(cur_ts, std::memory_order_relaxed);

            // Downsample: cap at 5000 events per batch to limit copy size
            // and GUI thread processing per queued invocation.
            const std::size_t count = static_cast<std::size_t>(e - b);
            auto copy = std::make_shared<std::vector<Metavision::EventCD>>();
            if (count > 5000) {
                const std::size_t stride = count / 5000;
                copy->reserve(count / stride + 1);
                for (std::size_t i = 0; i < count; i += stride) {
                    copy->push_back(b[i]);
                }
            } else {
                copy->assign(b, e);
            }
            QMetaObject::invokeMethod(this, [this, copy]() {
                if (temporal_plot_) {
                    temporal_plot_->push_events(copy->data(),
                                                copy->data() + copy->size());
                }
            }, Qt::QueuedConnection);
        });
}

void MainWindow::remove_temporal_callback() {
    if (!temporal_cd_cb_id_) return;
    if (auto* cam = camera_.camera_handle()) {
        try { cam->cd().remove_callback(*temporal_cd_cb_id_); } catch (...) {}
    }
    temporal_cd_cb_id_.reset();
}

// ---------------------------------------------------------------------------
// Algorithm event/result pipeline (design §3.8, §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::install_algo_callback() {
    if (algo_cd_cb_id_) return;              // already installed
    auto* cam = camera_.camera_handle();
    if (!cam) return;
    algo_cd_cb_id_ = cam->cd().add_callback(
        [this](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (b == nullptr || e == nullptr || b >= e) return;
            try {
                // Push events to every live algorithm instance. AlgoInstance
                // is internally mutex-guarded, so this is safe from the SDK
                // data thread. The reinterpret_cast inside AlgoBackend is
                // valid because gui_algo::Event is layout-compatible with
                // Metavision::EventCD (POD x,y,p,t).
                auto instances = algo_bridge_.list_live();
                for (auto& inst : instances) {
                    if (inst->is_enabled()) {
                        inst->push_events(b, e);
                    }
                }
            } catch (...) {}

            // Throttled, downsampled feed to the XYT 3D window. The
            // SpaceTimeDisplay is a QOpenGLWidget and must only be touched
            // from the GUI thread, so we marshal via QMetaObject::invokeMethod.
            // (Same throttle/downsample pattern as install_temporal_callback.)
            if (xyt_display_) {
                const Metavision::timestamp cur_ts = (e - 1)->t;
                const Metavision::timestamp last =
                    algo_last_xyt_post_us_.load(std::memory_order_relaxed);
                if (cur_ts - last >= 50000) {  // 50ms minimum interval
                    algo_last_xyt_post_us_.store(cur_ts, std::memory_order_relaxed);
                    const std::size_t count = static_cast<std::size_t>(e - b);
                    auto copy = std::make_shared<std::vector<Metavision::EventCD>>();
                    if (count > 5000) {
                        const std::size_t stride = count / 5000;
                        copy->reserve(count / stride + 1);
                        for (std::size_t i = 0; i < count; i += stride) {
                            copy->push_back(b[i]);
                        }
                    } else {
                        copy->assign(b, e);
                    }
                    QMetaObject::invokeMethod(this, [this, copy]() {
                        if (xyt_display_) {
                            xyt_display_->push_events(copy->data(),
                                                      copy->data() + copy->size());
                        }
                    }, Qt::QueuedConnection);
                }
            }
        });
}

void MainWindow::remove_algo_callback() {
    if (!algo_cd_cb_id_) return;
    if (auto* cam = camera_.camera_handle()) {
        try { cam->cd().remove_callback(*algo_cd_cb_id_); } catch (...) {}
    }
    algo_cd_cb_id_.reset();
}

void MainWindow::process_algo_results(QImage& frame) {
    // Iterate a snapshot of live instances and pull each one's latest result.
    // For Overlay/Replace instances we mutate @p frame in place; for
    // Standalone instances we route the result frame (if any) to the
    // dedicated child display widget. Passive instances are skipped.
    auto instances = algo_bridge_.list_live();
    for (auto& inst : instances) {
        if (!inst->is_enabled()) continue;
        const auto mode = inst->info().display_mode;
        if (mode == AlgoDisplayMode::Passive) continue;

        AlgoResult r;
        try {
            r = inst->pull_result();
        } catch (...) {
            continue;
        }

        if (mode == AlgoDisplayMode::Overlay) {
            // Convert AlgoResult overlay primitives into FrameAnnotator calls.
            // Boxes: tracked-object boxes with optional id.
            if (!r.boxes.empty()) {
                std::vector<FrameAnnotator::Box> boxes;
                boxes.reserve(r.boxes.size());
                for (const auto& b : r.boxes) {
                    FrameAnnotator::Box box;
                    box.rect = QRect(b.x, b.y, b.w, b.h);
                    box.id = b.id;
                    boxes.push_back(std::move(box));
                }
                annotator_.draw_boxes(frame, boxes);
            }
            // Lines.
            if (!r.lines.empty()) {
                std::vector<QLineF> lines;
                lines.reserve(r.lines.size());
                for (const auto& l : r.lines) {
                    lines.emplace_back(QPointF(l.x1, l.y1), QPointF(l.x2, l.y2));
                }
                annotator_.draw_lines(frame, lines);
            }
            // Points.
            if (!r.points.empty()) {
                std::vector<QPointF> pts;
                pts.reserve(r.points.size());
                for (const auto& p : r.points) {
                    pts.emplace_back(p.x, p.y);
                }
                annotator_.draw_points(frame, pts);
            }
            // Circles.
            if (!r.circles.empty()) {
                std::vector<std::pair<QPointF, double>> circs;
                circs.reserve(r.circles.size());
                for (const auto& c : r.circles) {
                    circs.emplace_back(QPointF(c.cx, c.cy), c.r);
                }
                annotator_.draw_circles(frame, circs);
            }
            // Text labels.
            if (!r.texts.empty()) {
                for (const auto& t : r.texts) {
                    annotator_.draw_text(frame, QString::fromStdString(t.text),
                                         QPoint(t.x, t.y));
                }
            }
            continue;
        }

        if (mode == AlgoDisplayMode::Replace) {
            // Replace the main display frame with the algorithm output.
            if (r.has_frame && !r.frame.empty()) {
                frame = mat_to_qimage(r.frame);
            }
            continue;
        }

        if (mode == AlgoDisplayMode::Standalone) {
            const std::string& name = inst->info().name;
            if (name == "time_surface") {
                if (time_surface_display_ && r.has_frame && !r.frame.empty()) {
                    QImage q = mat_to_qimage(r.frame);
                    QMetaObject::invokeMethod(time_surface_display_.data(),
                        "set_frame", Qt::QueuedConnection, Q_ARG(QImage, q));
                }
            } else if (name == "event_to_video") {
                if (event_to_video_display_ && r.has_frame && !r.frame.empty()) {
                    QImage q = mat_to_qimage(r.frame);
                    QMetaObject::invokeMethod(event_to_video_display_.data(),
                        "set_frame", Qt::QueuedConnection, Q_ARG(QImage, q));
                }
            } else if (name == "freq_detector") {
                if (freq_detector_label_) {
                    // Build a multi-line status string: header line + one
                    // line per detected light source (frequency in Hz).
                    QString text = QString::fromStdString(r.status);
                    for (const auto& t : r.texts) {
                        text += QStringLiteral("\n  ");
                        text += QString::fromStdString(t.text);
                    }
                    QMetaObject::invokeMethod(this, [this, text]() {
                        if (freq_detector_label_) {
                            freq_detector_label_->setText(text);
                        }
                    }, Qt::QueuedConnection);
                }
            }
            // Other Standalone algorithms (e.g. xyt_visualizer, flow_statistics,
            // isi_analyzer, calibration modules) are driven through their own
            // dedicated windows and callbacks; pull_result is a no-op for them
            // or the result is consumed elsewhere.
        }
    }
}

void MainWindow::on_intrinsic_wizard() {
    if (!calibration_wizard_) {
        calibration_wizard_ = new CalibrationWizard(this);
        calibration_wizard_->setAttribute(Qt::WA_DeleteOnClose);
        connect(calibration_wizard_, &QObject::destroyed, this, [this]() {
            calibration_wizard_ = nullptr;
        });
    }
    calibration_wizard_->set_camera(&camera_);
    calibration_wizard_->set_display(display_);
    calibration_wizard_->show_intrinsic();
}

void MainWindow::on_add_display_window() {
    if (!multi_window_) {
        multi_window_ = std::make_unique<MultiWindowManager>(this);
    }
    auto* w = multi_window_->add_display(tr("Display %1").arg(multi_window_->window_count() + 1));
    if (w) {
        // Feed the new display from the same frame pipeline as the main
        // display. The connection is auto-disconnected when w is destroyed
        // (Qt destroys receivers on deletion).
        connect(camera_.frame_pipeline(), &FramePipeline::frame_ready,
                w, &EventDisplayWidget::set_frame);
        statusBar()->showMessage(tr("Added display window (%1 total).")
            .arg(multi_window_->window_count()), 3000);
    }
}

void MainWindow::on_tile_windows() {
    if (multi_window_) multi_window_->tile();
    else statusBar()->showMessage(tr("No display windows open."), 3000);
}

void MainWindow::on_save_layout() {
    if (!layout_manager_) return;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save Layout"), {}, tr("Layout JSON (*.json);;All Files (*)"));
    if (!path.isEmpty()) {
        if (layout_manager_->save(path)) {
            statusBar()->showMessage(tr("Layout saved to %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Save Layout"), tr("Could not save layout."));
        }
    }
}

void MainWindow::on_load_layout() {
    if (!layout_manager_) return;
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Load Layout"), {}, tr("Layout JSON (*.json);;All Files (*)"));
    if (!path.isEmpty()) {
        if (layout_manager_->load(path)) {
            statusBar()->showMessage(tr("Layout loaded from %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Load Layout"), tr("Could not load layout."));
        }
    }
}

void MainWindow::on_reset_layout() {
    if (layout_manager_) layout_manager_->reset_layout();
    statusBar()->showMessage(tr("Layout reset to default."), 3000);
}

void MainWindow::forward_panel_message(const QString& msg, bool isError) {
    if (isError) {
        QMessageBox::warning(this, tr("Camera control"), msg);
    } else {
        statusBar()->showMessage(msg, 4000);
    }
}

void MainWindow::on_refresh_devices() {
    const auto sources = camera_.list_online_sources();
    settings_->devices_panel()->refresh_sources(sources);
    if (sources.empty()) {
        statusBar()->showMessage(tr("No cameras detected."), 3000);
    } else {
        statusBar()->showMessage(tr("%1 camera(s) found.").arg(sources.size()), 3000);
    }
}

void MainWindow::on_about() {
    QMessageBox::about(this, tr("About GUI for openEB"),
        tr("<h3>GUI for openEB</h3>"
           "<p>Phases 1-12 — Qt 6 + OpenEB 5.2.0.</p>"
           "<p>Camera discovery, real-time OpenGL event display, statistics, "
           "Bias / ROI / ESP / Trigger panels, recording & playback, HDF5 / AVI "
           "export, JSON config with presets, OpenEB filter-chain preprocessing, "
           "file conversion tools, calibration wizard, temporal plot, "
           "30 self-developed CV/analytics algorithms with overlay/replace/"
           "standalone display modes, XYT 3D point cloud, and more.</p>"
           "<p>See doc/design.md for the full roadmap.</p>"));
}

// ---------------------------------------------------------------------------
// Standalone algorithm windows (design §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::on_open_time_surface() {
    if (!time_surface_window_) {
        time_surface_window_ = new QWidget(this, Qt::Window);
        time_surface_window_->setWindowTitle(tr("Time Surface"));
        time_surface_window_->setAttribute(Qt::WA_DeleteOnClose);
        // Host an EventDisplayWidget so the Standalone Time Surface result
        // (a cv::Mat from TimeSurfaceBackend) can be shown as a real frame
        // rather than a blank placeholder QLabel. The result is pushed via
        // process_algo_results() each time frame_ready fires.
        time_surface_display_ = new EventDisplayWidget(time_surface_window_);
        auto* layout = new QVBoxLayout(time_surface_window_);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(time_surface_display_);
        time_surface_algo_ = algo_bridge_.find_live("time_surface");
        if (!time_surface_algo_) time_surface_algo_ = algo_bridge_.create("time_surface");
        if (time_surface_algo_) time_surface_algo_->set_enabled(true);
        // Null out QPointer members when the window is closed so the
        // process_algo_results() guard `if (time_surface_display_)` works.
        connect(time_surface_window_, &QObject::destroyed, this, [this]() {
            time_surface_display_ = nullptr;
            time_surface_algo_.reset();
        });
        time_surface_window_->resize(640, 480);
    }
    time_surface_window_->show();
    time_surface_window_->raise();
    statusBar()->showMessage(tr("Time Surface window opened"), 2000);
}

void MainWindow::on_open_xyt_view() {
    if (!xyt_display_) {
        xyt_display_ = new SpaceTimeDisplay(this);
        xyt_display_->setWindowFlag(Qt::Window, true);
        xyt_display_->setWindowTitle(tr("XYT 3D Event Cloud"));
        xyt_display_->setAttribute(Qt::WA_DeleteOnClose);
        if (camera_.is_connected()) {
            const auto& info = camera_.sensor_info();
            xyt_display_->set_sensor_geometry(info.width, info.height);
        }
        xyt_algo_ = algo_bridge_.find_live("xyt_visualizer");
        if (!xyt_algo_) xyt_algo_ = algo_bridge_.create("xyt_visualizer");
        if (xyt_algo_) xyt_algo_->set_enabled(true);
        // The QOpenGLWidget is itself the window; on close we must null
        // xyt_display_ (QPointer does this) and clear xyt_algo_ so the
        // throttled push in install_algo_callback()'s `if (xyt_display_)`
        // guard stops dispatching.
        connect(xyt_display_, &QObject::destroyed, this, [this]() {
            xyt_algo_.reset();
        });
        xyt_display_->resize(800, 600);
        // Install the algo callback if a camera is already connected so the
        // XYT window starts receiving events immediately.
        install_algo_callback();
    }
    xyt_display_->show();
    xyt_display_->raise();
    statusBar()->showMessage(tr("XYT 3D window opened"), 2000);
}

void MainWindow::on_open_event_to_video() {
    if (!event_to_video_window_) {
        event_to_video_window_ = new QWidget(this, Qt::Window);
        event_to_video_window_->setWindowTitle(tr("Event -> Video"));
        event_to_video_window_->setAttribute(Qt::WA_DeleteOnClose);
        // Host an EventDisplayWidget so the Standalone E2VID result (cv::Mat
        // from EventToVideoBackend) can be shown as a real reconstructed
        // intensity frame.
        event_to_video_display_ = new EventDisplayWidget(event_to_video_window_);
        auto* layout = new QVBoxLayout(event_to_video_window_);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->addWidget(event_to_video_display_);
        e2v_algo_ = algo_bridge_.find_live("event_to_video");
        if (!e2v_algo_) e2v_algo_ = algo_bridge_.create("event_to_video");
        if (e2v_algo_) e2v_algo_->set_enabled(true);
        connect(event_to_video_window_, &QObject::destroyed, this, [this]() {
            event_to_video_display_ = nullptr;
            e2v_algo_.reset();
        });
        event_to_video_window_->resize(640, 480);
    }
    event_to_video_window_->show();
    event_to_video_window_->raise();
    statusBar()->showMessage(tr("Event -> Video window opened"), 2000);
}

void MainWindow::on_open_freq_detector() {
    if (!freq_detector_window_) {
        freq_detector_window_ = new QWidget(this, Qt::Window);
        freq_detector_window_->setWindowTitle(tr("Frequency Detector"));
        freq_detector_window_->setAttribute(Qt::WA_DeleteOnClose);
        // FreqDetectorBackend emits a textual status string (peak frequency,
        // confidence, etc.); use a monospace QLabel so the result, posted
        // via process_algo_results(), renders as readable text.
        freq_detector_label_ = new QLabel(freq_detector_window_);
        QFont f(QStringLiteral("Monospace"));
        f.setStyleHint(QFont::TypeWriter);
        f.setPointSize(10);
        freq_detector_label_->setFont(f);
        freq_detector_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        freq_detector_label_->setText(tr("Waiting for events..."));
        auto* layout = new QVBoxLayout(freq_detector_window_);
        layout->setContentsMargins(4, 4, 4, 4);
        layout->addWidget(freq_detector_label_);
        freq_algo_ = algo_bridge_.find_live("freq_detector");
        if (!freq_algo_) freq_algo_ = algo_bridge_.create("freq_detector");
        if (freq_algo_) freq_algo_->set_enabled(true);
        connect(freq_detector_window_, &QObject::destroyed, this, [this]() {
            freq_detector_label_ = nullptr;
            freq_algo_.reset();
        });
        freq_detector_window_->resize(640, 480);
    }
    freq_detector_window_->show();
    freq_detector_window_->raise();
    statusBar()->showMessage(tr("Frequency Detector window opened"), 2000);
}

} // namespace gui
