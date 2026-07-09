// gui/main_window.cpp

#include "main_window.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QCloseEvent>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QSet>
#include <QSettings>
#include <QStatusBar>
#include <QStyle>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWindow>

#include <cmath>
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
    // Remove the native WM title bar and replace it with a custom title bar
    // widget (CustomTitleBar) whose background color follows the application
    // theme.  This is the same approach VSCode uses (frameless BrowserWindow
    // + custom titlebar part drawn in HTML/CSS).  The native title bar color
    // is controlled by the window manager and cannot be changed reliably
    // from Qt, so we draw our own.
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    setWindowTitle(QStringLiteral("EB plus"));
    resize(1280, 720);

    display_ = new EventDisplayWidget(this);
    setCentralWidget(display_);

    // Theme controller must be attached before the menu is built (so the
    // Theme menu actions reflect the persisted choice) and before the
    // SettingsPanel is built. The controller also persists settings via
    // QSettings.
    theme_.set_target(this);

    settings_ = new SettingsPanel(&algo_bridge_, &file_converter_, this);
    settings_dock_ = new QDockWidget(tr("Settings"), this);
    settings_dock_->setObjectName("SettingsDock");
    settings_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    settings_dock_->setWidget(settings_);
    settings_dock_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetClosable);
    addDockWidget(Qt::RightDockWidgetArea, settings_dock_);
    settings_dock_->setMinimumWidth(320);

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
    build_toolbar();
    build_status_bar();
    wire_signals();

    // --- Frameless-window title bar built on the native QMenuBar ---
    // With Qt::FramelessWindowHint the QMenuBar sits at the very top of
    // the window.  We add the window title (left corner) and min/max/close
    // buttons (right corner) directly to the menu bar, and install an
    // event filter so dragging the menu bar moves the window
    // (QWindow::startSystemMove).  This is simpler and more reliable than
    // setMenuWidget().
    build_title_bar_controls();

    // Edge/corner resize handles — since the window is frameless, the WM
    // no longer provides resize borders.  These 8 invisible grips call
    // QWindow::startSystemResize() so the WM handles the actual resize.
    using P = ResizeGrip::Position;
    for (auto p : {P::Left, P::Right, P::Top, P::Bottom,
                   P::TopLeft, P::TopRight, P::BottomLeft, P::BottomRight}) {
        auto* grip = new ResizeGrip(p, this);
        resize_grips_.push_back(grip);
    }

    // Capture the factory layout (all docks in their default positions) so
    // reset_layout() can restore it. Must be called before load_default()
    // which may overlay a user-customized layout.
    layout_manager_->capture_default();

    // Try restoring the previous layout (silent on failure).
    layout_manager_->load_default();

    // Force the sidebar visible on startup regardless of the restored
    // layout state — the user explicitly requested it be shown by default.
    // Sync the toggle action to match.
    if (settings_dock_) {
        settings_dock_->setVisible(true);
    }
    if (a_toggle_sidebar_) {
        a_toggle_sidebar_->setChecked(true);
    }
    update_sidebar_tab_visibility();

    // Populate the device list on startup.
    on_refresh_devices();
}

MainWindow::~MainWindow() = default;

void MainWindow::closeEvent(QCloseEvent* event) {
    if (layout_manager_) layout_manager_->save_default();

    // ---- Phase 1: stop all data sources BEFORE deleting child widgets. ----
    // The CD callback and frame pipeline run on the camera's data thread.
    // If we delete child widgets (AlgoWindow displays, xyt_display_, etc.)
    // while the pipeline is still active, queued invokeMethod lambdas may
    // dereference already-deleted widgets.
    remove_algo_callback();   // explicitly remove the CD callback
    recorder_.stop();
    camera_.disconnect();     // stops the data thread + frame pipeline

    // ---- Phase 2: delete lazily-created child windows. ----
    // Their destroyed() handlers access MainWindow members (e.g. camera_,
    // algo_cd_cb_id_) that would already be gone by the time ~QWidget
    // runs its automatic child cleanup.
    if (calibration_wizard_) {
        delete calibration_wizard_;
        // calibration_wizard_ is nulled by the destroyed signal handler.
    }
    // Standalone algorithm windows — use close() (not delete) so that
    // closeEvent fires, the `closing` signal is emitted, and the cleanup
    // handler disables the AlgoInstance. WA_DeleteOnClose schedules
    // deleteLater() for actual destruction; any remaining docks are also
    // deleted by Qt's parent-child cleanup when MainWindow is destroyed.
    // Collect pointers first and clear the hash: the `closing` handler
    // calls algo_windows_.remove(name), which would invalidate iterators
    // if we were still iterating.
    std::vector<QPointer<AlgoWindow>> algo_ptrs;
    algo_ptrs.reserve(algo_windows_.size());
    for (auto it = algo_windows_.begin(); it != algo_windows_.end(); ++it) {
        if (it.value()) algo_ptrs.push_back(it.value());
    }
    algo_windows_.clear();
    for (auto& ptr : algo_ptrs) {
        if (ptr) ptr->close();
    }
    if (xyt_display_) { delete xyt_display_.data(); }
    // Pre-delete child widgets that hold raw pointers to MainWindow members.
    // Without this, ~QObject child cleanup runs after member destructors,
    // causing use-after-free in widget destructors.
    if (export_dialog_) { delete export_dialog_; export_dialog_ = nullptr; }
    event->accept();
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    // Reposition the frameless-window resize grips to the new edges.
    const QRect r = rect();
    for (auto* grip : resize_grips_) {
        if (grip) grip->reposition(r);
    }
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event) {
    // Drag the window by dragging the menu bar (title bar).
    if (obj == menuBar()) {
        if (event->type() == QEvent::MouseButtonPress) {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::LeftButton) {
                if (auto* handle = windowHandle()) {
                    handle->startSystemMove();
                }
            }
        } else if (event->type() == QEvent::MouseButtonDblClick) {
            auto* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::LeftButton) {
                if (isMaximized()) showNormal();
                else showMaximized();
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}

/// @brief Adds the window title (left) and min/max/close buttons (right) to
/// the menu bar, turning it into a frameless-window title bar.
void MainWindow::build_title_bar_controls() {
    auto* mb = menuBar();

    // Left corner: window title (bold, inverse color of the title bar bg).
    auto* title_label = new QLabel(QStringLiteral("EB plus"), mb);
    title_label->setObjectName("WindowTitleLabel");
    title_label->setStyleSheet(QStringLiteral(
        "background: transparent; border: none; padding: 0 8px;"
        "font-weight: bold;"
    ));
    mb->setCornerWidget(title_label, Qt::TopLeftCorner);

    // Right corner: window control buttons.
    auto* controls = new QWidget(mb);
    controls->setStyleSheet(QStringLiteral(
        "QWidget { background: transparent; }"
        "QPushButton { background: transparent; border: none; "
        "  min-width: 40px; max-width: 40px; }"
        "QPushButton:hover { background: rgba(128,128,128,60); }"
        "QPushButton:pressed { background: rgba(128,128,128,100); }"
    ));
    auto* layout = new QHBoxLayout(controls);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    auto make_btn = [this](QStyle::StandardPixmap pix, const QString& tip) {
        auto* btn = new QPushButton;
        btn->setIcon(style()->standardIcon(pix));
        btn->setIconSize(QSize(16, 16));
        btn->setToolTip(tip);
        btn->setFocusPolicy(Qt::NoFocus);
        btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        return btn;
    };

    auto* btn_min   = make_btn(QStyle::SP_TitleBarMinButton,  tr("Minimize"));
    auto* btn_max   = make_btn(QStyle::SP_TitleBarMaxButton,  tr("Maximize"));
    auto* btn_close = make_btn(QStyle::SP_TitleBarCloseButton, tr("Close"));

    connect(btn_min, &QPushButton::clicked, this, [this]() {
        showMinimized();
    });
    connect(btn_max, &QPushButton::clicked, this, [this]() {
        if (isMaximized()) showNormal();
        else showMaximized();
    });
    connect(btn_close, &QPushButton::clicked, this, [this]() {
        close();
    });

    layout->addWidget(btn_min);
    layout->addWidget(btn_max);
    layout->addWidget(btn_close);
    mb->setCornerWidget(controls, Qt::TopRightCorner);

    // Let the menu bar act as a drag region for the frameless window.
    mb->installEventFilter(this);
}

// ---------------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------------

void MainWindow::build_menus() {
    auto* mb = menuBar();
    // The native menu bar (GNOME global menu, macOS screen menu) does not
    // respect Qt stylesheets, so the theme cannot recolor it. Force the
    // in-process Qt menu bar so the theme stylesheet applies to the menu
    // bar and its dropdowns — this keeps the top bar consistent with the
    // selected background color and text color.
    mb->setNativeMenuBar(false);

    // File
    auto* m_file = mb->addMenu(tr("&File"));
    m_file->addAction(tr("&Open File..."), this, &MainWindow::on_open_file,
                      QKeySequence::Open);
    m_recent_files_ = m_file->addMenu(tr("Open &Recent"));
    build_recent_files_menu();
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

    // View — includes the sidebar (settings dock) toggle with shortcut.
    auto* m_view = mb->addMenu(tr("&View"));
    a_toggle_sidebar_ = m_view->addAction(tr("Toggle Sidebar"));
    a_toggle_sidebar_->setCheckable(true);
    a_toggle_sidebar_->setChecked(true);
    a_toggle_sidebar_->setShortcut(QKeySequence("Ctrl+Shift+S"));
    // The toggled -> setVisible connection is wired in wire_signals() after
    // the sidebar tab is created, so the tab visibility stays in sync.
    auto* pb_toggle = m_view->addAction(tr("Toggle Playback Panel"));
    pb_toggle->setCheckable(true);
    pb_toggle->setChecked(false);
    pb_toggle->setShortcut(QKeySequence("Ctrl+Shift+P"));
    connect(pb_toggle, &QAction::toggled, this, [this](bool on) {
        auto* dock = findChild<QDockWidget*>("PlaybackDock");
        if (dock) dock->setVisible(on);
    });
    m_view->addAction(tr("Reset Layout"), this, &MainWindow::on_reset_layout);
    m_view->addAction(tr("Save Layout..."), this, &MainWindow::on_save_layout);
    m_view->addAction(tr("Load Layout..."), this, &MainWindow::on_load_layout);
    m_view->addSeparator();
    m_view->addAction(tr("&Fullscreen"), this,
                      [this]() {
                          if (isFullScreen()) showNormal();
                          else showFullScreen();
                      }, QKeySequence("F11"));

    // Theme — background color + light/dark mode. Moved here from the
    // sidebar so the controls are at the top of the window where appearance
    // settings are typically found. The controller builds its own submenu
    // with radio actions for each color and mode.
    auto* m_theme = mb->addMenu(tr("&Theme"));
    theme_.build_menu(m_theme);

    // Camera — only display-interaction controls and presets that are NOT
    // in the sidebar's Devices panel. Connect/Disconnect/Refresh live in
    // the DevicesPanel (基础功能 tab).
    auto* m_cam = mb->addMenu(tr("&Camera"));
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

    // Preprocess menu and Frame Mode menu have been removed — all
    // preprocessing stage toggles live in the PreprocessingPanel and all
    // frame mode selection lives in the DisplayPanel (both in the sidebar's
    // 基础功能 tab). This eliminates duplication with the sidebar.

    // The Algorithm menu has been removed — all algorithm configuration
    // (enable, ROI, parameters, configure) lives in the sidebar's "算法模块"
    // tab (AlgorithmsPanel).

    // Calibration (Phase 9) — launches the wizard lazily.
    m_calibration_ = mb->addMenu(tr("&Calibration"));
    m_calibration_->addAction(tr("&Intrinsic Wizard..."), this, &MainWindow::on_intrinsic_wizard);

    // Tools (design §5.5) — window management only. Algorithm-specific
    // windows are opened from the sidebar's "算法模块" tab, not duplicated here.
    m_tools_ = mb->addMenu(tr("&Tools"));
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

    // Help
    auto* m_help = mb->addMenu(tr("&Help"));
    m_help->addAction(tr("&About"), this, &MainWindow::on_about);
    m_help->addAction(tr("About &Qt"), this, &QApplication::aboutQt);
}

void MainWindow::build_toolbar() {
    // Main toolbar — prominent, always-visible toggles for the most-used
    // view controls. The sidebar toggle here is the "醒目位置按钮" requested
    // by the user: it's at the top of the window, checkable, and uses a
    // standard icon so it stands out from the menu items.
    main_toolbar_ = addToolBar(tr("Main"));
    main_toolbar_->setObjectName("MainToolbar");
    main_toolbar_->setMovable(false);
    main_toolbar_->setIconSize({20, 20});
    main_toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);

    // Sidebar toggle — the primary action on the toolbar.
    if (a_toggle_sidebar_) {
        // Make the toolbar button show the checked state visually.
        auto* btn = qobject_cast<QToolButton*>(main_toolbar_->widgetForAction(a_toggle_sidebar_));
        if (btn) btn->setCheckable(true);
        main_toolbar_->addAction(a_toggle_sidebar_);
    }

    main_toolbar_->addSeparator();

    // Tile / reset layout — quick access for multi-window arrangement.
    auto* a_tile = main_toolbar_->addAction(style()->standardIcon(QStyle::SP_TitleBarNormalButton),
                                            tr("Tile"));
    connect(a_tile, &QAction::triggered, this, &MainWindow::on_tile_windows);

    auto* a_reset = main_toolbar_->addAction(style()->standardIcon(QStyle::SP_BrowserReload),
                                             tr("Reset Layout"));
    connect(a_reset, &QAction::triggered, this, &MainWindow::on_reset_layout);

    main_toolbar_->addSeparator();

    // Fullscreen toggle on the toolbar for discoverability.
    auto* a_fs = main_toolbar_->addAction(style()->standardIcon(QStyle::SP_TitleBarMaxButton),
                                          tr("Fullscreen"));
    connect(a_fs, &QAction::triggered, this, [this]() {
        if (isFullScreen()) showNormal();
        else showFullScreen();
    });

    // --- Right-edge sidebar tab (collapsed marker) ---
    // A thin vertical toolbar docked to the right edge. It's visible ONLY
    // when the sidebar is hidden, giving the user a clear way to bring it
    // back without hunting through the View menu. The arrow icon makes it
    // obvious that clicking it expands something to the right.
    sidebar_tab_ = new QToolBar(tr("Sidebar Tab"), this);
    sidebar_tab_->setObjectName("SidebarTab");
    sidebar_tab_->setMovable(false);
    sidebar_tab_->setFloatable(false);
    sidebar_tab_->setIconSize({22, 22});
    sidebar_tab_->setToolButtonStyle(Qt::ToolButtonIconOnly);
    addToolBar(Qt::RightToolBarArea, sidebar_tab_);
    a_show_sidebar_ = sidebar_tab_->addAction(
        style()->standardIcon(QStyle::SP_ArrowLeft), tr("Show Sidebar"));
    a_show_sidebar_->setToolTip(tr("Show Sidebar (Ctrl+Shift+S)"));
    auto* sb_btn = qobject_cast<QToolButton*>(sidebar_tab_->widgetForAction(a_show_sidebar_));
    if (sb_btn) {
        sb_btn->setAutoRaise(true);
        sb_btn->setFixedWidth(28);
    }
    connect(a_show_sidebar_, &QAction::triggered, this, [this]() {
        if (settings_dock_) {
            settings_dock_->setVisible(true);
            if (a_toggle_sidebar_) {
                QSignalBlocker b(a_toggle_sidebar_);
                a_toggle_sidebar_->setChecked(true);
            }
        }
        update_sidebar_tab_visibility();
    });
    sidebar_tab_->setVisible(false);  // hidden until sidebar is closed
}

void MainWindow::update_sidebar_tab_visibility() {
    const bool dock_visible = settings_dock_ && settings_dock_->isVisible();
    if (sidebar_tab_) {
        sidebar_tab_->setVisible(!dock_visible);
    }
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
    // Sidebar toggle (menu action + toolbar button) <-> settings dock.
    // The dock's visibilityChanged signal keeps the toggle action checked
    // state and the collapsed-side-tab visibility in sync no matter how the
    // dock is shown/hidden (menu, toolbar, dock X button, or layout restore).
    if (a_toggle_sidebar_ && settings_dock_) {
        connect(a_toggle_sidebar_, &QAction::toggled, this, [this](bool on) {
            if (settings_dock_) settings_dock_->setVisible(on);
        });
        connect(settings_dock_, &QDockWidget::visibilityChanged, this,
                [this](bool visible) {
                    if (a_toggle_sidebar_) {
                        QSignalBlocker b(a_toggle_sidebar_);
                        a_toggle_sidebar_->setChecked(visible);
                    }
                    update_sidebar_tab_visibility();
                });
    }

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
        // Keep the XYT 3D display's sensor geometry in sync with the live
        // camera so point coordinates are normalized correctly. Without this
        // (e.g. when the XYT window was opened before a camera connected, or
        // after a reconnect) points render at raw pixel scale and the cloud
        // is clipped/garbled.
        if (xyt_display_) {
            xyt_display_->set_sensor_geometry(info.width, info.height);
        }
        // Reset ALL algorithm instances to clear temporal state from a
        // previous session (previous file or live camera). Without this,
        // stateful algorithms (EventToVideo's log_intensity_/current_t_/
        // last_frame_t_, InteractingMaps' I_map_, etc.) carry over stale
        // values, causing:
        //   - Wrong decay calculations (dt_us based on old current_t_)
        //   - Stale reconstructions / gray screens
        //   - NaN propagation from diverged state
        // This also covers the case where a new raw file starts at t=0 but
        // current_t_ is still at the previous file's end time → no new
        // events would update current_t_ (e.t > current_t_ is false) → the
        // algorithm freezes on stale output. See doc/gui_optimization.md §8.
        for (auto& inst : algo_bridge_.list_live()) {
            inst->reset();
        }
        // Clear the XYT 3D display buffer so stale events from the previous
        // session don't linger in the point cloud. Combined with the
        // time_window-based Z normalization (xyt_visualizer.h render()),
        // the new session starts with an empty cloud that fills naturally.
        if (xyt_display_) {
            xyt_display_->clear();
        }
        install_algo_callback();
        camera_.start();
        // Sync UI to FramePipeline's current (persisted) values so the
        // DisplayPanel and PlaybackControls reflect the actual fps /
        // accumulation / fps_limit after a reconnect or file reopen.
        if (auto* fp = camera_.frame_pipeline()) {
            settings_->display_panel()->set_accumulation_time_ms(
                fp->accumulation_time_us() / 1000.0);
            settings_->display_panel()->set_fps(fp->fps());
            settings_->display_panel()->set_fps_limit(fp->fps_limit());
            playback_controls_->on_time_window_changed(fp->accumulation_time_us());
            playback_controls_->on_frame_rate_changed(fp->fps());
            playback_controls_->on_fps_limit_changed(fp->fps_limit());
        }
    });
    connect(&camera_, &CameraController::disconnected, this, [this]() {
        // Explicitly remove the CD callback before clearing the ID, so the
        // SDK data thread stops calling our lambda before any MainWindow
        // members are destroyed.
        remove_algo_callback();
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
                // Forward the annotated frame to any multi-window child displays
                // so they see the same overlays/replacements as the main view.
                emit annotated_frame_ready(frame);
                settings_->statistics_panel()->set_timestamp(ts);
                status_ts_->setText(QStringLiteral("t: %1 s").arg(ts / 1.0e6, 0, 'f', 3));
                if (prev_frame_ts_ > 0 && ts > prev_frame_ts_) {
                    const double fps = 1.0e6 / static_cast<double>(ts - prev_frame_ts_);
                    settings_->statistics_panel()->set_fps(fps);
                }
                prev_frame_ts_ = ts;
            });

    // File playback: feed the events in each accumulation window to algorithm
    // instances + XYT display, synchronously with the displayed frame. This
    // signal is emitted just before frame_ready (see FileFrameGenerator::render_frame),
    // so by the time the frame_ready handler runs process_algo_results(), the
    // algorithms have already processed this window's events and their results
    // are ready to be pulled. For live cameras this signal is never emitted
    // (CDFrameGenerator path) and the SDK CD callback feeds algorithms instead.
    connect(camera_.frame_pipeline(), &FramePipeline::events_window_ready,
            this, &MainWindow::on_events_window_ready);

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

    // Display panel -> FramePipeline (single source of truth for display params)
    connect(settings_->display_panel(), &DisplayPanel::color_palette_changed, this,
            &MainWindow::update_palettes);
    connect(settings_->display_panel(), &DisplayPanel::frame_mode_changed, this,
            [this](int idx) {
                // Frame modes map to accumulation-time presets (the underlying
                // CDFrameGenerator only supports accumulation rendering).
                // 0: Diff Frame (10 ms), 1: Integration (50 ms), 2: Time Decay (30 ms).
                const int us[] = {10000, 50000, 30000};
                const int accumulation = (idx >= 0 && idx < 3) ? us[idx] : 10000;
                camera_.frame_pipeline()->set_accumulation_time_us(accumulation);
                // No need to manually sync the DisplayPanel — the
                // accumulation_time_changed signal from FramePipeline updates
                // both the DisplayPanel and PlaybackControls automatically.
                statusBar()->showMessage(
                    tr("Frame mode %1 (accumulation %2 us)").arg(idx).arg(accumulation), 3000);
            });
    connect(settings_->display_panel(), &DisplayPanel::accumulation_time_changed_us,
            this, [this](int us) {
                camera_.frame_pipeline()->set_accumulation_time_us(us);
            });
    connect(settings_->display_panel(), &DisplayPanel::fps_changed, this,
            [this](int fps) {
                camera_.frame_pipeline()->set_fps(static_cast<std::uint16_t>(fps));
            });
    connect(settings_->display_panel(), &DisplayPanel::fps_limit_changed, this,
            [this](int limit) {
                camera_.frame_pipeline()->set_fps_limit(static_cast<std::uint16_t>(limit));
            });

    // FramePipeline -> both UIs (DisplayPanel + PlaybackControls).
    // When the pipeline's params change (from either UI or programmatically),
    // sync both panels. The blocked setters prevent feedback loops.
    {
        auto* fp = camera_.frame_pipeline();
        connect(fp, &FramePipeline::accumulation_time_changed, this,
                [this](Metavision::timestamp us) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_accumulation_time_ms(us / 1000.0);
                    if (playback_controls_)
                        playback_controls_->on_time_window_changed(us);
                });
        connect(fp, &FramePipeline::fps_changed, this,
                [this](std::uint16_t fps) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_fps(static_cast<int>(fps));
                    if (playback_controls_)
                        playback_controls_->on_frame_rate_changed(static_cast<unsigned>(fps));
                });
        connect(fp, &FramePipeline::fps_limit_changed, this,
                [this](std::uint16_t limit) {
                    if (settings_ && settings_->display_panel())
                        settings_->display_panel()->set_fps_limit(static_cast<int>(limit));
                    if (playback_controls_)
                        playback_controls_->on_fps_limit_changed(static_cast<unsigned>(limit));
                });
    }

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
    // Parameter sync (time_window/fps/fps_limit) is handled by the
    // FramePipeline signal connections above — both the DisplayPanel and
    // PlaybackControls listen to the same FramePipeline signals.
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
                    const auto key = name.toStdString();
                    if (!on) {
                        // Close the AlgoWindow if open (its closing handler will
                        // disable the instance and uncheck the sidebar checkbox —
                        // both are idempotent given the blocker in set_algo_enabled).
                        auto it = algo_windows_.find(key);
                        if (it != algo_windows_.end() && it.value()) it.value()->close();
                        if (key == "xyt_visualizer" && xyt_display_) xyt_display_->close();
                    }
                });
        // When an algorithm is enabled from the sidebar, open its AlgoWindow
        // so Standalone algorithms have a display and Overlay algorithms get
        // their ROI zoom view. Without this the sidebar-enabled algorithm
        // produces no visible output (the root cause of "侧栏算法不生效").
        connect(ap, &AlgorithmsPanel::open_algo_window_requested, this,
                [this](const std::string& name) {
                    on_open_algo_window(name);
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
    if (!playback_.open_file(path)) {
        QMessageBox::warning(this, tr("Open file"),
                             tr("Failed to open event file:\n%1").arg(path));
        return;
    }
    add_recent_file(path);
}

void MainWindow::build_recent_files_menu() {
    if (!m_recent_files_) return;
    m_recent_files_->clear();
    QSettings s;
    const QStringList recent = s.value("recentFiles").toStringList();
    if (recent.isEmpty()) {
        auto* a = m_recent_files_->addAction(tr("(none)"));
        a->setEnabled(false);
        return;
    }
    for (const QString& path : recent) {
        // Show just the file name in the menu, full path as tooltip.
        const QString name = QFileInfo(path).fileName();
        auto* a = m_recent_files_->addAction(name);
        a->setToolTip(path);
        connect(a, &QAction::triggered, this, [this, path]() {
            on_open_recent_file(path);
        });
    }
}

void MainWindow::add_recent_file(const QString& path) {
    if (path.isEmpty()) return;
    QSettings s;
    QStringList recent = s.value("recentFiles").toStringList();
    // Remove duplicates (case-insensitive on Windows, exact on Linux), then
    // prepend and cap at 10.
    recent.removeAll(path);
    recent.prepend(path);
    while (recent.size() > 10) {
        recent.removeLast();
    }
    s.setValue("recentFiles", recent);
    build_recent_files_menu();
}

void MainWindow::on_open_recent_file(const QString& path) {
    if (!QFileInfo::exists(path)) {
        QMessageBox::warning(this, tr("Open recent"),
                             tr("File no longer exists:\n%1").arg(path));
        // Remove stale entry.
        QSettings s;
        QStringList recent = s.value("recentFiles").toStringList();
        recent.removeAll(path);
        s.setValue("recentFiles", recent);
        build_recent_files_menu();
        return;
    }
    on_file_opened_for_playback(path);
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
        // Pre-fill the source path with the file currently open for playback
        // (if any) so the user doesn't have to re-browse to it.
        export_dialog_->set_source(playback_.current_file());
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

// ---------------------------------------------------------------------------
// Algorithm event/result pipeline (design §3.8, §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::install_algo_callback() {
    if (algo_cd_cb_id_) return;              // already installed
    auto* cam = camera_.camera_handle();
    if (!cam) return;
    // For file sources, the SDK CD callback delivers ALL events in ~10ms
    // (real_time_playback(false)), which is completely out of sync with the
    // FileFrameGenerator's timer-based frame playback. Algorithms would
    // process everything before the first frame is even shown. Instead, file
    // sources feed events to algorithms via the FramePipeline::events_window_ready
    // signal (see on_events_window_ready), synchronized with each displayed
    // frame. We still install the SDK callback here so the event buffer in
    // FileFrameGenerator gets filled (FramePipeline::add_events forwards to it).
    const bool file_source = camera_.is_file_source();
    algo_cd_cb_id_ = cam->cd().add_callback(
        [this, file_source](const Metavision::EventCD* b, const Metavision::EventCD* e) {
            if (b == nullptr || e == nullptr || b >= e) return;
            // Only the live-camera path feeds algorithms from the SDK thread.
            // File playback feeds algorithms from the GUI thread via
            // events_window_ready (synchronous with frame display).
            if (!file_source) {
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
            }

            // Throttled, downsampled feed to the XYT 3D window. The
            // SpaceTimeDisplay is a QOpenGLWidget and must only be touched
            // from the GUI thread, so we marshal via QMetaObject::invokeMethod.
            // For file sources the XYT feed is driven by events_window_ready
            // instead (synchronized with playback), so skip it here.
            if (!file_source && xyt_display_) {
                const Metavision::timestamp cur_ts = (e - 1)->t;
                const Metavision::timestamp last =
                    algo_last_xyt_post_us_.load(std::memory_order_relaxed);
                if (cur_ts - last >= 16000) {  // 16ms ≈ 60 FPS
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
                            // Sync time_window from algo parameter in case
                            // the user changed it in the AlgoWindow.
                            if (xyt_algo_) {
                                const auto tw = xyt_algo_->get_param("time_window_us");
                                if (!tw.empty()) {
                                    xyt_display_->set_time_window_ms(
                                        static_cast<float>(std::stoi(tw)) / 1000.0f);
                                }
                            }
                            xyt_display_->push_events(copy->data(),
                                                      copy->data() + copy->size());
                        }
                    }, Qt::QueuedConnection);
                }
            }
        });
}

void MainWindow::on_events_window_ready(std::shared_ptr<std::vector<Metavision::EventCD>> events,
                                        Metavision::timestamp ts) {
    if (!events || events->empty()) return;

    // ======================================================================
    // TIMESTAMP SCALING FOR ALGORITHM EVENT FEEDING (FILE PLAYBACK ONLY)
    // ======================================================================
    // Problem: During file playback, FileFrameGenerator replays events at a
    // user-controlled rate:
    //   playback_rate = fps * accumulation_window_us / 1e6
    //
    //   - Slow motion (rate < 1): e.g. fps=30, window=100us → rate=0.003.
    //     Each displayed frame contains events spanning only 100us of real
    //     event time, but 33ms of wall-clock time pass between frames.
    //   - Fast forward (rate > 1): e.g. fps=60, window=33000us → rate=1.98.
    //     Each frame spans 33000us of event time but only 16ms of wall-clock.
    //
    // Temporal algorithms (InteractingMaps decay-tau, E2VID recurrent state,
    // XYT time-window normalization, etc.) rely on the RELATIVE time between
    // events to evolve their internal state. When fed real event timestamps
    // in slow motion, the algorithm sees time barely advancing (100us per
    // frame) while its decay tau is 500ms — the state never evolves, causing
    // flickering (InteractingMaps), flat gray output (BardowVariational), or
    // a collapsed Z axis (XYT 3D display shows a single plane instead of a
    // cloud).
    //
    // Solution: Scale event timestamps by 1/playback_rate before feeding them
    // to algorithm instances. This makes the algorithm perceive time
    // advancing at wall-clock rate (1/fps per frame), exactly as it would
    // with a live camera at the same fps. The algorithm's temporal parameters
    // (decay tau, time windows) then behave as intended.
    //
    //     scaled_t = real_t / playback_rate
    //
    // IMPORTANT — DISPLAY USES REAL TIMESTAMPS:
    //   The XYT 3D display and any algorithm-output annotations that use
    //   timestamps for visualization MUST use real event timestamps, not
    //   scaled ones. The XYT display is fed the ORIGINAL (unscaled) events
    //   below. Algorithm output frames/overlays produced from scaled-time
    //   input do NOT carry timestamps in their results (AlgoResult contains
    //   spatial data: colored_events with x/y/p, trajectories with x/y, and
    //   cv::Mat frames — no timestamp fields), so no back-conversion is
    //   needed for display. The `ts` parameter in frame_ready is the real
    //   cursor timestamp from FileFrameGenerator and is used directly for
    //   status-bar/position display.
    //
    // This scaling is NOT a bug — it is an intentional adaptation for
    // rate-controlled file playback. See doc/gui_optimization.md §8.
    // ======================================================================

    // Compute playback rate from FramePipeline's current parameters.
    double playback_rate = 1.0;
    if (auto* fp = camera_.frame_pipeline()) {
        const double fps = static_cast<double>(fp->fps());
        const double window_us = static_cast<double>(fp->accumulation_time_us());
        playback_rate = fps * window_us / 1.0e6;
    }
    // Guard against division by zero.
    if (playback_rate < 1.0e-9) playback_rate = 1.0;

    // Feed algorithm instances. If playback_rate ≈ 1.0 (real-time), feed the
    // original events directly. Otherwise, create a scaled-timestamp copy.
    // The 0.5% tolerance avoids unnecessary copies for near-real-time rates.
    const bool need_scaling = std::fabs(playback_rate - 1.0) > 0.005;
    try {
        auto instances = algo_bridge_.list_live();
        if (!instances.empty()) {
            if (need_scaling) {
                // Create a copy with scaled timestamps: scaled_t = real_t / rate.
                // This makes the algorithm see time advancing at wall-clock rate.
                auto scaled = std::make_shared<std::vector<Metavision::EventCD>>();
                scaled->reserve(events->size());
                const double inv_rate = 1.0 / playback_rate;
                for (const auto& ev : *events) {
                    Metavision::EventCD scaled_ev = ev;
                    scaled_ev.t = static_cast<Metavision::timestamp>(
                        static_cast<double>(ev.t) * inv_rate);
                    scaled->push_back(scaled_ev);
                }
                for (auto& inst : instances) {
                    if (inst->is_enabled()) {
                        inst->push_events(scaled->data(),
                                          scaled->data() + scaled->size());
                    }
                }
            } else {
                for (auto& inst : instances) {
                    if (inst->is_enabled()) {
                        inst->push_events(events->data(),
                                          events->data() + events->size());
                    }
                }
            }
        }
    } catch (...) {}

    // Feed the XYT 3D display with REAL (unscaled) timestamps — the Z axis
    // must show the actual event time, not the scaled playback time.
    // Same downsampling as the live path so a large accumulation window
    // doesn't overwhelm the OpenGL point cloud.
    if (xyt_display_) {
        const std::size_t count = events->size();
        auto copy = std::make_shared<std::vector<Metavision::EventCD>>();
        if (count > 5000) {
            const std::size_t stride = count / 5000;
            copy->reserve(count / stride + 1);
            for (std::size_t i = 0; i < count; i += stride) {
                copy->push_back((*events)[i]);
            }
        } else {
            *copy = *events;
        }
        if (xyt_algo_) {
            const auto tw = xyt_algo_->get_param("time_window_us");
            if (!tw.empty()) {
                xyt_display_->set_time_window_ms(
                    static_cast<float>(std::stoi(tw)) / 1000.0f);
            }
        }
        xyt_display_->push_events(copy->data(), copy->data() + copy->size());
    }
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
        // Surface the flood-guard auto-disable so the user knows why an algo
        // stopped producing output (design §5.6.7). Re-enabling it from the
        // sidebar's "算法模块" tab clears the overload and restarts processing.
        if (inst->is_overloaded()) {
            const QString nm = QString::fromStdString(inst->info().display_name);
            statusBar()->showMessage(
                tr("%1 auto-disabled: event rate too high (re-enable to retry)").arg(nm), 5000);
            // Reflect the disabled state in the AlgoWindow status label.
            auto wit = algo_windows_.find(inst->info().name);
            if (wit != algo_windows_.end() && wit.value()) {
                wit.value()->set_status_text(
                    tr("AUTO-DISABLED: event flooding detected. Re-enable from the sidebar."));
            }
            continue;
        }
        if (!inst->is_enabled()) continue;
        const auto mode = inst->info().display_mode;

        // Passive algorithms (in-place event filters like noise_filter /
        // hot_pixel_filter) don't draw overlays or replace the frame, but if
        // the user opened an AlgoWindow for one we still need to pull_result
        // and update the status text so the window doesn't stay stuck on
        // "Waiting for events..." forever.
        if (mode == AlgoDisplayMode::Passive) {
            auto wit = algo_windows_.find(inst->info().name);
            if (wit == algo_windows_.end() || !wit.value()) continue;
            AlgoResult r;
            try {
                r = inst->pull_result();
            } catch (...) {
                continue;
            }
            if (!r.status.empty()) {
                QString text = QString::fromStdString(r.status);
                for (const auto& t : r.texts) {
                    text += QStringLiteral("\n  ");
                    text += QString::fromStdString(t.text);
                }
                QPointer<AlgoWindow> w = wit.value();
                QMetaObject::invokeMethod(this, [w, text]() {
                    if (w) w->set_status_text(text);
                }, Qt::QueuedConnection);
            }
            continue;
        }

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
            // Colored points (optical-flow HSV visualization).
            if (!r.colored_points.empty()) {
                std::vector<std::pair<QPointF, QColor>> pts;
                pts.reserve(r.colored_points.size());
                for (const auto& p : r.colored_points) {
                    pts.emplace_back(QPointF(p.x, p.y),
                                     QColor(p.r, p.g, p.b));
                }
                annotator_.draw_colored_points(frame, pts, 3.0);
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
            // Colored events (orientation/direction per-event coloring).
            if (!r.colored_events.empty()) {
                std::vector<std::tuple<int, int, QColor>> cevs;
                cevs.reserve(r.colored_events.size());
                for (const auto& ce : r.colored_events) {
                    cevs.emplace_back(ce.event.x, ce.event.y,
                                      QColor(ce.r, ce.g, ce.b));
                }
                annotator_.draw_colored_events(frame, cevs);
            }
            // Trajectories (cluster history paths).
            if (!r.trajectories.empty()) {
                std::vector<std::pair<int, std::vector<QPointF>>> trajs;
                trajs.reserve(r.trajectories.size());
                for (const auto& t : r.trajectories) {
                    std::vector<QPointF> pts;
                    pts.reserve(t.points.size());
                    for (const auto& pt : t.points) {
                        pts.emplace_back(pt.x, pt.y);
                    }
                    trajs.emplace_back(t.id, std::move(pts));
                }
                annotator_.draw_trajectories(frame, trajs, QColor(0, 255, 0));
            }
            // Aux frame (Hough θ-ρ / per-pixel accumulator space).
            // Routed to the AlgoWindow's display widget if one is open, so
            // the user can see the accumulator state alongside the overlay.
            if (r.has_aux_frame && !r.aux_frame.empty()) {
                auto wit = algo_windows_.find(inst->info().name);
                if (wit != algo_windows_.end() && wit.value()) {
                    QPointer<EventDisplayWidget> disp = wit.value()->frame_display();
                    if (disp) {
                        QImage q = mat_to_qimage(r.aux_frame);
                        QMetaObject::invokeMethod(this, [disp, q]() {
                            if (disp) disp->set_frame(q);
                        }, Qt::QueuedConnection);
                    }
                }
            }
            // ROI zoom view (design §5.6.6): if the algo has ROI enabled and an
            // AlgoWindow with an EventDisplayWidget is open, crop the ROI
            // region from the annotated main frame and push it to the window.
            // This gives the user a zoomed-in view of just the ROI region
            // (with the algorithm's overlay drawn on it), which is otherwise
            // hard to inspect on the sensor-scale main display.
            const std::string en_str = inst->get_param("roi_enabled");
            const bool roi_on = (en_str == "true" || en_str == "1");
            if (roi_on) {
                auto parse_int = [](const std::string& s, int def) -> int {
                    try { return s.empty() ? def : std::stoi(s); }
                    catch (...) { return def; }
                };
                int sw = 1280, sh = 720;
                if (camera_.is_connected()) {
                    const auto& sinfo = camera_.sensor_info();
                    sw = sinfo.width; sh = sinfo.height;
                }
                int rx = parse_int(inst->get_param("roi_x"), -1);
                int ry = parse_int(inst->get_param("roi_y"), -1);
                int rw = parse_int(inst->get_param("roi_w"), 256);
                int rh = parse_int(inst->get_param("roi_h"), 256);
                int aw = (rw <= 0) ? sw : std::min(rw, sw);
                int ah = (rh <= 0) ? sh : std::min(rh, sh);
                int ax = (rx < 0) ? (sw - aw) / 2
                                  : std::min(std::max(0, rx), sw - aw);
                int ay = (ry < 0) ? (sh - ah) / 2
                                  : std::min(std::max(0, ry), sh - ah);
                auto wit = algo_windows_.find(inst->info().name);
                if (wit != algo_windows_.end() && wit.value()) {
                    // Use QPointer so the lambda safely no-ops if the
                    // AlgoWindow's display widget is destroyed between
                    // scheduling and execution (e.g. user closes/undocks the
                    // dock while a frame is in flight). Without this the raw
                    // pointer would dangle and crash on dereference.
                    QPointer<EventDisplayWidget> disp = wit.value()->frame_display();
                    if (disp) {
                        QRect roi_rect(ax, ay, aw, ah);
                        QImage zoom = frame.copy(roi_rect);
                        QMetaObject::invokeMethod(this, [disp, zoom]() {
                            if (disp) disp->set_frame(zoom);
                        }, Qt::QueuedConnection);
                    }
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
            // Route results to the AlgoWindow (design §5.6.6). Frame-producing
            // algos (time_surface, event_to_video, isi_analyzer, background_mask)
            // use an EventDisplayWidget; text-producing algos (freq_detector,
            // flow_statistics, auto_bias, etc.) use the default status QLabel.
            // xyt_visualizer is handled separately via SpaceTimeDisplay.
            auto wit = algo_windows_.find(name);
            if (wit != algo_windows_.end() && wit.value()) {
                QPointer<AlgoWindow> w = wit.value();
                if (r.has_frame && !r.frame.empty()) {
                    // QPointer protects against the dock being closed/undocked
                    // (and its display widget destroyed) between scheduling
                    // and execution of the queued call.
                    QPointer<EventDisplayWidget> disp = w->frame_display();
                    if (disp) {
                        QImage q = mat_to_qimage(r.frame);
                        QMetaObject::invokeMethod(this, [disp, q]() {
                            if (disp) disp->set_frame(q);
                        }, Qt::QueuedConnection);
                    }
                }
                if (!r.status.empty()) {
                    QString text = QString::fromStdString(r.status);
                    for (const auto& t : r.texts) {
                        text += QStringLiteral("\n  ");
                        text += QString::fromStdString(t.text);
                    }
                    QMetaObject::invokeMethod(this, [w, text]() {
                        if (w) w->set_status_text(text);
                    }, Qt::QueuedConnection);
                }
            }
            // xyt_visualizer is driven through its own SpaceTimeDisplay window
            // (design §5.6.1); pull_result is a no-op for it.
        }
    }

    // Draw the ROI rectangle of any enabled self-developed algorithm so the
    // user can see which region is being processed (design §5.6.6: all
    // self-developed algos support ROI, defaulting to center 128×128).
    draw_roi_overlays(frame);
}

void MainWindow::draw_roi_overlays(QImage& frame) {
    // All self-developed algorithms support ROI (design §5.6.6). Iterate the
    // live instances and draw a rectangle for each enabled one with
    // roi_enabled=true. The overlay coordinates are at sensor scale.
    int sensor_w = 1280, sensor_h = 720;
    if (camera_.is_connected()) {
        const auto& info = camera_.sensor_info();
        sensor_w = info.width;
        sensor_h = info.height;
    }
    auto parse = [](const std::string& s, int def) -> int {
        try { return s.empty() ? def : std::stoi(s); }
        catch (...) { return def; }
    };
    std::vector<QRect> boxes;
    std::vector<std::pair<QString, QPoint>> labels;  // (text, pos)
    auto instances = algo_bridge_.list_live();
    for (auto& inst : instances) {
        if (!inst->is_enabled()) continue;
        // Only self-developed algorithms carry the roi_* params (design §5.6.6).
        const AlgoInfo& info = inst->info();
        if (info.source != "self") continue;
        const std::string en = inst->get_param("roi_enabled");
        if (en != "true" && en != "1") continue;
        const int rx = parse(inst->get_param("roi_x"), -1);
        const int ry = parse(inst->get_param("roi_y"), -1);
        const int rw = parse(inst->get_param("roi_w"), 256);
        const int rh = parse(inst->get_param("roi_h"), 256);
        // Compute bounds (mirrors ProcessRegion::compute).
        const int aw = (rw <= 0) ? sensor_w : std::min(rw, sensor_w);
        const int ah = (rh <= 0) ? sensor_h : std::min(rh, sensor_h);
        const int ax = (rx < 0) ? (sensor_w - aw) / 2
                                 : std::min(std::max(0, rx), sensor_w - aw);
        const int ay = (ry < 0) ? (sensor_h - ah) / 2
                                 : std::min(std::max(0, ry), sensor_h - ah);
        boxes.emplace_back(ax, ay, aw, ah);
        labels.emplace_back(QString::fromStdString(info.name) + " ROI " +
                            QString::number(aw) + "x" + QString::number(ah),
                            QPoint(ax + 4, ay + 14));
    }
    if (!boxes.empty()) {
        annotator_.draw_bboxes(frame, boxes, QColor(255, 255, 0));
        for (const auto& [text, pos] : labels) {
            annotator_.draw_text(frame, text, pos, QColor(255, 255, 0));
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
        // Feed the new display the annotated frame (after process_algo_results)
        // so child windows see the same overlays/replacements as the main view.
        connect(this, &MainWindow::annotated_frame_ready,
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
            update_sidebar_tab_visibility();
            statusBar()->showMessage(tr("Layout loaded from %1").arg(path), 3000);
        } else {
            QMessageBox::warning(this, tr("Load Layout"), tr("Could not load layout."));
        }
    }
}

void MainWindow::on_reset_layout() {
    if (layout_manager_) layout_manager_->reset_layout();
    update_sidebar_tab_visibility();
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
           "<p>Qt 6 + OpenEB 5.2.0.</p>"
           "<p>Camera discovery, real-time OpenGL event display, statistics, "
           "Bias / ROI / ESP / Trigger panels, recording & playback, HDF5 / AVI "
           "export, JSON config with presets, OpenEB filter-chain preprocessing, "
           "file conversion tools, calibration wizard, "
           "30 self-developed CV/analytics algorithms with overlay/replace/"
           "standalone display modes, XYT 3D point cloud, and more.</p>"));
}

// ---------------------------------------------------------------------------
// Standalone algorithm windows (design §5.6.1)
// ---------------------------------------------------------------------------

void MainWindow::on_open_algo_window(const std::string& algo_name) {
    // Reuse an existing window if one is already open for this algorithm.
    auto it = algo_windows_.find(algo_name);
    if (it != algo_windows_.end() && it.value()) {
        it.value()->show();
        it.value()->raise();
        it.value()->activateWindow();
        return;
    }

    // Create the AlgoWindow (a QDockWidget). The constructor finds/creates
    // the live AlgoInstance and enables it.
    auto* w = new AlgoWindow(&algo_bridge_, algo_name, this);
    if (!w->instance()) {
        // Unknown algorithm — clean up and bail out.
        delete w;
        statusBar()->showMessage(tr("Unknown algorithm: %1")
                                     .arg(QString::fromStdString(algo_name)), 3000);
        return;
    }
    algo_windows_[algo_name] = w;

    // For Standalone frame-producing algorithms, install an EventDisplayWidget
    // so process_algo_results() can push frames to it. The frame size may be
    // the ROI dimensions (e.g. event_to_video at 128×128) and the widget
    // scales it to fit.
    //
    // For self-developed Overlay algorithms (design §5.6.6) we also install
    // an EventDisplayWidget: it shows a zoomed view of just the ROI region
    // with the algorithm's overlay primitives drawn on it. Without this the
    // user cannot easily inspect the algorithm's result inside the small ROI
    // on the main (sensor-scale) display.
    const auto* info = algo_bridge_.find(algo_name);
    if (info && info->display_mode == AlgoDisplayMode::Standalone) {
        if (algo_name == "time_surface" || algo_name == "event_to_video" ||
            algo_name == "isi_analyzer" || algo_name == "background_mask") {
            auto* disp = new EventDisplayWidget(nullptr);
            w->set_display_widget(disp);
        }
    } else if (info && info->display_mode == AlgoDisplayMode::Overlay &&
               info->source == "self") {
        // ROI zoom view for overlay algorithms (design §5.6.6).
        auto* disp = new EventDisplayWidget(nullptr);
        w->set_display_widget(disp);
    }

    // For xyt_visualizer, also open the dedicated SpaceTimeDisplay window
    // (the AlgoWindow provides parameter control only — the 3D rendering is
    // handled by SpaceTimeDisplay's own QOpenGLWidget).
    if (algo_name == "xyt_visualizer") {
        on_open_xyt_view();
    }

    // Sync the AlgorithmsPanel checkbox (blocked, no re-entry).
    if (auto* ap = settings_->algorithms_panel()) {
        ap->set_algo_enabled(algo_name, true);
    }

    // Dock the AlgoWindow into the main window. Default to the left edge so
    // it doesn't conflict with the settings sidebar (right) or playback bar
    // (bottom). If another algo dock is already there, tab this one with it
    // so multiple algo windows share a single dock slot (the user can switch
    // between them via the tab bar).
    addDockWidget(Qt::LeftDockWidgetArea, w);
    // Find any other already-docked algo window to tabify with.
    for (auto tit = algo_windows_.constBegin(); tit != algo_windows_.constEnd(); ++tit) {
        AlgoWindow* other = tit.value().data();
        if (other && other != w && !other->isFloating()) {
            tabifyDockWidget(other, w);
            break;
        }
    }

    // On close: disable the algo instance and remove the window from the map.
    // The AlgoWindow emits `closing` from its closeEvent, which fires before
    // Qt::WA_DeleteOnClose destroys it.
    connect(w, &AlgoWindow::closing, this, [this, algo_name](const std::string&) {
        algo_windows_.remove(algo_name);
        auto inst = algo_bridge_.find_live(algo_name);
        if (inst) inst->set_enabled(false);
        // Sync the AlgorithmsPanel checkbox (blocked, no re-entry).
        if (auto* ap = settings_->algorithms_panel()) {
            ap->set_algo_enabled(algo_name, false);
        }
        // xyt_visualizer: also close the SpaceTimeDisplay.
        if (algo_name == "xyt_visualizer" && xyt_display_) xyt_display_->close();
    });

    w->show();
    w->raise();
    if (info) {
        statusBar()->showMessage(
            tr("%1 window opened").arg(QString::fromStdString(info->display_name)), 2000);
    }
}

// XYT 3D view has a dedicated SpaceTimeDisplay (QOpenGLWidget) that is opened
// from on_open_algo_window("xyt_visualizer") or directly from the sidebar.
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
        if (!xyt_algo_) xyt_algo_ = algo_bridge_.find_or_create("xyt_visualizer");
        if (xyt_algo_) xyt_algo_->set_enabled(true);
        // Sync the time_window from the algo parameter to the 3D display.
        // The GUI registers time_window_us (default 1000000 = 1s); the
        // SpaceTimeDisplay uses time_window_ms. Without this sync, the
        // display would always use its 50ms default.
        if (xyt_algo_) {
            const auto tw_us = xyt_algo_->get_param("time_window_us");
            if (!tw_us.empty()) {
                xyt_display_->set_time_window_ms(
                    static_cast<float>(std::stoi(tw_us)) / 1000.0f);
            }
        }
        // Sync the sidebar checkbox (blocked, no re-entry).
        if (auto* ap = settings_->algorithms_panel()) {
            ap->set_algo_enabled("xyt_visualizer", true);
        }
        // The QOpenGLWidget is itself the window; on close we must null
        // xyt_display_ (QPointer does this), disable the algo instance so it
        // stops processing events, and sync the sidebar checkbox.
        connect(xyt_display_, &QObject::destroyed, this, [this]() {
            if (xyt_algo_) {
                xyt_algo_->set_enabled(false);
                xyt_algo_.reset();
            }
            if (auto* ap = settings_->algorithms_panel()) {
                ap->set_algo_enabled("xyt_visualizer", false);
            }
            // Also close the AlgoWindow if open.
            auto it = algo_windows_.find("xyt_visualizer");
            if (it != algo_windows_.end() && it.value()) it.value()->close();
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

} // namespace gui
