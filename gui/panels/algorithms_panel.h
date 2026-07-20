// gui/panels/algorithms_panel.h — algorithm selection & parameter UI (design §3.8).
//
// Lists every registered algorithm from AlgoBridge, grouped by category, with
// an enable checkbox and an expandable parameter editor per algorithm.
//
// Phases 6-8 self-developed CV/analytics algorithms are fully implemented in
// algo/cv and algo/analytics, and wired through AlgoBackend instances in
// algo_backend.cpp. Enabling an algorithm here activates its real processing
// pipeline (push_events / pull_result) and routes results to the display via
// FrameAnnotator (Overlay), frame replacement (Replace), or standalone
// windows (Standalone).

#ifndef GUI_PANELS_ALGORITHMS_PANEL_H
#define GUI_PANELS_ALGORITHMS_PANEL_H

#include <QCheckBox>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "algo_bridge/algo_bridge.h"  // for AlgoInfo + AlgoInstance
#include "abstract_panel.h"

// Forward declarations of Qt widgets (defined in the global namespace).
class QLabel;
class QComboBox;
class QFormLayout;
class QLineEdit;
class QPushButton;

namespace gui {

class AlgoBridge;

class AlgorithmsPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit AlgorithmsPanel(AlgoBridge* bridge, QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("algorithms"); }
    QString panel_title() const override { return tr("Algorithms"); }
    QString panel_group() const override { return QStringLiteral("Algorithms"); }

    /// @brief Programmatically sets the enable-checkbox state for @p name
    /// without emitting toggled signals. Used by MainWindow to keep the
    /// panel in sync with the Algorithm menu and AlgoWindow.
    void set_algo_enabled(const std::string& name, bool on);

signals:
    /// @brief Emitted when an algorithm's enable state changes.
    void algorithm_toggled(const QString& name, bool enabled);
    /// @brief Emitted when an algorithm is enabled from the sidebar and
    /// needs an AlgoWindow opened (Standalone/Overlay algos need a display).
    void open_algo_window_requested(const std::string& name);
    /// @brief Emitted (from the SDK data thread, via the bridge's overload
    /// callback) when the flood guard auto-disables an algorithm. Connected
    /// queued to on_algorithm_overloaded so the checkbox sync runs on the
    /// GUI thread (audit §5-E2).
    void algorithm_overloaded(const QString& name);

private slots:
    /// Unchecks the sidebar checkbox of a flood-guard-disabled algorithm and
    /// notifies the user once (audit §5-E2).
    void on_algorithm_overloaded(const QString& name);

private:
    void build_ui();
    /// Forwards a parameter edit to the live AlgoInstance (if one exists for
    /// the named algorithm). Creates the instance lazily via bridge_->create()
    /// so parameter changes before the checkbox is toggled are still recorded
    /// in the registry for ConfigManager save/load.
    void apply_param(const std::string& algo_name,
                     const std::string& param_key,
                     const std::string& value);
    /// Applies the global Algorithm ROI (x/y/w/h + enabled) to every live
    /// algorithm instance. Called whenever the user edits the global ROI
    /// controls at the top of the panel.
    void apply_global_roi();
    /// Applies a shared preprocessing parameter (preproc_*) to every live
    /// algorithm instance via AlgoBridge::apply_global_preproc. Preprocessing
    /// (noise filter + 1/4 downsample) is stackable and NOT mutually exclusive
    /// with the main algorithm.
    void apply_global_preproc(const std::string& key, const std::string& value);
    /// Builds the global Algorithm ROI selector group at the top of the panel.
    void build_roi_selector(QVBoxLayout* parent_layout);
    /// Builds the global Preprocessing selector group (noise filter + 1/4
    /// downsample). The checkboxes are NOT part of the algorithm mutex
    /// (checkboxes_) — preprocessing overlays on top of any main algorithm.
    void build_preproc_selector(QVBoxLayout* parent_layout);
    /// Rebuilds the mode-specific NoiseFilter parameter rows in the
    /// Preprocessing group based on the selected filter mode (BUG-3 fix).
    /// Each of the 8 denoiser modes (BAF/STCF/Refractory/DWF/AgePolarity/
    /// Harmonic/Repetitious/SpatialBP) has its own parameter set; only the
    /// rows matching the current mode are shown.
    void refresh_preproc_params();
    /// Shows/hides mode-scoped parameter rows for @p algo_name based on the
    /// currently selected index of its "mode" enum combobox. Params whose
    /// AlgoParamSpec::mode_filter does not contain the current mode index are
    /// hidden (label + field); common params (empty mode_filter) stay visible.
    void refresh_mode_visibility(const std::string& algo_name);

    /// One label + field widget pair for a parameter row, plus the
    /// mode_filter that decides whether the row is visible for the current
    /// mode (empty = always visible).
    struct ParamRow {
        QLabel* label{nullptr};
        QWidget* field{nullptr};
        std::string mode_filter;
        std::string key;  ///< Parameter key (e.g. "output_fps") — used to
                          ///< locate rows programmatically (e.g. auto-params
                          ///< on event_to_video mode switch).
    };
    /// Per-algorithm UI state for mode-scoped parameter visibility.
    struct AlgoPanelState {
        QComboBox* mode_combo{nullptr};
        QWidget* params_host{nullptr};  ///< Container widget for param rows
        std::vector<ParamRow> rows;
    };

    AlgoBridge* bridge_;
    /// Owns a long-lived copy of the registry so pointers handed to lambdas
    /// remain valid for the panel's lifetime. AlgoBridge::list_algos()
    /// returns by value, so storing iterators/pointers into its result would
    /// dangle after the temporary is destroyed.
    std::vector<AlgoInfo> algos_;
    /// Live algorithm instances created when the user enables an algorithm.
    /// Stored so parameter edits can be forwarded to the instance via
    /// set_param(), and so ConfigManager can query runtime values via
    /// AlgoBridge::find_live().
    std::unordered_map<std::string, std::shared_ptr<AlgoInstance>> live_instances_;
    /// Enable checkboxes keyed by algo name, for programmatic sync.
    std::unordered_map<std::string, QCheckBox*> checkboxes_;
    /// Per-algo parameter-row state, used to toggle mode-scoped params when
    /// the "mode" enum combobox changes (see refresh_mode_visibility).
    std::unordered_map<std::string, AlgoPanelState> algo_panel_state_;

    /// Global Algorithm ROI controls (design §5.6.6). All self-developed
    /// algorithms share this ROI; per-algorithm roi_* params are removed
    /// from the parameter editors and controlled exclusively here.
    QCheckBox* roi_enabled_cb_{nullptr};
    QSpinBox* roi_x_sp_{nullptr};
    QSpinBox* roi_y_sp_{nullptr};
    QSpinBox* roi_w_sp_{nullptr};
    QSpinBox* roi_h_sp_{nullptr};

    /// Global Preprocessing controls (v1.1.0). The noise filter + 1/4
    /// downsample are stackable stages applied AFTER the algorithm ROI
    /// (order: ROI → filter → downsample). They overlay on top of any main
    /// algorithm and are NOT mutually exclusive with it. These checkboxes
    /// are intentionally NOT stored in checkboxes_ (the algorithm-mutex map)
    /// so enabling preprocessing does not disable the main algorithm.
    /// preproc_downsample defaults to UNCHECKED (audit §5-F1): for most
    /// backends it only thins events (coordinates unchanged), which is a
    /// silent 4× input loss for detection/tracking algorithms.
    QCheckBox* preproc_filter_cb_{nullptr};
    QCheckBox* preproc_downsample_cb_{nullptr};
    QComboBox* preproc_filter_mode_combo_{nullptr};

    /// Undistort stage (applied AFTER filter + downsample). Loads the YAML
    /// produced by Tools → Intrinsic Wizard and applies a forward event LUT.
    /// Default path matches the wizard's default export directory.
    QCheckBox* preproc_undistort_cb_{nullptr};
    QLineEdit* preproc_undistort_path_{nullptr};
    QPushButton* preproc_undistort_browse_{nullptr};

    /// Container for mode-specific NoiseFilter parameter rows (BUG-3).
    /// Rows are pre-created for all 8 modes and shown/hidden based on the
    /// selected filter mode. Each entry: {label, field, mode_index}.
    /// mode_index -1 = cross-mode (always visible when filter is on).
    QFormLayout* preproc_params_form_{nullptr};
    struct PreprocRow {
        QLabel* label{nullptr};
        QWidget* field{nullptr};
        int mode{-1};  ///< -1 = cross-mode, 0-7 = specific filter mode
        std::string key;
    };
    std::vector<PreprocRow> preproc_rows_;

    /// Flag: true during build_ui() so refresh_mode_visibility knows to apply
    /// default ROI/fps; set to false after build_ui completes so user-driven
    /// mode switches don't clobber user-customised ROI/fps (BUG-14 fix).
    bool first_init_{true};

    /// One-shot guard for the E2VID "model failed to load" warning
    /// (audit §5-H1) so the user is not spammed on every model_path edit.
    bool e2vid_model_error_shown_{false};
};

} // namespace gui

#endif // GUI_PANELS_ALGORITHMS_PANEL_H
