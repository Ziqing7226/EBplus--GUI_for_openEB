// gui/panels/algorithms_panel.cpp

#include "algorithms_panel.h"

#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMap>

#include "algo_bridge/algo_bridge.h"

namespace gui {

AlgorithmsPanel::AlgorithmsPanel(AlgoBridge* bridge, QWidget* parent)
    : AbstractPanel(parent), bridge_(bridge) {
    build_ui();
}

void AlgorithmsPanel::build_ui() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);

    // Global Algorithm ROI selector — always visible at the top, above the
    // scrollable algorithm list. All self-developed algorithms share this ROI
    // (design §5.6.6). Per-algorithm roi_* params are no longer shown in each
    // algo's parameter editor; they're controlled exclusively here.
    build_roi_selector(outer);

    // Global Preprocessing selector (v1.1.0): stackable noise filter + 1/4
    // downsample applied AFTER the ROI (order: ROI → filter → downsample).
    // These overlay on top of any main algorithm and are NOT mutually
    // exclusive with it. Per-algorithm preproc_* params are skipped in each
    // algo's parameter editor and controlled exclusively here.
    build_preproc_selector(outer);

    // Algorithm category groups are added directly to the outer layout —
    // no inner QScrollArea. The outer SettingsPanel scroll area already
    // provides scrolling for the entire sidebar page (§12.2.5).
    auto* host = new QWidget(this);
    auto* layout = new QVBoxLayout(host);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);
    outer->addWidget(host);

    if (!bridge_) {
        auto* lbl = new QLabel(tr("Algorithm bridge unavailable."), host);
        layout->addWidget(lbl);
        layout->addStretch(1);
        return;
    }

    // Group algorithms by category. Only self-developed algorithms are shown
    // here: OpenEB-wrapped filters have no real backend in AlgoBridge and are
    // controlled via the Preprocess menu / PreprocessingPanel instead.
    algos_ = bridge_->list_algos();
    QMap<QString, std::vector<const AlgoInfo*>> by_cat;
    for (const auto& a : algos_) {
        if (a.source != "self") continue;
        by_cat[QString::fromStdString(a.category)].push_back(&a);
    }

    const QMap<QString, QString> cat_titles = {
        {"cv",              tr("Computer Vision")},
        {"analytics",       tr("Analytics")},
        {"calibration",     tr("Calibration")},
    };

    for (auto it = by_cat.constBegin(); it != by_cat.constEnd(); ++it) {
        const QString title = cat_titles.value(it.key(), it.key());
        auto* gb = new QGroupBox(title, host);
        auto* form = new QFormLayout(gb);
        form->setContentsMargins(6, 6, 6, 6);

        for (const auto* a : it.value()) {
            // OpenEB-wrapped algorithms (source != "self") have no real
            // backend in AlgoBridge — their parameters don't take effect here.
            // They are controlled via the Preprocess menu / PreprocessingPanel
            // FilterChain. Skip them to avoid presenting dead controls.
            if (a->source != "self") continue;

            auto* cb = new QCheckBox(QString::fromStdString(a->display_name), gb);
            checkboxes_[a->name] = cb;
            form->addRow(cb);

            // Parameter editor (shown only when enabled).
            auto* params_host = new QWidget(gb);
            auto* pform = new QFormLayout(params_host);
            pform->setContentsMargins(20, 0, 0, 0);
            params_host->setVisible(false);

            const std::string algo_name = a->name;
            // Match a default value to an enum_values entry. Entries may be
            // "N=Label" (match on the "N" prefix) or plain values.
            auto match_enum_index = [](const std::vector<std::string>& vals,
                                       const std::string& def) -> int {
                for (size_t i = 0; i < vals.size(); ++i) {
                    const auto& v = vals[i];
                    const auto eq = v.find('=');
                    const std::string token = (eq == std::string::npos)
                        ? v : v.substr(0, eq);
                    if (token == def) return static_cast<int>(i);
                }
                return -1;
            };
            for (const auto& p : a->params) {
                // Skip per-algorithm ROI params — they're controlled by the
                // global Algorithm ROI selector at the top of the panel.
                if (p.key == "roi_enabled" || p.key == "roi_x" ||
                    p.key == "roi_y" || p.key == "roi_w" ||
                    p.key == "roi_h") continue;
                // Skip per-algorithm preproc_* params — they're controlled by
                // the global Preprocessing selector at the top of the panel.
                if (p.key.rfind("preproc_", 0) == 0) continue;

                auto* lbl = new QLabel(QString::fromStdString(p.display_name),
                                       params_host);
                const std::string param_key = p.key;
                QWidget* w = nullptr;
                if (p.type == "enum") {
                    auto* cmb = new QComboBox(params_host);
                    for (const auto& v : p.enum_values) cmb->addItem(QString::fromStdString(v));
                    const int idx = match_enum_index(p.enum_values, p.default_value);
                    if (idx >= 0) cmb->setCurrentIndex(idx);
                    w = cmb;
                    // The "mode" enum drives per-mode parameter visibility.
                    // On change, apply the param and refresh which rows show.
                    if (p.key == "mode") {
                        algo_panel_state_[algo_name].mode_combo = cmb;
                        connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                                [this, algo_name, cmb](int) {
                                    apply_param(algo_name, "mode", cmb->currentText().toStdString());
                                    refresh_mode_visibility(algo_name);
                                });
                    } else {
                        connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                                [this, algo_name, param_key, cmb](int) {
                                    apply_param(algo_name, param_key, cmb->currentText().toStdString());
                                });
                    }
                } else if (p.type == "bool") {
                    auto* cmb = new QComboBox(params_host);
                    cmb->addItem("false"); cmb->addItem("true");
                    cmb->setCurrentIndex(p.default_value == "true" || p.default_value == "1" ? 1 : 0);
                    w = cmb;
                    connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                            [this, algo_name, param_key, cmb](int) {
                                apply_param(algo_name, param_key, cmb->currentText().toStdString());
                            });
                } else if (p.type == "int") {
                    auto* sp = new QSpinBox(params_host);
                    bool oklo = false, okhi = false;
                    int lo = QString::fromStdString(p.min_value).toInt(&oklo);
                    int hi = QString::fromStdString(p.max_value).toInt(&okhi);
                    sp->setRange(oklo ? lo : -100000000, okhi ? hi : 100000000);
                    sp->setValue(QString::fromStdString(p.default_value).toInt());
                    w = sp;
                    connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
                            [this, algo_name, param_key](int v) {
                                apply_param(algo_name, param_key, std::to_string(v));
                            });
                } else if (p.type == "float") {
                    auto* sp = new QDoubleSpinBox(params_host);
                    sp->setRange(-1e9, 1e9);
                    sp->setDecimals(6);
                    sp->setValue(QString::fromStdString(p.default_value).toDouble());
                    w = sp;
                    connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                            [this, algo_name, param_key](double v) {
                                apply_param(algo_name, param_key, std::to_string(v));
                            });
                } else {
                    auto* le = new QLineEdit(QString::fromStdString(p.default_value), params_host);
                    w = le;
                    connect(le, &QLineEdit::textChanged, this,
                            [this, algo_name, param_key](const QString& v) {
                                apply_param(algo_name, param_key, v.toStdString());
                            });
                }
                pform->addRow(lbl, w);
                algo_panel_state_[algo_name].rows.push_back({lbl, w, p.mode_filter, p.key});
            }
            // Apply initial per-mode visibility (hides params that don't apply
            // to the default mode, e.g. E2VID params when mode=BardowVariational).
            refresh_mode_visibility(algo_name);
            form->addRow(QString(), params_host);

            connect(cb, &QCheckBox::toggled, this, [this, params_host, cb, a, algo_name](bool on) {
                params_host->setVisible(on);
                if (on) {
                    // Algorithm mutex (design §5.6.6 — exclusive mode): only
                    // one algorithm may be enabled at a time. Uncheck every
                    // other checkbox (with signals blocked so we don't
                    // re-enter this handler) and disable its live instance.
                    // The AlgoWindow for the previously-enabled algorithm is
                    // closed by MainWindow via the algorithm_toggled signal
                    // emitted below for each disabled algo.
                    for (auto& [other_name, other_cb] : checkboxes_) {
                        if (other_name == algo_name) continue;
                        if (!other_cb || !other_cb->isChecked()) continue;
                        QSignalBlocker b(other_cb);
                        other_cb->setChecked(false);
                        // Hide the other algo's parameter editor.
                        // (The editor is the row immediately following the
                        // checkbox in the same QFormLayout; we can find it
                        // by re-using the params_host pattern: just hide via
                        // the live_instances_ map's enable flag.)
                        auto it = live_instances_.find(other_name);
                        if (it != live_instances_.end() && it->second) {
                            it->second->set_enabled(false);
                        }
                        emit algorithm_toggled(QString::fromStdString(other_name), false);
                    }
                    // Reuse the live instance if one already exists (e.g. the
                    // user edited a parameter before enabling). create() would
                    // discard those parameters by building a fresh instance.
                    auto inst = bridge_->find_or_create(algo_name);
                    if (inst) {
                        inst->set_enabled(true);
                        live_instances_[algo_name] = inst;
                        // Apply the current global ROI to this newly-enabled
                        // instance so it starts with the right region.
                        apply_global_roi();
                        // Apply the current global preprocessing settings
                        // (noise filter + 1/4 downsample) so the new instance
                        // inherits the shared stackable preprocessing stage.
                        apply_global_preproc("preproc_filter_enabled",
                            preproc_filter_cb_->isChecked() ? "true" : "false");
                        apply_global_preproc("preproc_downsample",
                            preproc_downsample_cb_->isChecked() ? "true" : "false");
                        apply_global_preproc("preproc_filter_mode",
                            std::to_string(preproc_filter_mode_combo_->currentIndex()));
                    }
                    emit info_message(tr("Algorithm enabled: %1")
                                          .arg(QString::fromStdString(a->display_name)));
                    // Request MainWindow to open the AlgoWindow so Standalone
                    // algorithms have a display and Overlay algorithms get
                    // their ROI zoom view. Without this the sidebar-enabled
                    // algorithm produces no visible output.
                    emit open_algo_window_requested(algo_name);
                } else {
                    auto it = live_instances_.find(algo_name);
                    if (it != live_instances_.end() && it->second) {
                        it->second->set_enabled(false);
                    }
                }
                emit algorithm_toggled(QString::fromStdString(a->name), on);
            });
        }
        layout->addWidget(gb);
    }

    layout->addStretch(1);

    // Initial build complete — subsequent mode switches are user-driven and
    // must not clobber user-customised ROI/fps (BUG-14 fix).
    first_init_ = false;
}

void AlgorithmsPanel::build_roi_selector(QVBoxLayout* parent_layout) {
    auto* gb = new QGroupBox(tr("Algorithm ROI"), this);
    auto* form = new QFormLayout(gb);
    form->setContentsMargins(6, 6, 6, 6);

    roi_enabled_cb_ = new QCheckBox(tr("Enabled (center 256×256 default)"), gb);
    roi_enabled_cb_->setChecked(true);
    form->addRow(roi_enabled_cb_);

    roi_x_sp_ = new QSpinBox(gb);
    roi_x_sp_->setRange(-1, 100000);
    roi_x_sp_->setValue(-1);
    roi_x_sp_->setSpecialValueText(tr("auto-center"));
    form->addRow(tr("X"), roi_x_sp_);

    roi_y_sp_ = new QSpinBox(gb);
    roi_y_sp_->setRange(-1, 100000);
    roi_y_sp_->setValue(-1);
    roi_y_sp_->setSpecialValueText(tr("auto-center"));
    form->addRow(tr("Y"), roi_y_sp_);

    roi_w_sp_ = new QSpinBox(gb);
    roi_w_sp_->setRange(0, 100000);
    roi_w_sp_->setValue(256);
    roi_w_sp_->setSpecialValueText(tr("full"));
    form->addRow(tr("W"), roi_w_sp_);

    roi_h_sp_ = new QSpinBox(gb);
    roi_h_sp_->setRange(0, 100000);
    roi_h_sp_->setValue(256);
    roi_h_sp_->setSpecialValueText(tr("full"));
    form->addRow(tr("H"), roi_h_sp_);

    parent_layout->addWidget(gb);

    // Wire up: any change applies the ROI to all live instances.
    auto apply_now = [this]() { apply_global_roi(); };
    connect(roi_enabled_cb_, &QCheckBox::toggled, this, apply_now);
    connect(roi_x_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, apply_now);
    connect(roi_y_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, apply_now);
    connect(roi_w_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, apply_now);
    connect(roi_h_sp_, QOverload<int>::of(&QSpinBox::valueChanged), this, apply_now);
}

void AlgorithmsPanel::apply_global_roi() {
    const std::string enabled = roi_enabled_cb_->isChecked() ? "true" : "false";
    const std::string x = std::to_string(roi_x_sp_->value());
    const std::string y = std::to_string(roi_y_sp_->value());
    const std::string w = std::to_string(roi_w_sp_->value());
    const std::string h = std::to_string(roi_h_sp_->value());
    // Apply to every live self-developed instance. OpenEB wrapper algorithms
    // (source=="openeb") have backend_==nullptr (pass-through); their ROI is
    // handled by the OpenEB filter_chain, not by AlgoBridge, so setting
    // roi_* params on them has no effect and would only pollute their
    // param_values_ map (BUG-13 fix).
    for (auto& [name, inst] : live_instances_) {
        if (!inst) continue;
        if (inst->info().source != "self") continue;
        inst->set_param("roi_enabled", enabled);
        inst->set_param("roi_x", x);
        inst->set_param("roi_y", y);
        inst->set_param("roi_w", w);
        inst->set_param("roi_h", h);
    }
}

void AlgorithmsPanel::apply_global_preproc(const std::string& key,
                                           const std::string& value) {
    // Delegate to the bridge, which iterates every live self-developed
    // instance and forwards the preproc_* parameter. Each backend's
    // Preprocessor / RoiFilter member recognises the key. Preprocessing is
    // stackable and NOT mutually exclusive with the main algorithm.
    if (bridge_) bridge_->apply_global_preproc(key, value);
}

void AlgorithmsPanel::build_preproc_selector(QVBoxLayout* parent_layout) {
    auto* gb = new QGroupBox(tr("Preprocessing (ROI > filter > downsample)"), this);
    auto* form = new QFormLayout(gb);
    form->setContentsMargins(6, 6, 6, 6);

    // Noise filter enable (default off — opt-in denoising stage).
    preproc_filter_cb_ = new QCheckBox(tr("Noise filter"), gb);
    preproc_filter_cb_->setChecked(false);
    form->addRow(preproc_filter_cb_);

    // 1/4 downsample (default ON to preserve v1.0.0 behaviour where
    // event_to_video had downsample=true).
    preproc_downsample_cb_ = new QCheckBox(tr("1/4 Downsample"), gb);
    preproc_downsample_cb_->setChecked(true);
    form->addRow(preproc_downsample_cb_);

    // Filter mode (8 modes, default STCF=1).
    preproc_filter_mode_combo_ = new QComboBox(gb);
    preproc_filter_mode_combo_->addItem("0=BAF");
    preproc_filter_mode_combo_->addItem("1=STCF");
    preproc_filter_mode_combo_->addItem("2=Refractory");
    preproc_filter_mode_combo_->addItem("3=DWF");
    preproc_filter_mode_combo_->addItem("4=AgePolarity");
    preproc_filter_mode_combo_->addItem("5=Harmonic");
    preproc_filter_mode_combo_->addItem("6=Repetitious");
    preproc_filter_mode_combo_->addItem("7=SpatialBP");
    preproc_filter_mode_combo_->setCurrentIndex(1);  // STCF
    form->addRow(tr("Filter mode"), preproc_filter_mode_combo_);

    // Mode-specific parameter rows (BUG-3 fix). All 8 modes' params are
    // pre-created and shown/hidden based on the selected filter mode.
    // Cross-mode params (mode=-1) are always visible when the filter is on.
    preproc_params_form_ = new QFormLayout();
    preproc_params_form_->setContentsMargins(6, 6, 6, 6);
    form->addRow(preproc_params_form_);

    // Parameter definitions: {key, display, type, def, lo, hi, mode}
    // type: "i"=int, "f"=float, "b"=bool
    struct PDef {
        const char* key;
        const char* disp;
        char type;
        const char* def;
        const char* lo;
        const char* hi;
        int mode;
    };
    static const PDef pdefs[] = {
        // STCF (mode 1)
        {"preproc_filter_correlation_time_s", "STCF corr (s)", 'f', "0.005", "0.001", "0.1", 1},
        {"preproc_filter_min_neighbors", "STCF min nbr", 'i', "2", "1", "8", 1},
        {"preproc_filter_require_polarity_match", "STCF pol match", 'b', "false", "", "", 1},
        {"preproc_filter_allow_coincidence", "STCF coincide", 'b', "false", "", "", 1},
        // BAF (mode 0)
        {"preproc_filter_baf_dt_us", "BAF dt (us)", 'i', "1000", "1000", "100000", 0},
        {"preproc_filter_baf_subsample_by", "BAF subsample", 'i', "0", "0", "4", 0},
        // Refractory (mode 2)
        {"preproc_filter_refractory_us", "Refractory (us)", 'i', "1000", "100", "100000", 2},
        // DWF (mode 3)
        {"preproc_filter_dwf_window_length", "DWF win len", 'i', "2", "1", "100", 3},
        {"preproc_filter_dwf_dist_threshold", "DWF dist", 'i', "2", "1", "1024", 3},
        {"preproc_filter_dwf_min_correlated", "DWF min corr", 'i', "2", "1", "8", 3},
        {"preproc_filter_dwf_double_mode", "DWF double", 'b', "false", "", "", 3},
        // AgePolarity (mode 4)
        {"preproc_filter_agep_tau_us", "AgePol tau (us)", 'i', "3000", "1000", "100000", 4},
        {"preproc_filter_age_threshold", "AgePol thresh", 'f', "2.0", "0.0", "8.0", 4},
        {"preproc_filter_agep_radius", "AgePol radius", 'i', "2", "1", "5", 4},
        // Harmonic (mode 5)
        {"preproc_filter_line_freq_hz", "Harm Hz", 'i', "50", "50", "60", 5},
        {"preproc_filter_notch_q", "Harm Q", 'f', "5.0", "0.1", "100.0", 5},
        {"preproc_filter_harmonic_threshold", "Harm thresh", 'f', "0.1", "0.0", "1.0", 5},
        // Repetitious (mode 6)
        {"preproc_filter_rep_period_us", "Rep period (us)", 'i', "5000", "1000", "1000000", 6},
        {"preproc_filter_rep_tolerance_us", "Rep tol (us)", 'i', "1000", "100", "10000", 6},
        {"preproc_filter_rep_ratio_shorter", "Rep ratio short", 'i', "10", "1", "100", 6},
        {"preproc_filter_rep_ratio_longer", "Rep ratio long", 'i', "10", "1", "100", 6},
        {"preproc_filter_rep_min_dt_to_store_us", "Rep min dt (us)", 'i', "1000", "0", "1000000", 6},
        // SpatialBP (mode 7)
        {"preproc_filter_sbp_center_radius_px", "SBP center", 'i', "2", "1", "10", 7},
        {"preproc_filter_sbp_surround_radius_px", "SBP surround", 'i', "10", "5", "30", 7},
        {"preproc_filter_sbp_dt_surround_us", "SBP dt (us)", 'i', "10000", "100", "1000000", 7},
        // Cross-mode flags
        {"preproc_filter_filter_hot_pixels", "Filter hot px", 'b', "false", "", "", -1},
        {"preproc_filter_adaptive_correlation_time", "Adaptive corr", 'b', "false", "", "", -1},
    };

    for (const auto& p : pdefs) {
        auto* lbl = new QLabel(tr(p.disp), gb);
        QWidget* w = nullptr;
        const std::string pkey = p.key;
        if (p.type == 'b') {
            auto* cmb = new QComboBox(gb);
            cmb->addItem("false"); cmb->addItem("true");
            cmb->setCurrentIndex(std::string(p.def) == "true" ? 1 : 0);
            w = cmb;
            connect(cmb, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                    [this, pkey, cmb](int) {
                        apply_global_preproc(pkey, cmb->currentText().toStdString());
                    });
        } else if (p.type == 'i') {
            auto* sp = new QSpinBox(gb);
            sp->setRange(std::string(p.lo).empty() ? -100000000 : std::stoi(p.lo),
                         std::string(p.hi).empty() ? 100000000 : std::stoi(p.hi));
            sp->setValue(std::stoi(p.def));
            w = sp;
            connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
                    [this, pkey](int v) {
                        apply_global_preproc(pkey, std::to_string(v));
                    });
        } else { // 'f'
            auto* sp = new QDoubleSpinBox(gb);
            sp->setRange(-1e9, 1e9);
            sp->setDecimals(6);
            sp->setValue(std::stod(p.def));
            w = sp;
            connect(sp, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                    [this, pkey](double v) {
                        apply_global_preproc(pkey, std::to_string(v));
                    });
        }
        preproc_params_form_->addRow(lbl, w);
        preproc_rows_.push_back({lbl, w, p.mode, pkey});
    }

    parent_layout->addWidget(gb);

    // Wire up: any change applies the preproc setting to all live instances.
    // These checkboxes are intentionally NOT added to checkboxes_ (the
    // algorithm-mutex map) so enabling preprocessing does not disable the
    // main algorithm — preprocessing overlays on top of it.
    connect(preproc_filter_cb_, &QCheckBox::toggled, this, [this](bool on) {
        apply_global_preproc("preproc_filter_enabled", on ? "true" : "false");
        refresh_preproc_params();
    });
    connect(preproc_downsample_cb_, &QCheckBox::toggled, this, [this](bool on) {
        apply_global_preproc("preproc_downsample", on ? "true" : "false");
    });
    connect(preproc_filter_mode_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int idx) {
                apply_global_preproc("preproc_filter_mode", std::to_string(idx));
                refresh_preproc_params();
            });

    // Show the rows matching the default mode (STCF=1).
    refresh_preproc_params();
}

void AlgorithmsPanel::refresh_preproc_params() {
    if (!preproc_filter_mode_combo_) return;
    const int mode = preproc_filter_mode_combo_->currentIndex();
    const bool filter_on = preproc_filter_cb_ && preproc_filter_cb_->isChecked();
    for (auto& row : preproc_rows_) {
        const bool visible = filter_on && (row.mode < 0 || row.mode == mode);
        if (row.label) row.label->setVisible(visible);
        if (row.field) row.field->setVisible(visible);
    }
}

void AlgorithmsPanel::apply_param(const std::string& algo_name,
                                  const std::string& param_key,
                                  const std::string& value) {
    // Lazily create the instance so parameter edits are recorded even before
    // the enable checkbox is toggled. find_or_create preserves any previously
    // set parameters instead of overwriting them with defaults.
    auto it = live_instances_.find(algo_name);
    if (it == live_instances_.end() || !it->second) {
        auto inst = bridge_ ? bridge_->find_or_create(algo_name) : nullptr;
        if (!inst) return;
        // Use operator[] (not emplace) so a pre-existing null entry is
        // replaced. std::unordered_map::emplace is a no-op if the key
        // already exists, which would leave a null shared_ptr and crash
        // on the following dereference.
        live_instances_[algo_name] = inst;
        it = live_instances_.find(algo_name);
    }
    it->second->set_param(param_key, value);
}

void AlgorithmsPanel::refresh_mode_visibility(const std::string& algo_name) {
    auto it = algo_panel_state_.find(algo_name);
    if (it == algo_panel_state_.end()) return;
    auto& state = it->second;
    if (!state.mode_combo) return;  // algo has no "mode" enum
    const int idx = state.mode_combo->currentIndex();
    const std::string idx_str = std::to_string(idx);
    for (auto& row : state.rows) {
        if (row.mode_filter.empty()) continue;  // common param: always visible
        // mode_filter is a comma-separated list of mode indices ("0", "1,2").
        bool visible = false;
        std::size_t pos = 0;
        while (pos < row.mode_filter.size()) {
            const auto comma = row.mode_filter.find(',', pos);
            const std::string token = (comma == std::string::npos)
                ? row.mode_filter.substr(pos)
                : row.mode_filter.substr(pos, comma - pos);
            if (token == idx_str) { visible = true; break; }
            if (comma == std::string::npos) break;
            pos = comma + 1;
        }
        if (row.label) row.label->setVisible(visible);
        if (row.field) row.field->setVisible(visible);
    }

    // Auto-set mode-appropriate ROI and output_fps only during initial build
    // (design §4.4.2): all three event_to_video modes default to a 128×128
    // center ROI with 1/4 downsample enabled (effective reconstruction at
    // 64×64). E2VID runs NN inference at this resolution; BardowVariational
    // and InteractingMaps also downsample for the same throughput benefit
    // (the output is upsampled back to the ROI size for display). 24 fps is
    // a comfortable target across all modes.
    // Only event_to_video has a "mode" enum, so this code only runs for it.
    // BUG-14 fix: skip ROI/fps reset on user-driven mode switches so
    // user-customised values are preserved.
    if (!first_init_) return;

    const int target_w  = 128;
    const int target_h  = 128;
    const int target_fps = 24;

    // ROI — global controls.
    {
        QSignalBlocker bx(roi_x_sp_);
        QSignalBlocker by(roi_y_sp_);
        QSignalBlocker bw(roi_w_sp_);
        QSignalBlocker bh(roi_h_sp_);
        QSignalBlocker be(roi_enabled_cb_);
        roi_x_sp_->setValue(-1);   // auto-center
        roi_y_sp_->setValue(-1);   // auto-center
        roi_w_sp_->setValue(target_w);
        roi_h_sp_->setValue(target_h);
        roi_enabled_cb_->setChecked(true);
        apply_global_roi();
    }

    // output_fps — find the per-algo param row by key.
    for (auto& row : state.rows) {
        if (row.key == "output_fps") {
            auto* sp = qobject_cast<QSpinBox*>(row.field);
            if (sp) { QSignalBlocker b(sp); sp->setValue(target_fps); }
            apply_param(algo_name, "output_fps", std::to_string(target_fps));
            break;
        }
    }
}

void AlgorithmsPanel::set_algo_enabled(const std::string& name, bool on) {
    auto it = checkboxes_.find(name);
    if (it == checkboxes_.end() || !it->second) return;
    // Block signals so this programmatic change does not re-enter the toggled
    // handler (which would create/enable instances and emit algorithm_toggled,
    // causing sync loops with the Algorithm menu / AlgoWindow).
    QSignalBlocker b(it->second);
    it->second->setChecked(on);

    // Algorithm mutex: when turning an algo on programmatically (e.g. from
    // on_open_algo_window), uncheck every other algo so only one is live at
    // a time. The toggled-handler path enforces mutex itself; this covers
    // the programmatic path.
    if (on) {
        for (auto& [other_name, other_cb] : checkboxes_) {
            if (other_name == name) continue;
            if (!other_cb || !other_cb->isChecked()) continue;
            QSignalBlocker ob(other_cb);
            other_cb->setChecked(false);
            auto oi = live_instances_.find(other_name);
            if (oi != live_instances_.end() && oi->second) {
                oi->second->set_enabled(false);
            }
            emit algorithm_toggled(QString::fromStdString(other_name), false);
        }
    }
}

} // namespace gui
