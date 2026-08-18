// gui/panels/preprocessing_panel.h — event filter chain UI (design §4.3.1).
//
// One checkbox + parameter row per FilterChain stage. Edits apply immediately
// to the FilterChain owned by the CameraController.

#ifndef GUI_PANELS_PREPROCESSING_PANEL_H
#define GUI_PANELS_PREPROCESSING_PANEL_H

#include <QWidget>
#include <QHash>
#include <QString>

#include "abstract_panel.h"

class QCheckBox;
class QSpinBox;
class QComboBox;
class QGroupBox;

namespace gui {

class CameraController;

class PreprocessingPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit PreprocessingPanel(QWidget* parent = nullptr);

    // Stable id "preprocessing" (layout/panel registry key) — do NOT rename.
    QString panel_id() const override { return QStringLiteral("preprocessing"); }
    // User-visible title: "Display Transform" distinguishes these OpenEB
    // event-transform stages from the AlgorithmsPanel "Preprocessing" stages
    // (filter/downsample/undistort). NOTE: both reach the display AND every
    // algorithm instance — the FilterChain output is shared (so Replace-mode
    // output keeps the display orientation).
    QString panel_title() const override { return tr("Display Transform"); }
    QString panel_group() const override { return QStringLiteral("Algorithms"); }

public slots:
    void on_camera_connected(CameraController* controller) override;
    void on_camera_disconnected() override;

    /// @brief Menu-friendly accessor: sets the enable state of @p stage.
    void set_stage_enabled(const QString& stage, bool on);
    /// @brief Menu-friendly accessor: queries the enable state of @p stage.
    bool is_stage_enabled(const QString& stage) const;

signals:
    /// @brief Emitted when a stage's enabled state changes (user or program).
    /// MainWindow uses this to sync the Preprocess menu actions.
    void stage_toggled(const QString& stage, bool on);

private:
    void build_ui();
    void apply_stage(const QString& name);

    QHash<QString, QCheckBox*> enables_;
    QHash<QString, QComboBox*> combos_;
    QHash<QString, QSpinBox*> spins_;
    QGroupBox* group_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_PREPROCESSING_PANEL_H
