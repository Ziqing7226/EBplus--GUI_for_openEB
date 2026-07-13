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
class QDoubleSpinBox;
class QComboBox;
class QGroupBox;

namespace gui {

class CameraController;

class PreprocessingPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit PreprocessingPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("preprocessing"); }
    QString panel_title() const override { return tr("Preprocessing"); }
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
    QHash<QString, QDoubleSpinBox*> double_spins_;
    QGroupBox* group_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_PREPROCESSING_PANEL_H
