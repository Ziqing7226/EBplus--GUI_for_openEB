// gui/panels/biases_panel.h — sensor bias control (design §3.1.2).
//
// Iterates I_LL_Biases::get_all_biases() at connect time and builds a row
// per bias with: name label, slider, precise spinbox, reset button. The
// recommended range from LL_Bias_Info::get_bias_range() drives the slider;
// the spinbox allows typing arbitrary values inside the same range. Edits
// are applied immediately via I_LL_Biases::set(name, value). Reset restores
// the value snapshot taken when the panel was populated.

#ifndef GUI_PANELS_BIASES_PANEL_H
#define GUI_PANELS_BIASES_PANEL_H

#include <QVBoxLayout>
#include <QWidget>
#include <memory>
#include <string>
#include <vector>

#include "abstract_panel.h"

class QSpinBox;
class QSlider;
class QLabel;

namespace gui {

class CameraController;

class BiasesPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit BiasesPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("biases"); }
    QString panel_title() const override { return tr("Biases"); }
    QString panel_group() const override { return QStringLiteral("Hardware"); }

public slots:
    /// @brief Populates the panel from the connected camera's bias facility.
    /// If the facility is unavailable (file playback or unsupported sensor),
    /// the panel is disabled with an explanatory hint.
    void on_camera_connected(CameraController* controller) override;

    /// @brief Clears all rows and disables the panel.
    void on_camera_disconnected() override;

    /// @brief Saves current biases to a file via I_LL_Biases::save_to_file.
    void save_to_file(const QString& path);
    /// @brief Loads biases from a file via I_LL_Biases::load_from_file and
    /// refreshes the UI to reflect the new values.
    void load_from_file(const QString& path);

private:
    struct BiasRow {
        std::string name;
        int snapshot_value{0};
        QWidget* row_widget{nullptr};
        QSlider* slider{nullptr};
        QSpinBox* spin{nullptr};
    };

    void clear_rows();
    void populate();
    void apply_value(BiasRow& row, int value);
    void refresh_row_values();

    QVBoxLayout* rows_layout_{nullptr};
    QWidget* container_{nullptr};
    QLabel* hint_label_{nullptr};

    std::vector<BiasRow> rows_;
    bool populated_{false};
};

} // namespace gui

#endif // GUI_PANELS_BIASES_PANEL_H
