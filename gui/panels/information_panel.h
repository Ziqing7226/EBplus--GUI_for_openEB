// gui/panels/information_panel.h — sensor metadata + live performance display.

#ifndef GUI_PANELS_INFORMATION_PANEL_H
#define GUI_PANELS_INFORMATION_PANEL_H

#include <QWidget>
#include "../app/camera_controller.h"
#include "abstract_panel.h"

class QLabel;
class QFormLayout;

namespace gui {

class InformationPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit InformationPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("information"); }
    QString panel_title() const override { return tr("Information"); }
    QString panel_group() const override { return QStringLiteral("Camera"); }

    public slots:
    void set_info(const SensorInfo& info);
    /// @brief Updates live performance metrics.
    /// @param latency_ms  End-to-end processing latency (event arrival → frame display).
    /// @param throughput_mbs  Data throughput in MB/s (from SDK RateEstimator × event size).
    void set_performance(double latency_ms, double throughput_mbs);
    /// @brief Updates algorithm overload status.
    void set_algo_status(int active, int overloaded);
    /// @brief Updates the maximum event drop rate across all live algorithm
    /// instances. @p max_drop_pct is a percentage in [0, 100]; 0 means no
    /// events were dropped by any flood guard.
    void set_drop_rate(double max_drop_pct);
    void clear();

private:
    QLabel* value_model_{nullptr};
    QLabel* value_resolution_{nullptr};
    QLabel* value_serial_{nullptr};
    QLabel* value_integrator_{nullptr};
    QLabel* value_plugin_{nullptr};
    QLabel* value_encoding_{nullptr};
    QLabel* value_firmware_{nullptr};
    QLabel* value_source_{nullptr};
    // Live performance metrics.
    QLabel* value_latency_{nullptr};
    QLabel* value_throughput_{nullptr};
    QLabel* value_algo_status_{nullptr};
    QLabel* value_drop_rate_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_INFORMATION_PANEL_H
