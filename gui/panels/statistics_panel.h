// gui/panels/statistics_panel.h — real-time event-rate / ON-OFF / FPS display.

#ifndef GUI_PANELS_STATISTICS_PANEL_H
#define GUI_PANELS_STATISTICS_PANEL_H

#include <QWidget>
#include <cstdint>

#include <metavision/sdk/base/utils/timestamp.h>

#include "abstract_panel.h"

class QLabel;
class QFormLayout;

namespace gui {

class StatisticsPanel : public AbstractPanel {
    Q_OBJECT
public:
    explicit StatisticsPanel(QWidget* parent = nullptr);

    QString panel_id() const override { return QStringLiteral("statistics"); }
    QString panel_title() const override { return tr("Statistics"); }
    QString panel_group() const override { return QStringLiteral("Display & Stats"); }

    public slots:
    void set_rate(double rate_eps, double peak_eps, Metavision::timestamp t);
    void set_on_off(std::uint64_t on_count, std::uint64_t off_count, double on_ratio);
    void set_fps(double fps);
    void set_timestamp(Metavision::timestamp t);
    void clear();

private:
    static QString format_rate(double rate_eps);

    QLabel* value_rate_{nullptr};
    QLabel* value_peak_{nullptr};
    QLabel* value_ratio_{nullptr};
    QLabel* value_on_{nullptr};
    QLabel* value_off_{nullptr};
    QLabel* value_fps_{nullptr};
    QLabel* value_ts_{nullptr};
};

} // namespace gui

#endif // GUI_PANELS_STATISTICS_PANEL_H
