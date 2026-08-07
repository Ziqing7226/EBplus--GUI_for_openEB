// gui/widgets/algo_window.cpp — algorithm display window (output only).
//
// See algo_window.h for the design rationale. The window hosts only the
// algorithm output (status label or a custom display widget such as
// EventDisplayWidget). All parameter adjustment is handled exclusively by
// the sidebar (AlgorithmsPanel).

#include "algo_window.h"

#include <QCloseEvent>
#include <QLabel>
#include <QVBoxLayout>

#include "display/event_display_widget.h"

namespace gui {

AlgoWindow::AlgoWindow(AlgoBridge* bridge, const std::string& algo_name,
                       QWidget* parent)
    : QDockWidget(parent),
      bridge_(bridge),
      algo_name_(algo_name) {
    setWindowTitle(QString::fromStdString(algo_name));
    setObjectName(QString("AlgoDock_%1").arg(QString::fromStdString(algo_name)));
    setAttribute(Qt::WA_DeleteOnClose);
    setAllowedAreas(Qt::AllDockWidgetAreas);
    setFeatures(QDockWidget::DockWidgetMovable |
                QDockWidget::DockWidgetClosable |
                QDockWidget::DockWidgetFloatable);
    setMinimumWidth(360);
    setMinimumHeight(300);

    // Look up the algo info and find/create the live instance.
    const AlgoInfo* info = bridge_ ? bridge_->find(algo_name_) : nullptr;
    if (info) {
        info_ = *info;
        setWindowTitle(QString::fromStdString(info_.display_name));
        instance_ = bridge_->find_live(algo_name_);
        if (!instance_) instance_ = bridge_->find_or_create(algo_name_);
        if (instance_) instance_->set_enabled(true);
    } else if (algo_name_ == "sensor_self_test") {
        // Hardware diagnostic (Devices panel button), intentionally NOT a
        // registered algorithm: use a built-in AlgoInfo so the window and
        // backend can be created without polluting the Algorithms panel.
        AlgoInfo a;
        a.name = "sensor_self_test";
        a.display_name = "Sensor Self-Test";
        a.category = "devices";
        a.source = "self";
        a.display_mode = AlgoDisplayMode::Standalone;
        info_ = a;
        setWindowTitle(QString::fromStdString(info_.display_name));
        instance_ = bridge_ ? bridge_->find_or_create_with_info(info_) : nullptr;
        if (instance_) instance_->set_enabled(true);
    }

    // QDockWidget requires an inner content widget set via setWidget().
    content_ = new QWidget(this);
    auto* outer = new QVBoxLayout(content_);
    outer->setContentsMargins(8, 8, 8, 8);
    outer->setSpacing(8);

    // Display area: defaults to a status QLabel; Standalone frame algos
    // install an EventDisplayWidget via set_display_widget() after construction.
    display_layout_ = new QVBoxLayout();
    display_layout_->setContentsMargins(0, 0, 0, 0);
    status_label_ = new QLabel(tr("Waiting for events..."), content_);
    status_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    QFont f(QStringLiteral("Monospace"));
    f.setStyleHint(QFont::TypeWriter);
    f.setPointSize(10);
    status_label_->setFont(f);
    status_label_->setMinimumHeight(120);
    display_widget_ = status_label_;
    display_layout_->addWidget(display_widget_);
    outer->addLayout(display_layout_, 1);

    setWidget(content_);
}

EventDisplayWidget* AlgoWindow::frame_display() const {
    return qobject_cast<EventDisplayWidget*>(display_widget_);
}

void AlgoWindow::set_display_widget(QWidget* w) {
    if (!w || w == display_widget_) return;
    // Remove the old display widget (default status label) and install the
    // new one in the same layout slot.
    if (display_widget_) {
        display_layout_->removeWidget(display_widget_);
        // Clear the member before deleting so any signal/event triggered
        // during deletion sees null instead of a dangling pointer (BUG-G10).
        QWidget* old = display_widget_;
        display_widget_ = nullptr;
        delete old;
    }
    display_widget_ = w;
    display_layout_->addWidget(display_widget_, 1);
    // The status label is gone after a custom widget is installed.
    status_label_ = nullptr;
}

void AlgoWindow::set_status_text(const QString& text) {
    if (status_label_) {
        status_label_->setText(text);
    }
}

void AlgoWindow::closeEvent(QCloseEvent* event) {
    emit closing(algo_name_);
    // Explicitly accept so QDockWidget::close() / WA_DeleteOnClose proceed
    // correctly. The default QWidget::closeEvent calls event->ignore(),
    // which would prevent the dock from closing.
    event->accept();
}

} // namespace gui
