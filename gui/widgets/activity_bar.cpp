// gui/widgets/activity_bar.cpp

#include "activity_bar.h"

#include <QButtonGroup>
#include <QDockWidget>
#include <QMainWindow>
#include <QMouseEvent>
#include <QPushButton>
#include <QStyle>
#include <QVBoxLayout>

#include "app/icon_provider.h"

namespace gui {

ActivityBar::ActivityBar(QWidget* parent)
    : QFrame(parent) {
    setFixedWidth(48);
    setAttribute(Qt::WA_StyledBackground, true);
    setObjectName(QStringLiteral("activity_bar"));
    // OpenHandCursor on blank areas indicates the sidebar is draggable.
    // Buttons override this with PointingHandCursor.
    setCursor(Qt::OpenHandCursor);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 6, 0, 6);
    layout->setSpacing(0);

    // Sub-layout for group buttons — spacing ~1/3 of icon size (20px → 7px).
    group_layout_ = new QVBoxLayout();
    group_layout_->setContentsMargins(0, 0, 0, 0);
    group_layout_->setSpacing(7);
    layout->addLayout(group_layout_);

    // Stretch pushes the toggle button to the very bottom.
    layout->addStretch(1);

    // Toggle button (non-checkable) — controls sidebar content visibility.
    // The host sets its icon via set_toggle_icon() based on dock area and
    // visibility. Default: chevron-left (sidebar on left, visible).
    toggle_btn_ = new QPushButton(this);
    toggle_btn_->setIcon(IconProvider::get(QStringLiteral("chevron-left")));
    toggle_icon_name_ = QStringLiteral("chevron-left");
    toggle_btn_->setIconSize(QSize(20, 20));
    toggle_btn_->setFixedSize(40, 40);
    toggle_btn_->setToolTip(QObject::tr("Toggle Sidebar"));
    toggle_btn_->setCursor(Qt::PointingHandCursor);
    toggle_btn_->setFocusPolicy(Qt::NoFocus);
    layout->addWidget(toggle_btn_, 0, Qt::AlignHCenter);

    connect(toggle_btn_, &QPushButton::clicked, this, [this]() {
        emit toggle_clicked();
    });

    group_ = new QButtonGroup(this);
    group_->setExclusive(true);

    // Forward QButtonGroup::idToggled → group_selected. Connected ONCE here
    // (not in add_button) so we don't create duplicate connections.
    connect(group_, &QButtonGroup::idToggled, this,
            [this](int id, bool checked) {
                if (checked && id >= 0 &&
                    id < static_cast<int>(entries_.size())) {
                    emit group_selected(id, entries_[id].title);
                }
            });

    // VSCode-style styling: transparent background, accent strip on the left
    // edge for the checked button, subtle hover/checked backgrounds.
    setStyleSheet(QStringLiteral(
        "QFrame#activity_bar { background-color: palette(window);"
        "  border-right: 1px solid palette(mid); }"
        "QPushButton {"
        "  background: transparent;"
        "  border: none;"
        "  border-left: 2px solid transparent;"
        "  padding: 8px;"
        "  margin: 0 4px;"
        "  border-radius: 6px;"
        "}"
        "QPushButton:hover { background-color: palette(midlight); }"
        "QPushButton:checked {"
        "  background-color: palette(midlight);"
        "  border-left: 2px solid palette(highlight);"
        "}"));
}

int ActivityBar::add_button(const QString& icon_name, const QString& title,
                            const QString& tooltip) {
    auto* btn = new QPushButton(this);
    btn->setIcon(IconProvider::get(icon_name));
    btn->setIconSize(QSize(20, 20));
    btn->setFixedSize(40, 40);
    btn->setCheckable(true);
    btn->setToolTip(tooltip.isEmpty() ? title : tooltip);
    btn->setCursor(Qt::PointingHandCursor);
    btn->setFocusPolicy(Qt::NoFocus);

    const int index = static_cast<int>(entries_.size());
    entries_.push_back({btn, icon_name, title});
    group_->addButton(btn, index);

    group_layout_->addWidget(btn, 0, Qt::AlignHCenter);

    // Auto-select the first entry so the bar never has an empty selection.
    if (index == 0) {
        btn->setChecked(true);
    }

    return index;
}

void ActivityBar::select(int index) {
    if (entries_.empty()) return;
    if (index < 0) index = 0;
    if (index >= static_cast<int>(entries_.size())) {
        index = static_cast<int>(entries_.size()) - 1;
    }
    auto* btn = entries_[index].button;
    if (!btn->isChecked()) {
        btn->setChecked(true);
    } else {
        emit group_selected(index, entries_[index].title);
    }
}

int ActivityBar::current_index() const {
    return group_ ? group_->checkedId() : -1;
}

QString ActivityBar::title_at(int index) const {
    if (index < 0 || index >= static_cast<int>(entries_.size())) {
        return QString();
    }
    return entries_[index].title;
}

void ActivityBar::set_toggle_icon(const QString& icon_name) {
    toggle_icon_name_ = icon_name;
    toggle_btn_->setIcon(IconProvider::get(icon_name));
}

void ActivityBar::refresh_icons() {
    for (auto& e : entries_) {
        e.button->setIcon(IconProvider::get(e.icon_name));
    }
    if (!toggle_icon_name_.isEmpty()) {
        toggle_btn_->setIcon(IconProvider::get(toggle_icon_name_));
    }
    // Force QSS re-polish so palette() references in the local stylesheet
    // pick up the new theme colors (§12.2.2 — theme color lag fix).
    style()->unpolish(this);
    style()->polish(this);
    update();
}

QDockWidget* ActivityBar::find_dock() const {
    auto* p = parentWidget();
    while (p) {
        if (auto* d = qobject_cast<QDockWidget*>(p)) return d;
        p = p->parentWidget();
    }
    return nullptr;
}

void ActivityBar::mousePressEvent(QMouseEvent* event) {
    if (event->button() != Qt::LeftButton) {
        QFrame::mousePressEvent(event);
        return;
    }
    // If the click lands on a button, let the button handle it.
    if (qobject_cast<QPushButton*>(childAt(event->pos()))) {
        QFrame::mousePressEvent(event);
        return;
    }
    // Blank area — start a potential sidebar drag.
    auto* dock = find_dock();
    if (!dock) {
        QFrame::mousePressEvent(event);
        return;
    }
    auto* win = qobject_cast<QMainWindow*>(dock->parentWidget());
    if (!win) {
        QFrame::mousePressEvent(event);
        return;
    }
    dragging_ = true;
    drag_start_pos_ = event->globalPosition().toPoint();
    drag_start_area_ = win->dockWidgetArea(dock);
    setCursor(Qt::ClosedHandCursor);
    event->accept();
}

void ActivityBar::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) {
        QFrame::mouseMoveEvent(event);
        return;
    }
    auto* dock = find_dock();
    if (!dock) return;
    auto* win = qobject_cast<QMainWindow*>(dock->parentWidget());
    if (!win) return;

    const QPoint global_pos = event->globalPosition().toPoint();
    const int window_center_x = win->x() + win->width() / 2;

    // If the mouse crosses the window's horizontal center, move the dock to
    // the opposite side.
    const auto current_area = win->dockWidgetArea(dock);
    if (drag_start_area_ == Qt::LeftDockWidgetArea &&
        global_pos.x() > window_center_x &&
        current_area == Qt::LeftDockWidgetArea) {
        win->removeDockWidget(dock);
        win->addDockWidget(Qt::RightDockWidgetArea, dock);
        dock->setVisible(true);
        dock->setFloating(false);
    } else if (drag_start_area_ == Qt::RightDockWidgetArea &&
               global_pos.x() < window_center_x &&
               current_area == Qt::RightDockWidgetArea) {
        win->removeDockWidget(dock);
        win->addDockWidget(Qt::LeftDockWidgetArea, dock);
        dock->setVisible(true);
        dock->setFloating(false);
    }
    event->accept();
}

void ActivityBar::mouseReleaseEvent(QMouseEvent* event) {
    if (dragging_) {
        dragging_ = false;
        setCursor(Qt::OpenHandCursor);
        event->accept();
        return;
    }
    QFrame::mouseReleaseEvent(event);
}

} // namespace gui
