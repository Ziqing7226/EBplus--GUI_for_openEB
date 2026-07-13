// gui/widgets/activity_bar.h — VSCode-style Activity Bar.
//
// A narrow (48px) vertical icon column used as the sidebar's primary
// navigation. Each entry is a checkable icon button; entries are mutually
// exclusive (QButtonGroup). Selecting an entry emits group_selected() so
// the host (SettingsPanel) can switch the QStackedWidget page.
//
// A non-checkable toggle button sits at the very bottom (after a stretch).
// It controls sidebar content visibility (hide/show the QStackedWidget).
// The host sets its icon via set_toggle_icon() based on dock position and
// visibility state.
//
// Dragging: since the dock has no title bar (§11.2 point 5), the ActivityBar's
// blank areas (between buttons, stretch area) serve as the drag handle.
// Clicking and dragging across the window's horizontal center moves the dock
// to the opposite side (Left↔Right). The cursor changes to OpenHandCursor
// on blank areas to indicate draggability (§12.2.1).
//
// Design (gui_optimization.md §10.3 + §11.2 + §12.2):
//   - Group buttons: 40x40, icon 20x20, spacing ~7px (1/3 icon size).
//   - The active button shows a 2px accent strip on its left edge.
//   - Tooltips describe each group so users can discover functionality.
//   - Toggle button at bottom: chevron-left/right depending on dock state.
//   - Icons are recolored via IconProvider so they track the active theme.

#ifndef GUI_WIDGETS_ACTIVITY_BAR_H
#define GUI_WIDGETS_ACTIVITY_BAR_H

#include <QFrame>
#include <QPoint>
#include <QString>
#include <vector>

class QPushButton;
class QButtonGroup;
class QVBoxLayout;
class QDockWidget;

namespace gui {

class ActivityBar : public QFrame {
    Q_OBJECT
public:
    explicit ActivityBar(QWidget* parent = nullptr);

    /// Adds a group button at the bottom of the group button column.
    /// @p icon_name is the resource name (without extension) loaded via
    /// IconProvider; @p title is the human-readable group name; @p tooltip
    /// is the hover description. Returns the index of the new entry.
    int add_button(const QString& icon_name, const QString& title,
                   const QString& tooltip);

    /// Selects the entry at @p index and emits group_selected(). Safe to call
    /// before any button is added (no-op). Out-of-range indices are clamped.
    void select(int index);

    /// Returns the index of the currently selected entry, or -1 if none.
    int current_index() const;

    /// Returns the title of the entry at @p index, or an empty string.
    QString title_at(int index) const;

    /// Sets the toggle button's icon. Called by the host when dock visibility
    /// or dock area changes.
    void set_toggle_icon(const QString& icon_name);

    /// Re-renders all icons (group buttons + toggle) with the current theme's
    /// foreground color. Called by the host when the theme changes.
    /// Also forces a QSS re-polish so palette() references in the local
    /// stylesheet pick up the new theme colors (§12.2.2).
    void refresh_icons();

signals:
    /// Emitted whenever the selected entry changes (user click or select()).
    void group_selected(int index, const QString& title);

    /// Emitted when the toggle button is clicked. The host hides/shows the
    /// sidebar content area in response.
    void toggle_clicked();

protected:
    /// Detects clicks on blank areas (non-button) to initiate sidebar drag.
    void mousePressEvent(QMouseEvent* event) override;
    /// Moves the dock to the opposite side if the mouse crosses the window
    /// center while dragging.
    void mouseMoveEvent(QMouseEvent* event) override;
    /// Clears the drag state.
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    /// Walks up the parent hierarchy to find the QDockWidget that contains
    /// this ActivityBar. Returns nullptr if not inside a dock.
    QDockWidget* find_dock() const;

    struct Entry {
        QPushButton* button;
        QString icon_name;
        QString title;
    };
    std::vector<Entry> entries_;
    QButtonGroup* group_;
    QVBoxLayout* group_layout_;  ///< Sub-layout for group buttons (above stretch).
    QPushButton* toggle_btn_;    ///< Toggle button at the very bottom.
    QString toggle_icon_name_;   ///< Current toggle icon name (for refresh).

    /// Drag state: true between mousePress and mouseRelease on a blank area.
    bool dragging_{false};
    /// Global position where the drag started.
    QPoint drag_start_pos_;
    /// The dock area the dock was in when the drag started.
    Qt::DockWidgetArea drag_start_area_{Qt::LeftDockWidgetArea};
};

} // namespace gui

#endif // GUI_WIDGETS_ACTIVITY_BAR_H
