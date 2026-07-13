// gui/widgets/custom_title_bar.h — custom title bar for the frameless main window.
//
// Inspired by VSCode's titlebar part (src/vs/workbench/browser/parts/titlebar/
// titlebarPart.ts): the native WM title bar is removed (Qt::FramelessWindowHint)
// and a custom widget is drawn in its place. The background color is set
// directly via setColors(), so it always follows the application theme —
// unlike the native title bar whose color is controlled by the window manager
// and cannot be changed reliably from Qt.
//
// Layout (left to right) per design §3.6.1:
//   [app icon][app name]  [文件▾][视图▾][相机▾][工具▾][帮助▾]  [—][□][✕]
//   left cluster           menu dropdown buttons            window controls
//
// The menus are no longer a top-level QMenuBar; each menu is a QPushButton
// that pops up a QMenu on click (addMenu). Dragging the empty title-bar area
// moves the window (QWindow::startSystemMove). Double-clicking toggles
// maximize/restore.
//
// ResizeGrip provides 8 edge/corner resize handles so the frameless window
// remains resizable (QWindow::startSystemResize).

#ifndef GUI_WIDGETS_CUSTOM_TITLE_BAR_H
#define GUI_WIDGETS_CUSTOM_TITLE_BAR_H

#include <QColor>
#include <QWidget>

class QHBoxLayout;
class QLabel;
class QMenu;
class QPushButton;

namespace gui {

/// @brief Custom title bar widget — replaces the native WM title bar and the
/// old QMenuBar hack. Installed via QMainWindow::setMenuWidget().
class CustomTitleBar : public QWidget {
    Q_OBJECT
public:
    explicit CustomTitleBar(QWidget* parent = nullptr);

    /// Sets the application title text shown on the left.
    void setTitle(const QString& title);
    /// Sets the application icon (by SVG name) shown on the left of the title.
    /// The icon is re-rendered with the title color whenever setColors() is
    /// called, so it tracks the inverse-color rule applied to the title.
    void setAppIcon(const QString& icon_name);

    /// Sets the background and text colors. The background always tracks the
    /// application theme. @p fg is used for menus and window controls; @p
    /// title_fg is used ONLY for the title label and app icon — it should be
    /// pure black/white (inverse of @p bg) so the title is the most eye-
    /// catching element on the bar (§13 — inverse color rule, title only).
    void setColors(const QColor& bg, const QColor& fg, const QColor& title_fg);

    /// Re-renders the window control button icons so they pick up the new
    /// theme's foreground color. Called from MainWindow's theme-changed handler.
    void refresh_icons();

    /// @brief Adds a menu dropdown button labeled @p title.
    /// @return The QMenu owned by the button, so the caller can populate it
    ///         with actions.
    QMenu* addMenu(const QString& title);

    QSize sizeHint() const override { return {0, 36}; }
    QSize minimumSizeHint() const override { return {0, 36}; }

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private:
    /// Re-renders icon_label_ with app_icon_name_ + title_color_.
    void renderAppIcon();

    QLabel* icon_label_{nullptr};
    QLabel* title_label_{nullptr};
    QHBoxLayout* menu_layout_{nullptr};
    QPushButton* btn_min_{nullptr};
    QPushButton* btn_max_{nullptr};
    QPushButton* btn_close_{nullptr};
    QColor bg_color_;
    QColor fg_color_;
    QColor title_color_;      // pure black/white — title label + app icon only
    QString app_icon_name_;   // SVG name for the app icon (e.g. "camera")
    QString min_icon_name_;
    QString max_icon_name_;
    QString close_icon_name_;
};

/// @brief Invisible resize handle on a window edge or corner.
///
/// Positioned by the parent window's resizeEvent. On mouse press, calls
/// QWindow::startSystemResize() which lets the WM handle the resize gesture.
class ResizeGrip : public QWidget {
    Q_OBJECT
public:
    enum class Position {
        Left, Right, Top, Bottom,
        TopLeft, TopRight, BottomLeft, BottomRight
    };
    explicit ResizeGrip(Position pos, QWidget* parent = nullptr);

    /// Repositions the grip within the parent widget's rect.
    void reposition(const QRect& parent_rect);

    static constexpr int kEdgeThickness = 5;
    static constexpr int kCornerSize = 12;

protected:
    void mousePressEvent(QMouseEvent* event) override;

private:
    Position pos_;
};

} // namespace gui

#endif // GUI_WIDGETS_CUSTOM_TITLE_BAR_H
