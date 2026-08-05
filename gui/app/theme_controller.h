// gui/app/theme_controller.h — application-wide theme (background color + light/dark mode).
//
// Provides:
//   - 5 background color choices (Light Gray default, Light Green,
//     Light Yellow, Light Pink, Light Blue). Each color has a
//     corresponding dark variant used when dark mode is active — so dark
//     mode is NOT a single black, it's the dark variant of the chosen hue.
//   - 3 mode options: Follow System (default), Always Light, Always Dark.
//   - Automatic text color: black in light mode, white in dark mode.
//
// When "Follow System" is selected, the controller listens to
// QGuiApplication's colorScheme hint (Qt 6.5+) and switches automatically.
//
// Settings are persisted via QSettings so the user's choice survives restarts.

#ifndef GUI_APP_THEME_CONTROLLER_H
#define GUI_APP_THEME_CONTROLLER_H

#include <QObject>
#include <QString>
#include <vector>

class QMainWindow;
class QMenu;
class QActionGroup;

// Forward declarations — these are GLOBAL Qt classes, NOT in namespace gui.
// Using "class QAction" inside namespace gui would create gui::QAction.
class QAction;

namespace gui {

class ThemeController : public QObject {
    Q_OBJECT
public:
    enum class Color {
        LightGray = 0,   ///< Default.
        LightGreen = 1,
        LightYellow = 2,
        LightPink = 3,
        LightBlue = 4,
    };
    Q_ENUM(Color)

    enum class Mode {
        FollowSystem = 0,  ///< Default — tracks QGuiApplication colorScheme.
        AlwaysLight = 1,
        AlwaysDark = 2,
    };
    Q_ENUM(Mode)

    explicit ThemeController(QObject* parent = nullptr);

    /// @brief Attaches the controller to a main window. Subsequent theme
    /// changes will re-apply the stylesheet to this window and its children.
    void set_target(QMainWindow* window);

    /// @brief Builds a "Theme" submenu inside @p parent_menu with radio
    /// actions for color and mode selection. The menu is kept in sync with
    /// the controller's state.
    void build_menu(QMenu* parent_menu);

    Color color() const { return color_; }
    Mode mode() const { return mode_; }

    /// @brief Sets the background color choice (one of the 5 colors).
    /// Persists immediately and re-applies the stylesheet.
    void set_color(Color c);

    /// @brief Sets the light/dark mode preference.
    /// Persists immediately and re-applies the stylesheet.
    void set_mode(Mode m);

    /// @brief Returns the human-readable name of a color choice.
    static QString color_name(Color c);
    /// @brief Returns the human-readable name of a mode choice.
    static QString mode_name(Mode m);

    /// @brief Returns the effective panel color (hex) — slightly different
    /// from the background, used for the title bar and status bar so they
    /// share the same shade while staying distinct from the sidebar.
    QString effective_panel_hex() const;
    /// @brief Returns the effective text color (hex) — black in light mode,
    /// white in dark mode.
    QString effective_text_hex() const;
    /// @brief Returns the panel color (hex) for the current color choice but
    /// the specified @p dark mode (instead of the effective mode). Used for
    /// cross-mode color computations (e.g. the title bar's inverse-mode
    /// rounded box background and the always-dark-mode title text color).
    QString panel_hex(bool dark) const;

    /// @brief Resolves the current Mode (+ system color scheme when
    /// FollowSystem) to a concrete light/dark flag.
    bool is_dark_mode() const;

signals:
    /// @brief Emitted whenever the effective background/text color changes
    /// (either because the user changed a setting, or because the system
    /// color scheme changed while in FollowSystem mode).
    void theme_changed();

private:
    void load_settings();
    void save_settings() const;
    void apply_stylesheet();
    void on_system_color_scheme_changed();
    void sync_menu_actions();

    QMainWindow* window_{nullptr};
    Color color_{Color::LightBlue};
    Mode mode_{Mode::FollowSystem};

    /// Cached system color scheme, refreshed from
    /// QGuiApplication::styleHints()->colorScheme(). When Qt < 6.5 is used
    /// the API is unavailable and we treat the system as Light.
    bool system_is_dark_{false};

    // Menu state (built lazily by build_menu).
    QMenu* menu_{nullptr};
    QActionGroup* color_group_{nullptr};
    QActionGroup* mode_group_{nullptr};
    std::vector<QAction*> color_actions_;
    std::vector<QAction*> mode_actions_;
};

} // namespace gui

#endif // GUI_APP_THEME_CONTROLLER_H
