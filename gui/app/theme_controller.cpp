// gui/app/theme_controller.cpp

#include "theme_controller.h"

#include <QAction>
#include <QActionGroup>
#include <QApplication>
#include <QFile>
#include <QMainWindow>
#include <QMenu>
#include <QPalette>
#include <QSettings>
#include <QStyleHints>
#include <QColor>

#include "resources/theme/tokens.h"

namespace gui {

namespace {

/// Returns true if Qt exposes a system color-scheme hint (Qt 6.5+).
/// Sets @p dark_out to the dark-mode flag when available.
bool query_system_dark(bool& dark_out) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    if (auto* app = qobject_cast<QApplication*>(QCoreApplication::instance())) {
        const auto scheme = app->styleHints()->colorScheme();
        dark_out = (scheme == Qt::ColorScheme::Dark);
        return true;
    }
#else
    (void)dark_out;
#endif
    return false;
}

} // namespace

ThemeController::ThemeController(QObject* parent) : QObject(parent) {
    load_settings();

    // Query the initial system color scheme (best-effort).
    (void)query_system_dark(system_is_dark_);

#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
    if (auto* app = qobject_cast<QApplication*>(QCoreApplication::instance())) {
        connect(app->styleHints(), &QStyleHints::colorSchemeChanged, this,
                [this](Qt::ColorScheme) { on_system_color_scheme_changed(); });
    }
#endif
}

void ThemeController::set_target(QMainWindow* window) {
    window_ = window;
    apply_stylesheet();
}

void ThemeController::set_color(Color c) {
    if (c == color_) return;
    color_ = c;
    save_settings();
    apply_stylesheet();
    sync_menu_actions();
    emit theme_changed();
}

void ThemeController::set_mode(Mode m) {
    if (m == mode_) return;
    mode_ = m;
    save_settings();
    apply_stylesheet();
    sync_menu_actions();
    emit theme_changed();
}

QString ThemeController::color_name(Color c) {
    switch (c) {
        case Color::LightGray:   return tr("Gray");
        case Color::LightGreen:  return tr("Green");
        case Color::LightYellow: return tr("Yellow");
        case Color::LightPink:   return tr("Pink");
        case Color::LightBlue:   return tr("Blue");
    }
    return tr("Gray");
}

QString ThemeController::mode_name(Mode m) {
    switch (m) {
        case Mode::FollowSystem: return tr("Follow System");
        case Mode::AlwaysLight:  return tr("Light");
        case Mode::AlwaysDark:   return tr("Dark");
    }
    return tr("Follow System");
}

QString ThemeController::effective_panel_hex() const {
    return theme_tokens::lookup(static_cast<int>(color_), is_dark_mode(),
                                theme_tokens::Token::BgPanel);
}

QString ThemeController::effective_text_hex() const {
    return theme_tokens::lookup(static_cast<int>(color_), is_dark_mode(),
                                theme_tokens::Token::FgPrimary);
}

QString ThemeController::panel_hex(bool dark) const {
    return theme_tokens::lookup(static_cast<int>(color_), dark,
                                theme_tokens::Token::BgPanel);
}

bool ThemeController::is_dark_mode() const {
    switch (mode_) {
        case Mode::FollowSystem: return system_is_dark_;
        case Mode::AlwaysLight:  return false;
        case Mode::AlwaysDark:   return true;
    }
    return false;
}

void ThemeController::build_menu(QMenu* parent_menu) {
    if (!parent_menu) return;
    menu_ = parent_menu;

    // --- Color submenu ---
    auto* color_menu = menu_->addMenu(tr("Color"));
    color_group_ = new QActionGroup(menu_);
    color_group_->setExclusive(true);
    color_actions_.clear();
    for (int i = 0; i <= static_cast<int>(Color::LightBlue); ++i) {
        const auto c = static_cast<Color>(i);
        auto* a = color_menu->addAction(color_name(c));
        a->setCheckable(true);
        a->setChecked(color_ == c);
        color_group_->addAction(a);
        connect(a, &QAction::triggered, this, [this, c]() { set_color(c); });
        color_actions_.push_back(a);
    }

    // --- Mode submenu ---
    auto* mode_menu = menu_->addMenu(tr("Mode"));
    mode_group_ = new QActionGroup(menu_);
    mode_group_->setExclusive(true);
    mode_actions_.clear();
    for (int i = 0; i <= static_cast<int>(Mode::AlwaysDark); ++i) {
        const auto m = static_cast<Mode>(i);
        auto* a = mode_menu->addAction(mode_name(m));
        a->setCheckable(true);
        a->setChecked(mode_ == m);
        mode_group_->addAction(a);
        connect(a, &QAction::triggered, this, [this, m]() { set_mode(m); });
        mode_actions_.push_back(a);
    }
}

void ThemeController::sync_menu_actions() {
    for (size_t i = 0; i < color_actions_.size(); ++i) {
        if (color_actions_[i]) {
            QSignalBlocker b(color_actions_[i]);
            color_actions_[i]->setChecked(static_cast<int>(color_) == static_cast<int>(i));
        }
    }
    for (size_t i = 0; i < mode_actions_.size(); ++i) {
        if (mode_actions_[i]) {
            QSignalBlocker b(mode_actions_[i]);
            mode_actions_[i]->setChecked(static_cast<int>(mode_) == static_cast<int>(i));
        }
    }
}

void ThemeController::load_settings() {
    QSettings s;
    // Default color: Blue. Default mode: Follow System.
    color_ = static_cast<Color>(s.value("theme/color", static_cast<int>(Color::LightBlue)).toInt());
    mode_  = static_cast<Mode>(s.value("theme/mode",  static_cast<int>(Mode::FollowSystem)).toInt());
    // Clamp to valid enum ranges to guard against corrupted settings.
    if (color_ < Color::LightGray || color_ > Color::LightBlue) color_ = Color::LightBlue;
    if (mode_  < Mode::FollowSystem || mode_  > Mode::AlwaysDark)  mode_  = Mode::FollowSystem;
}

void ThemeController::save_settings() const {
    QSettings s;
    s.setValue("theme/color", static_cast<int>(color_));
    s.setValue("theme/mode",  static_cast<int>(mode_));
}

void ThemeController::apply_stylesheet() {
    if (!window_) return;
    const int color_idx = static_cast<int>(color_);
    const bool dark = is_dark_mode();
    using T = theme_tokens::Token;

    // Load the token-templated stylesheet from the Qt resource and inject the
    // current theme's token values via placeholder replacement.
    QString qss;
    QFile tpl(QStringLiteral(":/theme/base.qss.in"));
    if (tpl.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qss = QString::fromUtf8(tpl.readAll());
        const auto sub = [&qss, color_idx, dark](const char* ph, T t) {
            qss.replace(QString::fromLatin1(ph),
                        theme_tokens::lookup(color_idx, dark, t));
        };
        sub("%bg-primary%", T::BgPrimary);
        sub("%bg-panel%", T::BgPanel);
        sub("%bg-input%", T::BgInput);
        sub("%bg-hover%", T::BgHover);
        sub("%fg-primary%", T::FgPrimary);
        sub("%fg-secondary%", T::FgSecondary);
        sub("%fg-muted%", T::FgMuted);
        sub("%accent%", T::Accent);
        sub("%border%", T::Border);
    }
    window_->setStyleSheet(qss);

    // Mirror the token values into the application palette so palette-derived
    // widgets (and IconProvider, which reads WindowText) follow the theme.
    const QColor bg_primary(theme_tokens::lookup(color_idx, dark, T::BgPrimary));
    const QColor fg_primary(theme_tokens::lookup(color_idx, dark, T::FgPrimary));
    const QColor bg_input(theme_tokens::lookup(color_idx, dark, T::BgInput));
    const QColor bg_panel(theme_tokens::lookup(color_idx, dark, T::BgPanel));
    const QColor accent(theme_tokens::lookup(color_idx, dark, T::Accent));
    const QColor bg_hover(theme_tokens::lookup(color_idx, dark, T::BgHover));
    const QColor border(theme_tokens::lookup(color_idx, dark, T::Border));

    QPalette pal;
    pal.setColor(QPalette::Window, bg_primary);
    pal.setColor(QPalette::WindowText, fg_primary);
    pal.setColor(QPalette::Base, bg_input);
    pal.setColor(QPalette::AlternateBase, bg_panel);
    pal.setColor(QPalette::Text, fg_primary);
    pal.setColor(QPalette::Button, bg_panel);
    pal.setColor(QPalette::ButtonText, fg_primary);
    pal.setColor(QPalette::ToolTipBase, bg_panel);
    pal.setColor(QPalette::ToolTipText, fg_primary);
    pal.setColor(QPalette::Highlight, accent);
    pal.setColor(QPalette::HighlightedText, QColor(Qt::white));
    // §15.4: set Mid/Midlight explicitly so QSS palette(midlight) references
    // (e.g. ActivityBar :checked/:hover) use the theme's bg-hover token
    // instead of Qt's auto-computed default which is far too bright in dark
    // mode.
    pal.setColor(QPalette::Midlight, bg_hover);
    pal.setColor(QPalette::Mid, border);
    pal.setColor(QPalette::Light, bg_hover);
    pal.setColor(QPalette::Dark, border);
    if (auto* app = qobject_cast<QApplication*>(QCoreApplication::instance())) {
        app->setPalette(pal);
    }
}

void ThemeController::on_system_color_scheme_changed() {
    bool dark = false;
    if (!query_system_dark(dark)) return;
    if (dark == system_is_dark_) return;
    system_is_dark_ = dark;
    if (mode_ == Mode::FollowSystem) {
        apply_stylesheet();
        sync_menu_actions();
        emit theme_changed();
    }
}

} // namespace gui
