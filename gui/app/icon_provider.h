// gui/app/icon_provider.h — recolorable SVG icon provider (Lucide-based).
//
// Icons are loaded from the Qt resource system at ":/icons/<name>.svg" and
// recolored by replacing the SVG's "currentColor" with the requested color,
// then rendered directly via QSvgRenderer. This gives cross-platform consistent icons
// that follow the active theme's foreground color.
//
// Rendered icons are cached per (name, color) pair so repeated requests for
// the same icon (e.g. on theme refresh) skip file I/O and SVG parsing.

#ifndef GUI_APP_ICON_PROVIDER_H
#define GUI_APP_ICON_PROVIDER_H

#include <QColor>
#include <QIcon>
#include <QString>

namespace gui {

class IconProvider {
public:
    /// Returns @p name recolored with the active theme's primary foreground
    /// color (read from the application palette so it always tracks the
    /// current ThemeController selection).
    static QIcon get(const QString& name);

    /// Returns @p name recolored with the explicit @p color. Use this for
    /// status indicators (green / red / gray dots, etc.).
    static QIcon get(const QString& name, const QColor& color);

    /// Clears the icon cache so subsequent get() calls re-render icons with
    /// the current theme color (BUG-R7: without this, stale icons from the
    /// previous theme persist after a color switch).
    static void clear_cache();

private:
    /// Renders the SVG named @p name with @p color into a QIcon. Returns a
    /// null icon if the resource is missing or invalid.
    static QIcon render(const QString& name, const QColor& color);
};

} // namespace gui

#endif // GUI_APP_ICON_PROVIDER_H
