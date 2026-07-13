// gui/resources/theme/tokens.h
//
// Semantic color tokens for the 5 pastel themes (Gray / Green / Yellow /
// Pink / Blue) in both Light and Dark modes — 10 variants total.
//
// Indexed as [color][mode][token] where:
//   - color matches ThemeController::Color order (Gray=0 .. Blue=4)
//   - mode  is Light=0 / Dark=1
//   - token selects one of the 9 semantic slots below
//
// The Dark-mode `fg-muted` values are tuned so that hint text reaches
// WCAG AA contrast (>= 4.5:1) against the corresponding Dark `bg-primary`
// (see design doc §7.1). The 5 background colors are the EXISTING pastel
// set preserved from ThemeController — they are not changed.

#ifndef GUI_RESOURCES_THEME_TOKENS_H
#define GUI_RESOURCES_THEME_TOKENS_H

#include <QString>

namespace gui {
namespace theme_tokens {

// Color index — MUST match ThemeController::Color numeric order
// (LightGray=0, LightGreen=1, LightYellow=2, LightPink=3, LightBlue=4).
enum ColorIndex : int {
    kGray = 0,
    kGreen = 1,
    kYellow = 2,
    kPink = 3,
    kBlue = 4,
    kColorCount = 5,
};

enum ModeIndex : int {
    kLight = 0,
    kDark = 1,
    kModeCount = 2,
};

enum class Token : int {
    BgPrimary = 0,    // window background
    BgPanel,          // panel / dock / toolbar / menubar background
    BgInput,          // input control background
    BgHover,          // hover background
    FgPrimary,        // primary text
    FgSecondary,      // secondary text
    FgMuted,          // hint / placeholder text (>= 4.5:1 in dark modes)
    Accent,           // focus / selection accent
    Border,           // control border
    kTokenCount = 9,
};

// [color][mode][token] -> hex string (without leading '#'); e.g. "E8E8E8".
// Kept as raw const char* so ThemeController can build "#RRGGBB" via replace.
constexpr const char* kValues[kColorCount][kModeCount][9] = {
    // ---- Gray ----
    {
        // Light Gray
        {
            "E8E8E8",  // bg-primary
            "F0F0F0",  // bg-panel
            "FFFFFF",  // bg-input
            "DCDCDC",  // bg-hover
            "1A1A1A",  // fg-primary
            "5A5A5A",  // fg-secondary
            "5A5A5A",  // fg-muted (BUG-6 fix: 4.5:1 contrast)
            "0066B8",  // accent
            "C0C0C0",  // border
        },
        // Dark Gray
        {
            "2D2D30",  // bg-primary
            "252525",  // bg-panel
            "3C3C3C",  // bg-input
            "3A3A3A",  // bg-hover
            "F0F0F0",  // fg-primary
            "B0B0B0",  // fg-secondary
            "9A9A9A",  // fg-muted  (4.6:1 on #2D2D30)
            "4A9FE0",  // accent
            "555555",  // border
        },
    },
    // ---- Green ----
    {
        // Light Green
        {
            "D7EBD4",  // bg-primary
            "E2F0DE",  // bg-panel
            "FFFFFF",  // bg-input
            "C4DCC4",  // bg-hover
            "1A2A1A",  // fg-primary
            "3A5A3A",  // fg-secondary
            "4A6A4A",  // fg-muted
            "2E7D32",  // accent
            "A8C8A8",  // border
        },
        // Dark Green
        {
            "1E3A22",  // bg-primary
            "1A331E",  // bg-panel
            "243F28",  // bg-input
            "28492E",  // bg-hover
            "E8F0E8",  // fg-primary
            "A8C0A8",  // fg-secondary
            "8FA68F",  // fg-muted  (4.76:1 on #1E3A22, WCAG AA)
            "66BB6A",  // accent
            "3A553E",  // border
        },
    },
    // ---- Yellow ----
    {
        // Light Yellow
        {
            "FDF4D2",  // bg-primary
            "FEF8E2",  // bg-panel
            "FFFFFF",  // bg-input
            "F0E8C4",  // bg-hover
            "2A2410",  // fg-primary
            "5A4A2A",  // fg-secondary
            "6A5A3A",  // fg-muted
            "B8860B",  // accent
            "D8C8A0",  // border
        },
        // Dark Yellow
        {
            "3D3520",  // bg-primary
            "38301E",  // bg-panel
            "443B26",  // bg-input
            "48402A",  // bg-hover
            "F0E8D0",  // fg-primary
            "C0B898",  // fg-secondary
            "B0A080",  // fg-muted  (4.73:1 on #3D3520, WCAG AA)
            "D4A843",  // accent
            "5A4F36",  // border
        },
    },
    // ---- Pink ----
    {
        // Light Pink
        {
            "F8D7DC",  // bg-primary
            "FCE6EA",  // bg-panel
            "FFFFFF",  // bg-input
            "F0C4CA",  // bg-hover
            "2A1015",  // fg-primary
            "5A2A35",  // fg-secondary
            "6A3A45",  // fg-muted
            "AD1457",  // accent
            "D8A8B0",  // border
        },
        // Dark Pink
        {
            "3D2030",  // bg-primary
            "361E2C",  // bg-panel
            "422836",  // bg-input
            "482C3C",  // bg-hover
            "F0E0E8",  // fg-primary
            "C0A8B8",  // fg-secondary
            "A88898",  // fg-muted  (5.0:1 on #3D2030)
            "E06090",  // accent
            "5A3848",  // border
        },
    },
    // ---- Blue ----
    {
        // Light Blue
        {
            "D4E6F1",  // bg-primary
            "E0EEF7",  // bg-panel
            "FFFFFF",  // bg-input
            "C4DAEC",  // bg-hover
            "14233A",  // fg-primary
            "3A5A7A",  // fg-secondary
            "4A6580",  // fg-muted (BUG-6 fix: 4.6:1 contrast)
            "005A9E",  // accent
            "A8C4DC",  // border
        },
        // Dark Blue
        {
            "1E2A3D",  // bg-primary
            "243349",  // bg-panel
            "2A3B52",  // bg-input
            "2E405A",  // bg-hover
            "E8EEF5",  // fg-primary
            "A8BCD0",  // fg-secondary
            "8898AC",  // fg-muted  (4.7:1 on #1E2A3D)
            "5AA8E0",  // accent
            "3A4D66",  // border
        },
    },
};

/// Returns the hex string (with leading '#') for @p color / @p dark / @p token.
inline QString lookup(int color, bool dark, Token token) {
    const int c = (color < 0 || color >= kColorCount) ? kGray : color;
    const int m = dark ? kDark : kLight;
    const int t = static_cast<int>(token);
    return QStringLiteral("#") + QString::fromLatin1(kValues[c][m][t]);
}

} // namespace theme_tokens
} // namespace gui

#endif // GUI_RESOURCES_THEME_TOKENS_H
