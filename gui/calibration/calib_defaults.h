// gui/calibration/calib_defaults.h — single source of truth for the default
// intrinsic-calibration file path.
//
// BOTH consumers must always agree: the Intrinsic Wizard's default EXPORT
// path (where the calibrated intrinsics land) and the Preprocessor undistort
// stage's default intrinsic path (what it loads). Keep it that way — a user
// who calibrates with the defaults and enables Undistort with the defaults
// must get the file they just wrote, with no path typing.

#ifndef GUI_CALIBRATION_CALIB_DEFAULTS_H
#define GUI_CALIBRATION_CALIB_DEFAULTS_H

#include <QString>
#include <QDir>
#include <QStandardPaths>

namespace gui {

/// @brief Default intrinsic-calibration YAML path:
///        <Documents>/EBplus/calibration/intrinsic.yml (home dir fallback).
/// Stable filename (no timestamp) so both defaults always point at the same
/// file; the parent directory is created on wizard export (auto-mkdir).
inline QString default_intrinsic_yml_path() {
    const QString docs = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const QString base = docs.isEmpty() ? QDir::homePath() : docs;
    return base + QStringLiteral("/EBplus/calibration/intrinsic.yml");
}

} // namespace gui

#endif // GUI_CALIBRATION_CALIB_DEFAULTS_H
