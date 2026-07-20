// gui/main.cpp — application entry point.

#include <QApplication>
#include <QFont>
#include <QSurfaceFormat>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include "main_window.h"

namespace {

std::string path_list(const std::vector<std::filesystem::path> &paths) {
    std::string value;
    for (const auto &path : paths) {
        if (path.empty() || !std::filesystem::exists(path)) {
            continue;
        }
        if (!value.empty()) {
            value += ':';
        }
        value += path.string();
    }
    return value;
}

void set_env_if_unset(const char *name, const std::string &value) {
    if (!std::getenv(name) && !value.empty()) {
        ::setenv(name, value.c_str(), 0);
    }
}

#ifdef __APPLE__
std::filesystem::path executable_path(const char *argv0) {
    uint32_t size = 0;
    _NSGetExecutablePath(nullptr, &size);
    std::string buffer(size, '\0');
    if (_NSGetExecutablePath(buffer.data(), &size) == 0) {
        buffer.resize(std::strlen(buffer.c_str()));
        return std::filesystem::weakly_canonical(buffer);
    }
    return std::filesystem::weakly_canonical(argv0);
}

std::filesystem::path bundle_contents_dir(const char *argv0) {
    const auto exe_dir = executable_path(argv0).parent_path();
    if (exe_dir.filename() == "MacOS" && exe_dir.parent_path().filename() == "Contents") {
        return exe_dir.parent_path();
    }
    return {};
}
#endif

} // namespace

// Sets default OpenEB runtime env vars only if the user has not already
// configured them.  This allows users with non-standard HAL plugin locations
// (e.g. CenturyArks cameras) to simply export MV_HAL_PLUGIN_PATH in their
// shell and have the GUI respect it.
static void ensure_openeb_env_defaults(const char *argv0) {
    // HAL plugin path — where the SDK looks for camera driver .so files.
    // Default: Prophesee openEB install location. Override by exporting
    // MV_HAL_PLUGIN_PATH before launching the app.
#ifdef __APPLE__
    const auto contents_dir = bundle_contents_dir(argv0);
    set_env_if_unset("MV_HAL_PLUGIN_PATH",
                     path_list({
                         contents_dir / "PlugIns" / "metavision" / "hal" / "plugins",
                     }));
    // The app bundle is self-contained. Avoid OpenEB loading this directory a
    // second time through its installation-path fallback.
    set_env_if_unset("MV_HAL_PLUGIN_SEARCH_MODE", "PLUGIN_PATH_ONLY");
    set_env_if_unset("MV_HAL_INSTALL_PATH", path_list({contents_dir / "Resources" / "metavision" / "hal"}));
#else
    set_env_if_unset("MV_HAL_PLUGIN_PATH", "/usr/local/lib/metavision/hal/plugins");
#endif
    // HDF5 plugin path — needed for reading HDF5 event files.
#ifdef __APPLE__
    set_env_if_unset("HDF5_PLUGIN_PATH",
                     path_list({
                         contents_dir / "PlugIns" / "hdf5",
                         "/opt/homebrew/lib/hdf5/plugin",
                         "/usr/local/lib/hdf5/plugin",
                     }));
#else
    set_env_if_unset("HDF5_PLUGIN_PATH", "/usr/local/lib/hdf5/plugin");
#endif
    // On Wayland sessions Qt 6's Wayland plugin renders a black viewport for
    // our QOpenGLWidget; the XCB plugin (via XWayland) is the reliable path.
    // Respect any user override.
    if (!std::getenv("QT_QPA_PLATFORM") && std::getenv("WAYLAND_DISPLAY")) {
        ::setenv("QT_QPA_PLATFORM", "xcb", 0);
    }
    // Qt 6 may default to the Vulkan RHI backend and produce a black viewport
    // on this GPU; force OpenGL unless the user has set it.
    if (!std::getenv("QSG_RHI_BACKEND")) {
        ::setenv("QSG_RHI_BACKEND", "opengl", 0);
    }
    // Note: LD_LIBRARY_PATH cannot be set here (the dynamic linker has
    // already resolved shared libraries at process start).  If the SDK
    // libraries are in a non-standard path, set it in the shell or use
    // the run.sh launcher.
}

int main(int argc, char* argv[]) {
    ensure_openeb_env_defaults(argc > 0 ? argv[0] : "");

    // Request a core-profile OpenGL 3.3 context for the display widget.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSwapInterval(1); // vsync on
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    QApplication::setApplicationName("GUI for openEB");
    QApplication::setOrganizationName("GUI-for-openEB");
    QApplication::setApplicationVersion("1.9.0");

    // Global UI font — Inter (design §3.9.1) with platform fallbacks so the
    // typeface stays consistent on systems without Inter installed.
    QFont font(QStringLiteral("Inter"), 10);
    font.setFamilies({QStringLiteral("Inter"),
                      QStringLiteral("Segoe UI"),
                      QStringLiteral("Ubuntu"),
                      QStringLiteral("Noto Sans"),
                      QStringLiteral("Sans Serif")});
    font.setStyleStrategy(QFont::PreferAntialias);
    app.setFont(font);

    gui::MainWindow window;
    window.show();

    return app.exec();
}
