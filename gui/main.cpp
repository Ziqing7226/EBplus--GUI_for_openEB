// gui/main.cpp — application entry point.

#include <QApplication>
#include <QFont>
#include <QSurfaceFormat>
#include <cstdlib>
#include <cstring>
#include <string>

#include "main_window.h"

// Sets default OpenEB runtime env vars only if the user has not already
// configured them.  This allows users with non-standard HAL plugin locations
// (e.g. CenturyArks cameras) to simply export MV_HAL_PLUGIN_PATH in their
// shell and have the GUI respect it.
static void ensure_openeb_env_defaults() {
    // HAL plugin path — where the SDK looks for camera driver .so files.
    // Default: Prophesee openEB install location. Override by exporting
    // MV_HAL_PLUGIN_PATH before launching the app.
    if (!std::getenv("MV_HAL_PLUGIN_PATH")) {
        ::setenv("MV_HAL_PLUGIN_PATH", "/usr/local/lib/metavision/hal/plugins", 0);
    }
    // HDF5 plugin path — needed for reading HDF5 event files.
    if (!std::getenv("HDF5_PLUGIN_PATH")) {
        ::setenv("HDF5_PLUGIN_PATH", "/usr/local/lib/hdf5/plugin", 0);
    }
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
    ensure_openeb_env_defaults();

    // Request a core-profile OpenGL 3.3 context for the display widget.
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setSwapInterval(1); // vsync on
    QSurfaceFormat::setDefaultFormat(fmt);

    QApplication app(argc, argv);
    QApplication::setApplicationName("GUI for openEB");
    QApplication::setOrganizationName("GUI-for-openEB");
    QApplication::setApplicationVersion("2.0.0");

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
