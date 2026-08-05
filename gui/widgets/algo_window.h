// gui/widgets/algo_window.h — algorithm display dock (output only).
//
// Design §5.6.6: every self-developed algorithm exposes an AlgoWindow after
// being enabled. The window shows only the algorithm title (window title) and
// its output: for Standalone algorithms it hosts the result display widget
// (EventDisplayWidget for frame producers, QLabel for text producers); for
// Overlay/Replace/Passive algorithms the display area shows the algorithm's
// status string and the visual output remains on the main display frame.
//
// All algorithm parameter adjustment is handled exclusively by the sidebar
// (AlgorithmsPanel). Keeping parameters in a single location avoids the
// synchronization problem where two independent parameter panels (sidebar +
// display window) could drift out of sync, misleading the user about which
// values the algorithm actually uses.
//
// AlgoWindow is a QDockWidget so it docks into the MainWindow (default:
// left edge), can be dragged to any edge, floated, or tabbed with other
// algo windows. This keeps all algorithm outputs within the main window
// frame instead of spawning separate floating windows.
//
// The window is opened by MainWindow when the algorithm is enabled, and is
// closed automatically when the algorithm is disabled. Closing the window
// disables the algorithm and unchecks the sidebar checkbox.

#ifndef GUI_WIDGETS_ALGO_WINDOW_H
#define GUI_WIDGETS_ALGO_WINDOW_H

#include <QDockWidget>
#include <QLabel>
#include <QString>
#include <QWidget>
#include <memory>
#include <string>

#include "algo_bridge/algo_bridge.h"

class QVBoxLayout;

namespace gui {

class EventDisplayWidget;

class AlgoWindow : public QDockWidget {
    Q_OBJECT
public:
    /// @brief Constructs an AlgoWindow for the named algorithm.
    /// @p bridge must outlive the window. The constructor finds (or creates)
    /// the live AlgoInstance and enables it.
    AlgoWindow(AlgoBridge* bridge, const std::string& algo_name,
               QWidget* parent = nullptr);

    /// @brief Returns the algorithm name this window manages.
    const std::string& algo_name() const { return algo_name_; }

    /// @brief Returns the live instance managed by this window (may be null
    /// if the algorithm was unknown).
    std::shared_ptr<AlgoInstance> instance() const { return instance_; }

    /// @brief Returns the frame display widget if one was installed, else
    /// nullptr. Used by MainWindow to route Standalone frame results.
    EventDisplayWidget* frame_display() const;

    /// @brief Installs a custom display widget, replacing the default status
    /// label. Ownership transfers to the window's layout.
    void set_display_widget(QWidget* w);

    /// @brief Updates the status label text (if present). Called from
    /// MainWindow::process_algo_results for text-based Standalone algos.
    void set_status_text(const QString& text);

signals:
    /// @brief Emitted when the window is about to close (closeEvent).
    /// MainWindow connects this to disable the algo instance and uncheck
    /// the sidebar checkbox.
    void closing(const std::string& algo_name);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    AlgoBridge* bridge_;
    std::string algo_name_;
    AlgoInfo info_;
    std::shared_ptr<AlgoInstance> instance_;

    QWidget* content_{nullptr};  ///< Inner widget set via QDockWidget::setWidget.
    QLabel* status_label_{nullptr};
    QWidget* display_widget_{nullptr};
    QVBoxLayout* display_layout_{nullptr};
};

} // namespace gui

#endif // GUI_WIDGETS_ALGO_WINDOW_H
