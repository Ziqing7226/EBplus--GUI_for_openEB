// gui/exporter/exporter_controller.h — export event recordings (design §3.6).
//
// Converts a source event file (RAW/HDF5/DAT) to:
//   • HDF5 — via Metavision::HDF5EventFileWriter
//   • AVI  — via Metavision::CDFrameGenerator + Metavision::CvVideoRecorder
//
// Runs the conversion on a background thread; emits progress [0..1] and a
// completion signal. Aborts cleanly if cancel() is called.

#ifndef GUI_EXPORTER_EXPORTER_CONTROLLER_H
#define GUI_EXPORTER_EXPORTER_CONTROLLER_H

#include <QObject>
#include <QString>
#include <atomic>
#include <memory>
#include <thread>

namespace gui {

struct ExportParams {
    QString source_path;
    QString output_path;
    enum class Format { HDF5, AVI } format{Format::HDF5};
    // AVI-only options:
    int fps{30};
    int accumulation_us{33000};
    int quality{90};          // 1..100 (codec selection heuristic)
    bool color{true};
};

class ExporterController : public QObject {
    Q_OBJECT
public:
    explicit ExporterController(QObject* parent = nullptr);
    ~ExporterController();

    /// @brief Starts an export. Returns false if an export is already running
    /// (the caller should show an error in that case so the UI doesn't hang).
    bool start(const ExportParams& params);
    void cancel();

signals:
    void progress(double ratio);          // 0..1
    void completed(const QString& output);
    void failed(const QString& message);

private:
    void run_hdf5(const ExportParams& p);
    void run_avi(const ExportParams& p);

    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_{false};
    std::thread worker_;
};

} // namespace gui

#endif // GUI_EXPORTER_EXPORTER_CONTROLLER_H
