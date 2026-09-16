// gui/app/external_file_source.h — playback source for file formats the
// Metavision SDK cannot open (AEDAT4 from inivation DV, ALPDATA from
// Alpsentek). Implementations parse the header on open() (GUI thread) and
// stream EventCD batches from run() on a worker thread owned by
// CameraController — the same contract as the SDK's streaming thread feeding
// FramePipeline in file mode.
//
// ONLY event data is extracted: frame/imu/trigger streams in AEDAT4 and APS
// frames in ALPDATA are skipped, matching the GUI's single-camera pure event
// stream scope.

#ifndef GUI_APP_EXTERNAL_FILE_SOURCE_H
#define GUI_APP_EXTERNAL_FILE_SOURCE_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <QString>

#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

struct ExternalFileMeta {
    int width{0};
    int height{0};
    /// Total duration in µs (0 = unknown until fully streamed).
    Metavision::timestamp duration_us{0};
    /// Upper bound on synthesized events (-1 = unknown); feeds the OOM warning.
    std::int64_t worst_case_events{-1};
    /// Suggested accumulation window (0 = no hint). ALPDATA frames carry all
    /// their pixels on one timestamp, so the window is set to the frame
    /// period — every displayed frame then shows exactly one recorded frame.
    Metavision::timestamp accumulation_hint_us{0};
    QString serial;
    QString integrator;
    /// Short format label shown in the Information panel ("AEDAT4"/"ALPDATA").
    QString plugin_name;
    QString encoding_format;
};

class ExternalFileSource {
public:
    /// Sink invoked on the reader thread with a batch of sorted events. The
    /// span is only valid for the duration of the call (the consumer copies).
    using EventSink =
        std::function<void(const Metavision::EventCD*, const Metavision::EventCD*)>;
    /// Completion callback: invoked exactly once at the end of run() with an
    /// empty string on success or a user-readable error message.
    using DoneFn = std::function<void(const std::string&)>;

    virtual ~ExternalFileSource();

    /// Parses the file header / metadata. Throws std::runtime_error with a
    /// user-readable message on failure. Called once, before run().
    virtual void open() = 0;

    /// Streams the whole file, invoking @p sink per batch, then @p done
    /// exactly once. Runs on the caller's thread; must observe request_stop()
    /// promptly. Must not throw past its first sink call — errors are
    /// reported through @p done.
    virtual void run(EventSink sink, DoneFn done) = 0;

    /// Cooperative cancellation (GUI thread); run() returns soon after.
    virtual void request_stop() { stop_.store(true, std::memory_order_relaxed); }

    const ExternalFileMeta& meta() const { return meta_; }

protected:
    ExternalFileMeta meta_;
    std::string path_;
    std::atomic<bool> stop_{false};
};

/// Factory: returns a source for @p path when the extension is a supported
/// external format (.aedat4 / .alpdata), nullptr otherwise. Throws nothing —
/// header errors surface from open().
std::unique_ptr<ExternalFileSource> try_open_external_file(const std::string& path);

/// True when @p path has an external-format extension (used for user hints).
bool is_external_file_extension(const std::string& path);

} // namespace gui

#endif // GUI_APP_EXTERNAL_FILE_SOURCE_H
