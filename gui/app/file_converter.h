// gui/app/file_converter.h — offline file conversion tools (design §1.5.7).
//
// Converts a source event file to:
//   • HDF5   (Metavision::HDF5EventFileWriter)
//   • CSV    (plain "t,x,y,p" text — one event per line)
//   • RAW cut(time-range sub-clip via Metavision::RAWEvt2EventFileWriter)
//
// Also reports file metadata (duration, geometry, encoding, integrator).
// All operations run on a background thread; progress is reported via signals.

#ifndef GUI_APP_FILE_CONVERTER_H
#define GUI_APP_FILE_CONVERTER_H

#include <QObject>
#include <QString>
#include <QJsonObject>
#include <atomic>
#include <thread>

#include <metavision/sdk/base/utils/timestamp.h>

namespace gui {

struct FileInfo {
    QString path;
    int width{0};
    int height{0};
    Metavision::timestamp duration_us{0};
    QString integrator;
    QString serial;
    QString plugin;
    QString encoding;
    qint64 event_count{-1};
};

class FileConverter : public QObject {
    Q_OBJECT
public:
    explicit FileConverter(QObject* parent = nullptr);
    ~FileConverter();

    enum class Format { HDF5, CSV };
    void convert(const QString& src, const QString& dst, Format fmt);
    /// @brief Cuts @p src to [@p start_us, @p end_us] writing RAW to @p dst.
    void cut(const QString& src, const QString& dst,
             Metavision::timestamp start_us, Metavision::timestamp end_us);
    /// @brief Queries metadata for @p src (synchronous, fast).
    FileInfo info(const QString& src) const;

    void cancel();

signals:
    void progress(double ratio);
    void completed(const QString& output);
    void failed(const QString& message);

private:
    void run_convert(const QString& src, const QString& dst, Format fmt);
    void run_cut(const QString& src, const QString& dst,
                 Metavision::timestamp start_us, Metavision::timestamp end_us);

    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_{false};
    std::thread worker_;
};

} // namespace gui

#endif // GUI_APP_FILE_CONVERTER_H
