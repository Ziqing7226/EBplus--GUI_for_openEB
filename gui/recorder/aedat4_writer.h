// gui/recorder/aedat4_writer.h — AEDAT4 (DV-native) event recording for the
// inivation live sources (Phase 4). Produces the byte-level layout of the
// DV v2.0 recorder (verified against real files and our AEDAT4 reader):
//   "#!AER-DAT4.0\r\n" + [u32 ioHeaderSize][IOHeader flatbuffer]
//   packets: {i32 streamID = 0; i32 size} + body [u32 fbSize]["EVTS" fb]
//   trailing FileDataTable ("FTAB") + IOHeader.dataTablePosition back-patch
// Packets are uncompressed (compression NONE); events accumulate and flush
// as one EventPacket per threshold to keep packet overhead low.

#ifndef GUI_RECORDER_AEDAT4_WRITER_H
#define GUI_RECORDER_AEDAT4_WRITER_H

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <metavision/sdk/base/events/event_cd.h>

namespace gui {

class Aedat4Writer {
public:
    /// FileDataTable record per written packet.
    struct Entry {
        std::int32_t size;
        std::int64_t num;
        std::int64_t ts0;
        std::int64_t ts1;
    };

    Aedat4Writer() = default;
    ~Aedat4Writer();

    Aedat4Writer(const Aedat4Writer&) = delete;
    Aedat4Writer& operator=(const Aedat4Writer&) = delete;

    /// @brief Opens @p path and writes the header. @p source names the
    ///        camera (recorded in the stream-info XML).
    bool open(const std::string& path, int width, int height,
              const std::string& source);
    /// @brief Appends a batch (called from the USB thread). Accumulates and
    ///        flushes a packet once the threshold is reached.
    void write(const Metavision::EventCD* begin, const Metavision::EventCD* end);
    /// @brief Flushes the pending packet and writes the data table. Safe to
    ///        call twice.
    void close();

    [[nodiscard]] bool is_open() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return file_ != nullptr;
    }
    [[nodiscard]] std::uint64_t events_written() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return total_events_;
    }

private:
    void flush_locked();

    std::FILE* file_{nullptr};
    mutable std::mutex mtx_;
    std::vector<Metavision::EventCD> pending_;
    std::vector<Entry> entries_;
    std::streamoff table_pos_field_{0};  ///< IOHeader dataTablePosition slot.
    std::uint64_t total_events_{0};
};

} // namespace gui

#endif // GUI_RECORDER_AEDAT4_WRITER_H
