// gui/app/aedat4_file_source.h — playback source for inivation DV .aedat4
// recordings (e.g. DAVIS346). Reads ONLY the EVTS (polarity event) streams —
// frame/IMU/trigger packets are skipped, matching the GUI's pure event stream
// scope. Layout verified byte-level against DV v2.0 recorder output:
//   "#!AER-DAT4.0\r\n" + [u32 ioHeaderSize][IOHeader flatbuffer]
//   packets: {i32 streamID; i32 size} + body [u32 fbSize][flatbuffer]
//   EVTS event = {i64 t (µs); i16 x; i16 y; u8 polarity; 3 pad} × count
// Packets may be LZ4-framed (IOHeader.compression 1/2); zstd is rejected.

#ifndef GUI_APP_AEDAT4_FILE_SOURCE_H
#define GUI_APP_AEDAT4_FILE_SOURCE_H

#include "external_file_source.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace gui {

class Aedat4FileSource : public ExternalFileSource {
public:
    explicit Aedat4FileSource(const std::string& path) { path_ = path; }

    void open() override;
    void run(EventSink sink, DoneFn done) override;

private:
    struct PacketInfo {
        std::int64_t byte_offset{0}; // packet body (after the 8-byte header)
        std::int32_t size{0};
        std::int64_t num_elements{0};
        std::int64_t ts_start{0};
        std::int64_t ts_end{0};
    };

    /// Parses the FileDataTable (optional) for duration / event-count metadata.
    void parse_data_table(std::ifstream& file, std::streamoff table_pos);

    std::streamoff first_packet_offset_{0};
    std::streamoff file_size_{0};
    /// End of the packet stream (start of the FileDataTable when known).
    std::streamoff stream_end_{-1};
    int compression_{0}; // IOHeader.compression (0 none, 1/2 lz4, 3/4 zstd)
    /// streamID → true when that stream carries EVTS packets.
    std::map<std::int32_t, bool> stream_is_events_;
};

} // namespace gui

#endif // GUI_APP_AEDAT4_FILE_SOURCE_H
