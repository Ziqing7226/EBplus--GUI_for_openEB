// gui/app/alpdata_file_source.h — playback source for Alpsentek .alpdata
// recordings (APX014BA modules; an HDF5 container of per-frame V5.0 records).
//
// ONLY the EVS event frames are decoded — APS image frames are skipped,
// matching the GUI's pure event stream scope. Layout verified byte-level
// against AlpsentekVision V1.9.3 output:
//   HDF5 superblock v0 → root group → … → per-frame datasets (contiguous
//   storage, u8 payload). Each dataset = one V5.0 record:
//     magic u64 0xEEF2F2F2F2F2F2F2 | header (ImageType, DataType bit15=EVS,
//     timestamp µs u64, FrameID, FPGADataSize, ExtendDataSize, rows u16,
//     cols u16, …) | ExtendData | FPGA tiles | magic 0xEEF3F3F3F3F3F3F3.
//   FPGA tiles = (rows/16) × [18B tile header + 16 rows × cols/4 bytes of a
//   2-bit-per-pixel polarity plane (LSB first; up=ON, down=OFF, 0=no event)
//   + 14B tile trailer]. A frame's pixels all share the frame timestamp —
//   the file does not carry per-event times.
// Frame timestamps are a device tick counter that wraps at 2^32 (long
// recordings); wraps are detected and unfolded, then the stream is
// normalized to start at 0.

#ifndef GUI_APP_ALPDATA_FILE_SOURCE_H
#define GUI_APP_ALPDATA_FILE_SOURCE_H

#include "external_file_source.h"

#include <cstdint>
#include <string>
#include <vector>

namespace gui {

class AlpdataFileSource : public ExternalFileSource {
public:
    explicit AlpdataFileSource(const std::string& path) { path_ = path; }

    void open() override;
    void run(EventSink sink, DoneFn done) override;

private:
    struct FrameInfo {
        std::uint64_t addr{0};
        std::uint64_t size{0};
        std::int64_t t_us{0}; // unwrapped, normalized to first frame
    };

    std::vector<FrameInfo> frames_;
    std::int64_t up_value_{2};
    std::int64_t down_value_{1};
    std::int64_t zero_value_{0};
};

} // namespace gui

#endif // GUI_APP_ALPDATA_FILE_SOURCE_H
