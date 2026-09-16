// gui/app/lz4_frame_decoder.h — self-contained LZ4 frame decompressor (decode
// only). DV (inivation) AEDAT4 files wrap each packet in an LZ4 frame when the
// IOHeader compression field is LZ4/LZ4_HIGH; zstd is not supported. A decoder
// is built in instead of linking liblz4 so the GUI keeps zero extra external
// dependencies.

#ifndef GUI_APP_LZ4_FRAME_DECODER_H
#define GUI_APP_LZ4_FRAME_DECODER_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace gui {

/// @brief Decompresses one (or more, concatenated) LZ4 frame(s).
/// @param data input bytes (complete frames; trailing garbage is an error).
/// @param out receives the decompressed bytes (appended).
/// @returns true on success; on failure @p error carries a short reason.
bool lz4_decompress_frame(const std::uint8_t* data, std::size_t size,
                          std::vector<std::uint8_t>& out, std::string& error);

} // namespace gui

#endif // GUI_APP_LZ4_FRAME_DECODER_H
