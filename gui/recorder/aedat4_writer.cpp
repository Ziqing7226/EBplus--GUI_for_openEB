// gui/recorder/aedat4_writer.cpp — see aedat4_writer.h.
//
// Flatbuffers are built forward with explicit offsets (equivalent to what
// our reader walks in aedat4_file_source.cpp):
//   buffer = [u32 rootOffset][ident "XXXX"][payload...]
//   table  = [i32 vtableOffset][fields...]; the offset is SIGNED — a vtable
//   placed after its table yields a negative value (dv's FTAB does this).
//   vector = [u32 count][elements]; string = [u32 length][bytes][0].
// EventPacket: one field, VT4 = vector of 16-byte structs
//   {i64 t; i16 x; i16 y; u8 polarity; 3 pad}. IOHeader: VT4 = i32
//   compression (0 = NONE), VT6 = i64 dataTablePosition, VT8 = string
//   sourceInfo. DataTable ("FTAB"): VT4 = vector of entry tables, each
//   {VT6 = struct{i32 streamID; i32 size}, VT8 = i64 numEvents,
//   VT10 = i64 tsStart, VT12 = i64 tsEnd}.

#include "aedat4_writer.h"

#include <algorithm>
#include <cstring>

namespace gui {
namespace {

constexpr char kMagic[14] = {'#', '!', 'A', 'E', 'R', '-', 'D', 'A', 'T', '4',
                             '.', '0', '\r', '\n'};
constexpr std::size_t kFlushEvents = 2048;

void put_u16(std::vector<std::uint8_t>& b, std::size_t pos, std::uint16_t v) {
    b[pos] = static_cast<std::uint8_t>(v);
    b[pos + 1] = static_cast<std::uint8_t>(v >> 8);
}
void put_u32(std::vector<std::uint8_t>& b, std::size_t pos, std::uint32_t v) {
    b[pos] = static_cast<std::uint8_t>(v);
    b[pos + 1] = static_cast<std::uint8_t>(v >> 8);
    b[pos + 2] = static_cast<std::uint8_t>(v >> 16);
    b[pos + 3] = static_cast<std::uint8_t>(v >> 24);
}
void put_i32_at(std::vector<std::uint8_t>& b, std::size_t pos, std::int32_t v) {
    put_u32(b, pos, static_cast<std::uint32_t>(v));
}
/// EventPacket — flatbuffers uoffsets point FORWARD (target = pos + rel):
/// [0] u32 root → table@8; [4] "EVTS"; table@8 {soffset −8 → vtable@16;
/// VT4@12 → vector@28}; vtable@16; [28] count; [32] structs (16 B each,
/// 8-aligned — flatbuffers vectors store elements directly at count+4).
void build_event_packet(const std::vector<Metavision::EventCD>& evs,
                        std::vector<std::uint8_t>& out) {
    const std::size_t count = evs.size();
    out.assign(32 + 16 * count, 0);

    put_u32(out, 0, 8);                    // root → table
    std::memcpy(out.data() + 4, "EVTS", 4);
    put_i32_at(out, 8, -8);                // vtable = table − (−8) = 16
    put_u32(out, 12, 16);                  // VT4: field@12 + 16 = vector@28
    put_u16(out, 16, 6);                   // vtable size
    put_u16(out, 18, 8);                   // table size
    put_u16(out, 20, 4);                   // VT4 offset in table
    put_u32(out, 28, static_cast<std::uint32_t>(count));  // vector count
    for (std::size_t i = 0; i < count; ++i) {
        std::size_t pos = 28 + 4 + 16 * i;
        const std::int64_t t = evs[i].t;
        std::memcpy(out.data() + pos, &t, 8);
        const std::int16_t x = static_cast<std::int16_t>(evs[i].x);
        const std::int16_t y = static_cast<std::int16_t>(evs[i].y);
        std::memcpy(out.data() + pos + 8, &x, 2);
        std::memcpy(out.data() + pos + 10, &y, 2);
        out[pos + 12] = evs[i].p ? 1 : 0;
    }
}

/// IOHeader — [0] u32 root → table@12 (table+12 8-aligned); [4] "IOHE";
/// table@12 {soffset −20 → vtable@32; VT4 i32 compression@16; VT8 u32@20 →
/// string after the table; VT6 i64 dataTablePosition@24}; vtable@32; string
/// (u32 len + bytes + 0) after the vtable. Returns the buffer offset of the
/// dataTablePosition i64 (the caller patches it once the region is placed).
std::size_t build_io_header(std::vector<std::uint8_t>& out, std::int64_t table_position,
                            const std::string& xml) {
    const std::size_t slen = xml.size();
    const std::size_t str_len_pos = 44;
    const std::size_t total = str_len_pos + 4 + slen + 1;
    out.assign(total, 0);

    put_u32(out, 0, 12);                   // root → table@12
    std::memcpy(out.data() + 4, "IOHE", 4);
    put_i32_at(out, 12, -20);              // vtable = 12 + 20 = 32
    put_i32_at(out, 16, 0);                // VT4: compression NONE
    put_u32(out, 20, static_cast<std::uint32_t>(str_len_pos - 20));  // VT8 → string
    std::memcpy(out.data() + 24, &table_position, 8);                // VT6
    put_u16(out, 32, 10);                  // vtable size
    put_u16(out, 34, 20);                  // table size
    put_u16(out, 36, 4);                   // VT4 offset
    put_u16(out, 38, 12);                  // VT6 offset
    put_u16(out, 40, 8);                   // VT8 offset
    put_u32(out, str_len_pos, static_cast<std::uint32_t>(slen));
    std::memcpy(out.data() + str_len_pos + 4, xml.data(), slen);
    out[total - 1] = 0;
    return 24;  // dataTablePosition i64 sits at table(12) + 12
}

/// DataTable — [0] u32 root → table@8; [4] "FTAB"; table@8 {soffset −8 →
/// vtable@16; VT4 u32@12 → vector@24}; vtable@16; [24] count; [28] slots
/// (u32 forward displacements); entry tables (E ≡ 4 mod 8, vtable after
/// each entry): [i32 soffset][i32 streamID][i32 size][i32 pad][i64 num]
/// [i64 ts0][i64 ts1].
void build_data_table(std::vector<std::uint8_t>& out,
                      const std::vector<Aedat4Writer::Entry>& entries) {
    const std::size_t slots = 28 + 4 * entries.size();
    out.reserve(slots + entries.size() * 48 + 16);
    out.assign(slots, 0);

    put_u32(out, 0, 8);    // root → table
    std::memcpy(out.data() + 4, "FTAB", 4);
    put_i32_at(out, 8, -8);  // vtable = 8 + 8 = 16
    put_u32(out, 12, 12);    // VT4: field@12 + 12 = vector@24
    put_u16(out, 16, 6);
    put_u16(out, 18, 8);
    put_u16(out, 20, 4);
    put_u32(out, 24, static_cast<std::uint32_t>(entries.size()));

    for (std::size_t i = 0; i < entries.size(); ++i) {
        while ((out.size() + 12) % 8 != 0) out.push_back(0);
        const std::size_t slot = 28 + 4 * i;
        const std::size_t entry = out.size();
        const std::size_t vtable = entry + 36;
        out.resize(vtable + 14);
        put_u32(out, slot, static_cast<std::uint32_t>(entry - slot));
        put_i32_at(out, entry, static_cast<std::int32_t>(entry - vtable));
        put_i32_at(out, entry + 4, 0);  // VT6 struct: streamID
        put_i32_at(out, entry + 8, entries[i].size);
        std::memcpy(out.data() + entry + 12, &entries[i].num, 8);   // VT8
        std::memcpy(out.data() + entry + 20, &entries[i].ts0, 8);   // VT10
        std::memcpy(out.data() + entry + 28, &entries[i].ts1, 8);   // VT12
        put_u16(out, vtable, 14);
        put_u16(out, vtable + 2, 36);
        put_u16(out, vtable + 4, 0);
        put_u16(out, vtable + 6, 4);
        put_u16(out, vtable + 8, 12);
        put_u16(out, vtable + 10, 20);
        put_u16(out, vtable + 12, 28);
    }
}

} // namespace

Aedat4Writer::~Aedat4Writer() {
    close();
}

bool Aedat4Writer::open(const std::string& path, int width, int height,
                        const std::string& source) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (file_) return false;
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;

    // Stream-info XML — the DV recorder's outInfo shape (our reader's
    // parse_out_info consumes typeIdentifier/source/sizeX/sizeY).
    std::string xml = "<dv version=\"2.0\">\n";
    xml += "    <node name=\"outInfo\" path=\"/mainloop/Recorder/outInfo/\">\n";
    xml += "        <node name=\"0\" path=\"/mainloop/Recorder/outInfo/0/\">\n";
    xml += "            <attr key=\"compression\" type=\"string\">NONE</attr>\n";
    xml += "            <attr key=\"originalModuleName\" type=\"string\">capture</attr>\n";
    xml += "            <attr key=\"originalOutputName\" type=\"string\">events</attr>\n";
    xml += "            <attr key=\"typeDescription\" type=\"string\">Array of events (polarity ON/OFF).</attr>\n";
    xml += "            <attr key=\"typeIdentifier\" type=\"string\">EVTS</attr>\n";
    xml += "            <node name=\"info\" path=\"/mainloop/Recorder/outInfo/0/info/\">\n";
    xml += "                <attr key=\"eventsPixelArrangement\" type=\"int\">0</attr>\n";
    xml += "                <attr key=\"sizeX\" type=\"int\">" + std::to_string(width) + "</attr>\n";
    xml += "                <attr key=\"sizeY\" type=\"int\">" + std::to_string(height) + "</attr>\n";
    xml += "                <attr key=\"source\" type=\"string\">" + source + "</attr>\n";
    xml += "            </node>\n";
    xml += "        </node>\n";
    xml += "    </node>\n";
    xml += "</dv>\n";

    std::vector<std::uint8_t> header;
    // dataTablePosition placeholder — patched at close() once the region
    // offset is known.
    const std::size_t field_in_io = build_io_header(header, 0, xml);
    table_pos_field_ = 14 + 4 + static_cast<std::streamoff>(field_in_io);

    bool ok = std::fwrite(kMagic, 1, sizeof(kMagic), f) == sizeof(kMagic);
    const std::uint32_t io_size = static_cast<std::uint32_t>(header.size());
    ok = ok && std::fwrite(&io_size, 1, 4, f) == 4;
    ok = ok && std::fwrite(header.data(), 1, header.size(), f) == header.size();
    if (!ok) {
        std::fclose(f);
        return false;
    }
    file_ = f;
    total_events_ = 0;
    pending_.clear();
    entries_.clear();
    return true;
}

void Aedat4Writer::write(const Metavision::EventCD* begin,
                         const Metavision::EventCD* end) {
    if (begin == nullptr || end == nullptr || begin >= end) return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;
    // Chunk at the flush threshold so a huge incoming batch becomes several
    // bounded packets (DV commits packets frequently for seekability).
    while (begin < end) {
        const std::size_t room = kFlushEvents - pending_.size();
        const std::size_t take = std::min<std::size_t>(
            static_cast<std::size_t>(end - begin), room);
        pending_.insert(pending_.end(), begin, begin + take);
        begin += take;
        if (pending_.size() >= kFlushEvents) flush_locked();
    }
}

void Aedat4Writer::flush_locked() {
    if (pending_.empty() || !file_) return;

    std::vector<std::uint8_t> packet;
    build_event_packet(pending_, packet);
    // Body = [u32 fbSize][flatbuffer]; packet header = {i32 streamID}{i32
    // bodySize} — the real recorder files carry size = fbSize + 4.
    const auto fb_size = static_cast<std::int32_t>(packet.size());
    const auto body_size = static_cast<std::int32_t>(packet.size() + 4);
    const std::int32_t header[3] = {0, body_size, fb_size};
    if (std::fwrite(header, 4, 3, file_) != 3 ||
        std::fwrite(packet.data(), 1, packet.size(), file_) != packet.size()) {
        return;  // write failure: keep pending_ (a later flush retries)
    }
    std::int64_t ts0 = pending_.front().t;
    std::int64_t ts1 = pending_.back().t;
    if (ts1 < ts0) std::swap(ts0, ts1);
    entries_.push_back({body_size, static_cast<std::int64_t>(pending_.size()), ts0, ts1});
    total_events_ += pending_.size();
    pending_.clear();
}

void Aedat4Writer::close() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!file_) return;
    flush_locked();

    const std::streamoff region_start = std::ftell(file_);
    std::vector<std::uint8_t> table;
    build_data_table(table, entries_);
    const auto region = static_cast<std::uint32_t>(table.size());
    const bool ok = std::fwrite(&region, 1, 4, file_) == 4 &&
                    std::fwrite(table.data(), 1, table.size(), file_) == table.size();
    if (ok && std::fseek(file_, table_pos_field_, SEEK_SET) == 0) {
        std::fwrite(&region_start, 8, 1, file_);
    }
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
    pending_.clear();
    entries_.clear();
}

} // namespace gui
