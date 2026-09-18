// gui/tests/test_aedat4_writer.cpp — Phase 4 round-trip: Aedat4Writer output
// must be readable by our AEDAT4 file source with event-exact fidelity (the
// reader was validated byte-level against real DV files, so this transitively
// validates the writer against the DV-native format).

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "app/aedat4_file_source.h"
#include "recorder/aedat4_writer.h"

namespace {

std::vector<Metavision::EventCD> read_all(const std::string& path,
                                          gui::ExternalFileMeta& meta) {
    gui::Aedat4FileSource source(path);
    source.open();
    meta = source.meta();
    std::vector<Metavision::EventCD> out;
    std::string error;
    source.run([&out](const Metavision::EventCD* b, const Metavision::EventCD* e) {
                  out.insert(out.end(), b, e);
              },
              [&error](const std::string& err) { error = err; });
    if (!error.empty()) ADD_FAILURE() << "reader error: " << error;
    return out;
}

} // namespace

TEST(Aedat4Writer, RoundTripEventExact) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_rt.aedat4").string();

    // Three batches (forces several packets): timestamps start at 0 so the
    // reader's t0 normalization is the identity; both polarities; in-bounds
    // coordinates only (the reader drops out-of-range events).
    std::vector<std::vector<Metavision::EventCD>> batches(3);
    std::int64_t t = 0;
    for (int b = 0; b < 3; ++b) {
        for (int i = 0; i < 977; ++i) {  // 977: prime, non-multiple of 16
            Metavision::EventCD ev;
            ev.t = t;
            ev.x = static_cast<std::uint16_t>((i * 7) % 346);
            ev.y = static_cast<std::uint16_t>((i * 13) % 260);
            ev.p = (i + b) % 2;
            batches[static_cast<std::size_t>(b)].push_back(ev);
            t += 17;
        }
    }

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 346, 260, "TEST-0001"));
    for (const auto& batch : batches) writer.write(batch.data(), batch.data() + batch.size());
    writer.close();
    ASSERT_EQ(writer.events_written(), 3u * 977);

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    ASSERT_EQ(read_back.size(), batches[0].size() + batches[1].size() + batches[2].size());
    EXPECT_EQ(meta.width, 346);
    EXPECT_EQ(meta.height, 260);
    EXPECT_EQ(meta.integrator, QStringLiteral("inivation"));
    EXPECT_EQ(meta.encoding_format, QStringLiteral("EVTS"));
    EXPECT_GT(meta.duration_us, 0);

    std::size_t k = 0;
    for (const auto& batch : batches) {
        for (const auto& ev : batch) {
            SCOPED_TRACE(k);
            EXPECT_EQ(read_back[k].t, ev.t);
            EXPECT_EQ(read_back[k].x, ev.x);
            EXPECT_EQ(read_back[k].y, ev.y);
            EXPECT_EQ(read_back[k].p, ev.p);
            ++k;
        }
    }
    std::filesystem::remove(path);
}

TEST(Aedat4Writer, LargeStreamSpawnsMultiplePackets) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_multi.aedat4").string();

    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 640, 480, "TEST-MULTI"));
    // > the 2048-event flush threshold → at least two packets, each recorded
    // in the data table.
    std::vector<Metavision::EventCD> big;
    for (int i = 0; i < 5000; ++i) {
        Metavision::EventCD ev;
        ev.t = 10 * i;
        ev.x = static_cast<std::uint16_t>(i % 640);
        ev.y = static_cast<std::uint16_t>((i * 3) % 480);
        ev.p = i % 2;
        big.push_back(ev);
    }
    writer.write(big.data(), big.data() + big.size());
    writer.close();

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    ASSERT_EQ(read_back.size(), big.size());
    for (std::size_t i = 0; i < big.size(); ++i) {
        EXPECT_EQ(read_back[i].t, big[i].t);
        EXPECT_EQ(read_back[i].x, big[i].x);
        EXPECT_EQ(read_back[i].y, big[i].y);
        EXPECT_EQ(read_back[i].p, big[i].p);
    }
    EXPECT_EQ(meta.duration_us, 10 * 4999);
    std::filesystem::remove(path);
}

TEST(Aedat4Writer, WriteAfterCloseIsIgnored) {
    const std::string path =
        (std::filesystem::temp_directory_path() / "ebplus_aedat4_closed.aedat4").string();
    gui::Aedat4Writer writer;
    ASSERT_TRUE(writer.open(path, 10, 10, "X"));
    writer.close();
    ASSERT_FALSE(writer.is_open());

    std::vector<Metavision::EventCD> evs(10, Metavision::EventCD{1, 1, 1, 1});
    writer.write(evs.data(), evs.data() + evs.size());  // no-op, no crash

    gui::ExternalFileMeta meta;
    const auto read_back = read_all(path, meta);
    EXPECT_TRUE(read_back.empty());  // nothing was recorded
    std::filesystem::remove(path);
}
