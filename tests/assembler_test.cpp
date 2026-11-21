#include "rt/frame/frame_assembler.hpp"
#include "rt/types.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

// All times are written as offsets from the first scan (t0). W = 75'000 us, so
// frame k covers [k * 75'000, (k + 1) * 75'000) and closes on an odo >= end - 1.

namespace
{

constexpr std::int64_t kT0 = 1'000'000; // nonzero so a missing "- t0" shows up

class FrameAssemblerTest : public ::testing::Test
{
  protected:
    rt::FrameAssembler fa;

    // One scan holding a single measurement at t0 + offset.
    void scan(std::int64_t offset_us, std::uint8_t sensor_id = 1)
    {
        const std::int64_t t = kT0 + offset_us;
        fa.push(rt::Scan{
            .sensor_id = sensor_id,
            .t_us = t,
            .measurements = {rt::Measurement{.t_us = t, .sensor_id = sensor_id}},
        });
    }

    void odo(std::int64_t offset_us)
    {
        fa.push(rt::OdometrySample{.t_us = kT0 + offset_us});
    }

    // Pop until empty.
    std::vector<rt::Frame> drain()
    {
        std::vector<rt::Frame> frames;
        while (auto f = fa.pop_ready())
        {
            frames.push_back(std::move(*f));
        }
        return frames;
    }
};

} // namespace

TEST_F(FrameAssemblerTest, ReleasesOnlyOnceOdoReachesWindowEndMinusOne)
{
    EXPECT_FALSE(fa.pop_ready().has_value());

    odo(-10);
    scan(0);
    odo(74'998);
    EXPECT_TRUE(drain().empty());

    odo(74'999);
    const auto frames = drain();
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].frame_idx, 0);
    EXPECT_FALSE(fa.pop_ready().has_value());
}

TEST_F(FrameAssemblerTest, OdoBeforeFirstScanOnlyBuffers)
{
    odo(-200'000);
    odo(-100'000);
    EXPECT_TRUE(drain().empty());
}

TEST_F(FrameAssemblerTest, FirstFrameStartsAtLatestOdoBeforeT0)
{
    // a backlog of odo from before the first scan; only the last one is frame 0's front bracket
    odo(-300'000);
    odo(-200'000);
    odo(-100'000);
    odo(-5'000);
    scan(0);
    odo(80'000); // closes frame 0

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 1U);
    ASSERT_EQ(frames[0].ego_states.size(), 2U); // -5'000 and 80'000
    EXPECT_DOUBLE_EQ(frames[0].ego_states.front().t_s, -0.005);
}

TEST_F(FrameAssemblerTest, BuildsFrameContentsInSecondsSinceT0)
{
    odo(-5'000);
    scan(0, 1);
    odo(30'000);
    scan(37'512, 2);
    scan(80'000); // belongs to frame 1
    odo(80'000);  // closes frame 0

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 1U);
    const auto& f = frames[0];

    EXPECT_EQ(f.frame_idx, 0);
    EXPECT_DOUBLE_EQ(f.frame_time, 0.0375);

    ASSERT_EQ(f.detections.size(), 2U);
    EXPECT_DOUBLE_EQ(f.detections[0].t_s, 0.0);
    EXPECT_DOUBLE_EQ(f.detections[1].t_s, 0.037512);
    EXPECT_EQ(f.detections[1].sensor_id, 2);

    // front bracket (before t0, so negative) + inside + back bracket
    ASSERT_EQ(f.ego_states.size(), 3U);
    EXPECT_DOUBLE_EQ(f.ego_states[0].t_s, -0.005);
    EXPECT_DOUBLE_EQ(f.ego_states[1].t_s, 0.030);
    EXPECT_DOUBLE_EQ(f.ego_states[2].t_s, 0.080);
}

TEST_F(FrameAssemblerTest, OneOdoAfterAGapReleasesSeveralFramesInOrder)
{
    odo(-1);
    scan(0);
    scan(100'000);
    // window 2 [150'000, 225'000) gets no scans
    odo(230'000);

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 3U);
    for (std::size_t i = 0; i < frames.size(); ++i)
    {
        SCOPED_TRACE(i);
        EXPECT_EQ(frames[i].frame_idx, static_cast<std::int64_t>(i));
        // no odo inside any window: just the two brackets
        ASSERT_EQ(frames[i].ego_states.size(), 2U);
        EXPECT_DOUBLE_EQ(frames[i].ego_states[0].t_s, -0.000001);
        EXPECT_DOUBLE_EQ(frames[i].ego_states[1].t_s, 0.230);
    }
    EXPECT_EQ(frames[0].detections.size(), 1U);
    EXPECT_EQ(frames[1].detections.size(), 1U);
    EXPECT_TRUE(frames[2].detections.empty()); // empty windows are still released
    EXPECT_DOUBLE_EQ(frames[1].frame_time, 0.1125);
}

TEST_F(FrameAssemblerTest, DiscardsWindowsThatStartBeforeOdometry)
{
    scan(0);
    odo(74'999); // closes frame 0, which has no front bracket; is frame 1's front bracket
    scan(100'000);
    odo(150'000);

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].frame_idx, 1);
    ASSERT_EQ(frames[0].detections.size(), 1U);
    EXPECT_DOUBLE_EQ(frames[0].detections[0].t_s, 0.1);
}

TEST_F(FrameAssemblerTest, ScanAfterAnOdoInsideItsWindowIsDiscardedWithThatWindow)
{
    scan(0);
    odo(30'000); // inside window 0, does not close it
    scan(50'000);
    odo(80'000); // closes window 0 -> discarded (no front bracket)
    odo(160'000);

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].frame_idx, 1);
    EXPECT_TRUE(frames[0].detections.empty()); // neither window-0 scan leaked in
}

TEST_F(FrameAssemblerTest, TrimKeepsSampleExactlyAtNextWindowStart)
{
    odo(-10);
    scan(0);
    odo(40'000);
    odo(75'000); // closes frame 0 and sits exactly on frame 1's start
    scan(100'000);
    odo(150'000);

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_DOUBLE_EQ(frames[1].ego_states.front().t_s, 0.075);
}

TEST_F(FrameAssemblerTest, TrimKeepsLatestSampleBeforeNextWindowStart)
{
    odo(-10);
    scan(0);
    odo(70'000);
    odo(80'000); // closes frame 0
    scan(100'000);
    odo(150'000);

    const auto frames = drain();
    ASSERT_EQ(frames.size(), 2U);
    ASSERT_EQ(frames[1].ego_states.size(), 3U); // 70'000, 80'000, 150'000
    EXPECT_DOUBLE_EQ(frames[1].ego_states.front().t_s, 0.070);
}
