#include "rt/io/loader.hpp"
#include "rt/types.hpp"

#include <gtest/gtest.h>
#include <highfive/H5DataType.hpp>
#include <highfive/H5File.hpp>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Fixture-file helpers
//
// These structs mirror the on-disk RadarScenes layout (float32 fields), not the
// pipeline types, so the loader is exercised through the same file-to-memory
// conversion it does on real data.
// ---------------------------------------------------------------------------

namespace
{

struct OdoRow
{
    std::int64_t timestamp{};
    float x_seq{};
    float y_seq{};
    float yaw_seq{};
    float vx{};
    float yaw_rate{};
};

struct RadarRow
{
    std::int64_t timestamp{};
    std::uint8_t sensor_id{};
    float range_sc{};
    float azimuth_sc{};
    float rcs{};
    float vr{};
};

HighFive::CompoundType make_odo_row()
{
    using HighFive::create_datatype;
    return {{
                {"timestamp", create_datatype<std::int64_t>(), offsetof(OdoRow, timestamp)},
                {"x_seq", create_datatype<float>(), offsetof(OdoRow, x_seq)},
                {"y_seq", create_datatype<float>(), offsetof(OdoRow, y_seq)},
                {"yaw_seq", create_datatype<float>(), offsetof(OdoRow, yaw_seq)},
                {"vx", create_datatype<float>(), offsetof(OdoRow, vx)},
                {"yaw_rate", create_datatype<float>(), offsetof(OdoRow, yaw_rate)},
            },
            sizeof(OdoRow)};
}

HighFive::CompoundType make_radar_row()
{
    using HighFive::create_datatype;
    return {{
                {"timestamp", create_datatype<std::int64_t>(), offsetof(RadarRow, timestamp)},
                {"sensor_id", create_datatype<std::uint8_t>(), offsetof(RadarRow, sensor_id)},
                {"range_sc", create_datatype<float>(), offsetof(RadarRow, range_sc)},
                {"azimuth_sc", create_datatype<float>(), offsetof(RadarRow, azimuth_sc)},
                {"rcs", create_datatype<float>(), offsetof(RadarRow, rcs)},
                {"vr", create_datatype<float>(), offsetof(RadarRow, vr)},
            },
            sizeof(RadarRow)};
}

} // namespace

HIGHFIVE_REGISTER_TYPE(OdoRow, make_odo_row)
HIGHFIVE_REGISTER_TYPE(RadarRow, make_radar_row)

namespace
{

// Gives each test its own empty scratch directory, removed afterwards.
class LoaderTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               (std::string("rt_tests_") + info->test_suite_name() + "_" + info->name());
        fs::remove_all(dir_);
        fs::create_directories(dir_);
    }

    void TearDown() override
    {
        fs::remove_all(dir_);
    }

    // Writes `contents` to a file in the scratch directory and returns its path.
    [[nodiscard]]
    std::string write_text(const std::string& name, const std::string& contents) const
    {
        const fs::path path = dir_ / name;
        std::ofstream(path) << contents;
        return path.string();
    }

    // Writes a radar_data.h5 with the given rows and returns its path.
    [[nodiscard]]
    std::string write_h5(const std::vector<OdoRow>& odo, const std::vector<RadarRow>& radar) const
    {
        const fs::path path = dir_ / "radar_data.h5";
        HighFive::File f(path.string(), HighFive::File::Truncate);
        f.createDataSet("odometry", odo);
        f.createDataSet("radar_data", radar);
        return path.string();
    }

    fs::path dir_;
};

// Returns the path to a real sequence's radar_data.h5, or empty if RT_DATA_DIR is
// unset or the file is missing.
std::string real_sequence(const std::string& name)
{
    const char* root = std::getenv("RT_DATA_DIR");
    if (root == nullptr)
    {
        return {};
    }

    const fs::path path = fs::path(root) / name / "radar_data.h5";
    return fs::exists(path) ? path.string() : std::string{};
}

// sensors.json with one entry per id, using the given ids verbatim.
std::string sensors_json(const std::vector<int>& ids)
{
    std::string json = "{";
    for (std::size_t i = 0; i < ids.size(); ++i)
    {
        const auto id = std::to_string(ids[i]);
        json += (i == 0 ? "" : ",");
        json += "\"radar_" + std::to_string(i) + "\": {\"id\": " + id +
                ", \"x\": 1.0, \"y\": 2.0, \"yaw\": 0.5}";
    }
    return json + "}";
}

struct Drained
{
    std::size_t scans{};
    std::size_t measurements{};
    std::size_t odometry{};
};

// Pulls every message out of the source, checking the stream invariants from
// docs/conventions.md ("Replay model") along the way.
Drained drain(rt::RadarScenesSource& source)
{
    Drained out;
    std::int64_t prev_t_us = INT64_MIN;

    while (auto msg = source.next())
    {
        std::int64_t t_us = 0;

        if (const auto* scan = std::get_if<rt::Scan>(&*msg))
        {
            t_us = scan->t_us;
            EXPECT_FALSE(scan->measurements.empty()) << "empty scan at t_us=" << t_us;

            for (const auto& m : scan->measurements)
            {
                EXPECT_EQ(m.t_us, scan->t_us);
                EXPECT_EQ(m.sensor_id, scan->sensor_id);
            }

            ++out.scans;
            out.measurements += scan->measurements.size();
        }
        else
        {
            t_us = std::get<rt::OdometrySample>(*msg).t_us;
            ++out.odometry;
        }

        EXPECT_GE(t_us, prev_t_us) << "timestamp went backwards";
        prev_t_us = t_us;
    }

    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Group 1: load_sensor_data
// ---------------------------------------------------------------------------

// Each sensor lands at index id - 1, regardless of the order entries appear in.
TEST_F(LoaderTest, SensorLoadPlacesSensorsByIdMinusOne)
{
    // keys deliberately out of order relative to id
    const auto path = write_text("sensors.json", R"({
        "radar_c": {"id": 3, "x": 3.86,  "y":  0.7,   "yaw":  0.436},
        "radar_a": {"id": 1, "x": 3.663, "y": -0.873, "yaw": -1.484},
        "radar_d": {"id": 4, "x": 3.663, "y":  0.873, "yaw":  1.484},
        "radar_b": {"id": 2, "x": 3.86,  "y": -0.7,   "yaw": -0.436}
    })");

    const auto sensors = rt::load_sensor_data(path);

    // EXPECT_* records a failure and keeps going; ASSERT_* stops the test.
    // EXPECT_NEAR for doubles: exact == on floating point is fragile.
    EXPECT_NEAR(sensors[0].x, 3.663, 1e-9);
    EXPECT_NEAR(sensors[0].y, -0.873, 1e-9);
    EXPECT_NEAR(sensors[0].yaw, -1.484, 1e-9);

    EXPECT_NEAR(sensors[2].yaw, 0.436, 1e-9);
    EXPECT_NEAR(sensors[3].y, 0.873, 1e-9);
}

TEST_F(LoaderTest, SensorLoadThrowsOnMissingFile)
{
    EXPECT_THROW(static_cast<void>(rt::load_sensor_data((dir_ / "nope.json").string())),
                 std::runtime_error);
}

TEST_F(LoaderTest, SensorLoadRejectsBadIdSets)
{
    // SCOPED_TRACE labels any failure with the case that caused it
    const std::vector<std::pair<std::string, std::vector<int>>> cases = {
        {"id 0", {0, 1, 2, 3}},         {"id above kMaxSensors", {1, 2, 3, 5}},
        {"negative id", {-1, 1, 2, 3}}, {"duplicate id", {1, 2, 2, 3}},
        {"missing id", {1, 2, 3}},
    };

    for (const auto& [label, ids] : cases)
    {
        SCOPED_TRACE(label);
        const auto path = write_text("sensors.json", sensors_json(ids));
        EXPECT_THROW(static_cast<void>(rt::load_sensor_data(path)), std::runtime_error);
    }
}

// The real file matches the mounting table in docs/conventions.md.
TEST(SensorLoadRealData, MatchesConventionsTable)
{
    const char* root = std::getenv("RT_DATA_DIR");
    if (root == nullptr)
    {
        GTEST_SKIP() << "RT_DATA_DIR not set";
    }

    const auto sensors = rt::load_sensor_data((fs::path(root) / "sensors.json").string());

    // the doc rounds to 3 decimals
    constexpr double kTol = 1e-3;
    const std::array<rt::SensorMount, rt::kMaxSensors> expected = {
        {
            {.x = 3.663, .y = -0.873, .yaw = -1.484},
            {.x = 3.860, .y = -0.700, .yaw = -0.436},
            {.x = 3.860, .y = 0.700, .yaw = 0.436},
            {.x = 3.663, .y = 0.873, .yaw = 1.484},
        },
    };

    for (std::size_t i = 0; i < expected.size(); ++i)
    {
        SCOPED_TRACE("sensor_id " + std::to_string(i + 1));
        EXPECT_NEAR(sensors[i].x, expected[i].x, kTol);
        EXPECT_NEAR(sensors[i].y, expected[i].y, kTol);
        EXPECT_NEAR(sensors[i].yaw, expected[i].yaw, kTol);
    }
}

// ---------------------------------------------------------------------------
// Group 2: RadarScenesSource on small synthetic files
// ---------------------------------------------------------------------------

// On equal timestamps the radar scan is emitted before the odometry sample
// (docs/conventions.md, "Replay model").
TEST_F(LoaderTest, SourceEmitsRadarBeforeOdometryOnTie)
{
    const auto path = write_h5(
        {
            {.timestamp = 100},
            {.timestamp = 200},
        },
        {
            {.timestamp = 200, .sensor_id = 1, .range_sc = 10.0F},
            {.timestamp = 200, .sensor_id = 1, .range_sc = 11.0F},
        });

    rt::RadarScenesSource source(path);

    // odometry at 100 comes first: it's strictly earlier
    auto msg = source.next();
    ASSERT_TRUE(msg.has_value());
    ASSERT_TRUE(std::holds_alternative<rt::OdometrySample>(*msg));
    EXPECT_EQ(std::get<rt::OdometrySample>(*msg).t_us, 100);

    // tie at 200: the scan wins, carrying both rows
    msg = source.next();
    ASSERT_TRUE(msg.has_value());
    ASSERT_TRUE(std::holds_alternative<rt::Scan>(*msg));
    const auto& scan = std::get<rt::Scan>(*msg);
    EXPECT_EQ(scan.t_us, 200);
    EXPECT_EQ(scan.sensor_id, 1);
    EXPECT_EQ(scan.measurements.size(), 2U);

    // then the tied odometry sample
    msg = source.next();
    ASSERT_TRUE(msg.has_value());
    ASSERT_TRUE(std::holds_alternative<rt::OdometrySample>(*msg));
    EXPECT_EQ(std::get<rt::OdometrySample>(*msg).t_us, 200);

    // and the stream is done
    EXPECT_FALSE(source.next().has_value());
}

// Two sensors sharing a timestamp come out as two scans, each holding only its
// own rows (the sequence_34/37/128 case). Also checks field values survive the
// float32 -> double read, which guards the compound-type layout.
TEST_F(LoaderTest, SourceSplitsSensorsSharingATimestamp)
{
    const auto path =
        write_h5({{.timestamp = 500}}, {
                                           {.timestamp = 100, .sensor_id = 1, .range_sc = 1.5F},
                                           {.timestamp = 100, .sensor_id = 1, .range_sc = 2.5F},
                                           {.timestamp = 100,
                                            .sensor_id = 2,
                                            .range_sc = 3.5F,
                                            .azimuth_sc = -0.25F,
                                            .rcs = 7.0F,
                                            .vr = -4.0F},
                                       });

    rt::RadarScenesSource source(path);

    auto msg = source.next();
    ASSERT_TRUE(msg.has_value());
    ASSERT_TRUE(std::holds_alternative<rt::Scan>(*msg));
    const auto first = std::get<rt::Scan>(*msg);
    EXPECT_EQ(first.sensor_id, 1);
    ASSERT_EQ(first.measurements.size(), 2U);
    EXPECT_NEAR(first.measurements[0].range_sc, 1.5, 1e-6);
    EXPECT_NEAR(first.measurements[1].range_sc, 2.5, 1e-6);

    msg = source.next();
    ASSERT_TRUE(msg.has_value());
    ASSERT_TRUE(std::holds_alternative<rt::Scan>(*msg));
    const auto& second = std::get<rt::Scan>(*msg);
    EXPECT_EQ(second.sensor_id, 2);
    EXPECT_EQ(second.t_us, 100);
    ASSERT_EQ(second.measurements.size(), 1U);
    EXPECT_NEAR(second.measurements[0].range_sc, 3.5, 1e-6);
    EXPECT_NEAR(second.measurements[0].azimuth_sc, -0.25, 1e-6);
    EXPECT_NEAR(second.measurements[0].rcs, 7.0, 1e-6);
    EXPECT_NEAR(second.measurements[0].vr, -4.0, 1e-6);
}

// Whichever table runs out first, every row of the other still comes out, the
// last scan ends cleanly at the end of radar_data, and nullopt is sticky.
TEST_F(LoaderTest, SourceDrainsBothTablesWhicheverEndsFirst)
{
    {
        SCOPED_TRACE("radar outlasts odometry");
        const auto path = write_h5({{.timestamp = 50}}, {
                                                            {.timestamp = 100, .sensor_id = 1},
                                                            {.timestamp = 200, .sensor_id = 1},
                                                            {.timestamp = 200, .sensor_id = 1},
                                                        });
        rt::RadarScenesSource source(path);
        const auto out = drain(source);
        EXPECT_EQ(out.scans, 2U);
        EXPECT_EQ(out.measurements, 3U);
        EXPECT_EQ(out.odometry, 1U);
        EXPECT_FALSE(source.next().has_value());
        EXPECT_FALSE(source.next().has_value());
    }
    {
        SCOPED_TRACE("odometry outlasts radar");
        const auto path = write_h5({{.timestamp = 50}, {.timestamp = 150}, {.timestamp = 250}},
                                   {{.timestamp = 100, .sensor_id = 3}});
        rt::RadarScenesSource source(path);
        const auto out = drain(source);
        EXPECT_EQ(out.scans, 1U);
        EXPECT_EQ(out.measurements, 1U);
        EXPECT_EQ(out.odometry, 3U);
        EXPECT_FALSE(source.next().has_value());
    }
}

// A timestamp going backwards in either table is an error. The check runs as
// messages are emitted, so the throw comes from next(), not the constructor.
TEST_F(LoaderTest, SourceThrowsOnBackwardsTimestamp)
{
    {
        SCOPED_TRACE("radar_data goes backwards");
        const auto path = write_h5({{.timestamp = 1000}}, {
                                                              {.timestamp = 200, .sensor_id = 1},
                                                              {.timestamp = 100, .sensor_id = 1},
                                                          });
        rt::RadarScenesSource source(path);
        EXPECT_THROW(
            {
                while (source.next())
                {
                }
            },
            std::runtime_error);
    }
    {
        SCOPED_TRACE("odometry goes backwards");
        const auto path = write_h5({{.timestamp = 200}, {.timestamp = 100}},
                                   {{.timestamp = 1000, .sensor_id = 1}});
        rt::RadarScenesSource source(path);
        EXPECT_THROW(
            {
                while (source.next())
                {
                }
            },
            std::runtime_error);
    }
}

// ---------------------------------------------------------------------------
// Group 3: RadarScenesSource on real data (skipped without RT_DATA_DIR)
// ---------------------------------------------------------------------------

// Every row of both tables comes out exactly once, timestamps never go
// backwards, and every scan is one sensor at one timestamp (checked in drain()).
// sequence_34 is one where two sensors share a timestamp.
TEST(SourceRealData, ReplaysWholeSequences)
{
    struct Case
    {
        std::string name;
        std::size_t radar_rows;
        std::size_t odo_rows;
    };

    // row counts from h5dump
    const std::vector<Case> cases = {
        {"sequence_1", 555'979, 9'005},
        {"sequence_34", 399'886, 9'693},
    };

    for (const auto& c : cases)
    {
        const auto path = real_sequence(c.name);
        if (path.empty())
        {
            GTEST_SKIP() << "RT_DATA_DIR not set or " << c.name << " missing";
        }

        SCOPED_TRACE(c.name);
        rt::RadarScenesSource source(path);
        const auto out = drain(source);
        EXPECT_EQ(out.measurements, c.radar_rows);
        EXPECT_EQ(out.odometry, c.odo_rows);
    }
}
