/**
 * Command line entry point for the radar tracker
 *
 * Resolves a RadarScene sequence from the command line and hands
 * its files to the rt lib
 */

#include "rt/io/loader.hpp"
#include "rt/types.hpp"

#include <array>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#define SENSORS_PATH "data/RadarScenes/data/sensors.json"

namespace fs = std::filesystem;

namespace
{

constexpr const char* kDataDirEnv = "RT_DATA_DIR";

// Resolves a sequence arg to a directory
// Returns std::nullptr if neither form names an existing directory
std::optional<fs::path> resolve_seq(const std::string& arg)
{
    if (fs::is_directory(arg))
    {
        return fs::path(arg);
    }

    if (const char* root = std::getenv(kDataDirEnv))
    {
        fs::path candidate = fs::path(root) / arg;
        if (fs::is_directory(candidate))
        {
            return candidate;
        }
    }

    return std::nullopt;
}

} // namespace

int main(int argc, char** argv)
{
    // TODO: extend this to also take an "-all" flag, processes all available sequences
    // arg check
    if (argc < 2)
    {
        std::cerr << "usage: run <sequence>\n"
                     " <sequence> is a sequence directory, or a name like sequence_1\n"
                     "looked up under $RT_DATA_DIR\n";

        return 1;
    }

    // validate sequence
    const auto seq_path = resolve_seq(argv[1]);

    if (!seq_path)
    {
        std::cerr << "sequence not found: " << argv[1] << "\n";

        if (std::getenv(kDataDirEnv) == nullptr)
        {
            std::cerr << "hint: RT_DATA_DIR is not yet set!\n";
        }

        return 1;
    }

    // look for h5 file given sequence
    const fs::path h5_path = *seq_path / "radar_data.h5";

    if (!fs::exists(h5_path))
    {
        std::cerr << "missing " << h5_path << "\n";

        return 1;
    }

    const fs::path sensors_path = SENSORS_PATH;

    if (!fs::exists(sensors_path))
    {
        std::cerr << "missing " << SENSORS_PATH << "\n";

        return 1;
    }

    // load sensors.json into memory
    // sensor id i is indexed to i - 1
    std::array<rt::SensorMount, rt::kMaxSensors> sensors;
    sensors = rt::load_sensor_data(SENSORS_PATH);
    std::cout << "sensors loaded!" << "\n";

    rt::RadarScenesSource RadarScenesSource = rt::RadarScenesSource(h5_path.string());

    // main loop, consume sensor messages one by one and process
    while (auto curr_msg = RadarScenesSource.next())
    {
    }

    // TODO: test, refactor, write some optimization ideas down

    return 0;
}
