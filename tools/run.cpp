/**
 * Command line entry point for the radar tracker
 *
 * Resolves a RadarScene sequence from the command line and hands
 * its files to the rt lib
 */

#include "rt/loader.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

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
    if (argc < 2)
    {
        std::cerr << "usage: run <sequence>\n"
                     " <sequence> is a sequence directory, or a name like sequence_1\n"
                     "looked up under $RT_DATA_DIR\n";

        return 1;
    }

    const auto seq = resolve_seq(argv[1]);

    if (!seq)
    {
        std::cerr << "sequence not found: " << argv[1] << "\n";

        if (std::getenv(kDataDirEnv) == nullptr)
        {
            std::cerr << "hint: RT_DATA_DIR is not yet set!\n";
        }

        return 1;
    }

    const fs::path h5 = *seq / "radar_data.h5";
    if (!fs::exists(h5))
    {
        std::cerr << "missing " << h5 << "\n";

        return 1;
    }

    std::cout << seq->filename().string() << ": " << rt::count_detections(h5.string())
              << " count_detections\n";

    return 0;
}
