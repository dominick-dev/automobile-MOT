#pragma once
#include "rt/types.hpp"
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <vector>

namespace rt
{

class RadarScenesSource
{
  public:
    explicit RadarScenesSource(const std::filesystem::path& h5_path);
    [[nodiscard]]
    std::optional<rt::SensorMessage> next();

  private:
    // data vectors
    std::vector<rt::OdometrySample> odo_;
    std::vector<rt::Measurement> radar_;
    // position cursors into data vectors
    std::size_t odo_idx_{};
    std::size_t radar_idx_{};
    // error checking state
    std::int64_t prev_ts{};
};

[[nodiscard]]
SensorArray load_sensor_data(const std::filesystem::path& sensor_path);

} // namespace rt
