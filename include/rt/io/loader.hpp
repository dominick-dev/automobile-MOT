#pragma once
#include "../types.hpp"
#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace rt
{

class RadarScenesSource
{
  public:
    explicit RadarScenesSource(const std::string& h5_path);
    std::optional<rt::SensorMessage> next();

  private:
    // data vectors
    std::vector<rt::OdometrySample> odo_;
    std::vector<rt::Measurement> radar_;
    // position cursors into data vectors
    std::size_t odo_idx_{};
    std::size_t radar_idx_{};
};

std::array<rt::SensorMount, kMaxSensors> load_sensor_data(const std::string& sensor_path);

} // namespace rt
