#pragma once

// Definitions of types that cross pipeline stage boundaries

#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

namespace rt
{

inline constexpr std::size_t kMaxSensors = 4;

struct SensorMount
{
    double x{};   // meters
    double y{};   // meters
    double yaw{}; // radians, CCW from car +x
};

struct OdometrySample
{
    std::int64_t t_us{}; // us
    double x_seq{};      // position, meters
    double y_seq{};      // position, meters
    double yaw_seq{};    // heading, radians
    double vx{};         // forward velocity, m/s
    double yaw_rate{};   // CCW (+), rad/s
};

struct Measurement
{
    std::int64_t t_us{};      // us
    std::uint8_t sensor_id{}; // sensor that collected measurement
    double range_sc{};        // radial distance to detection, meters
    double azimuth_sc{};      // from sensor boresight, counterclockwise pos, radians
    double rcs{};             // dBsm
    double vr{};              // radial velocity, m/s, (-) means approaching
};

struct Scan
{
    std::uint8_t sensor_id{};              // scanning sensor id
    std::int64_t t_us{};                   // time of scan, us
    std::vector<Measurement> measurements; // raw measurements in scan
};

using SensorMessage = std::variant<Scan, OdometrySample>;

} // namespace rt
