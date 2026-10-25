#include "rt/io/loader.hpp"
#include "rt/types.hpp"
#include <bitset>
#include <fstream>
#include <highfive/H5DataType.hpp>
#include <highfive/H5File.hpp>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace
{

HighFive::CompoundType make_odo()
{
    using HighFive::create_datatype;
    return {
        {"timestamp", create_datatype<int64_t>(), offsetof(rt::OdometrySample, t_us)},
        {"x_seq", create_datatype<double>(), offsetof(rt::OdometrySample, x_seq)},
        {"y_seq", create_datatype<double>(), offsetof(rt::OdometrySample, y_seq)},
        {"yaw_seq", create_datatype<double>(), offsetof(rt::OdometrySample, yaw_seq)},
        {"vx", create_datatype<double>(), offsetof(rt::OdometrySample, vx)},
        {"yaw_rate", create_datatype<double>(), offsetof(rt::OdometrySample, yaw_rate)},
    };
}

HighFive::CompoundType make_measurement()
{
    using HighFive::create_datatype;
    return {
        {"timestamp", create_datatype<int64_t>(), offsetof(rt::Measurement, t_us)},
        {"sensor_id", create_datatype<uint8_t>(), offsetof(rt::Measurement, sensor_id)},
        {"range_sc", create_datatype<double>(), offsetof(rt::Measurement, range_sc)},
        {"azimuth_sc", create_datatype<double>(), offsetof(rt::Measurement, azimuth_sc)},
        {"rcs", create_datatype<double>(), offsetof(rt::Measurement, rcs)},
        {"vr", create_datatype<double>(), offsetof(rt::Measurement, vr)},
    };
}

} // namespace

HIGHFIVE_REGISTER_TYPE(rt::OdometrySample, make_odo)
HIGHFIVE_REGISTER_TYPE(rt::Measurement, make_measurement)

namespace rt
{

RadarScenesSource::RadarScenesSource(const std::string& H5_path)
{
    HighFive::File f(H5_path, HighFive::File::ReadOnly);

    // open each DataSet
    auto odo_ds = f.getDataSet("odometry");
    auto radar_ds = f.getDataSet("radar_data");

    // read each dataset into a vector, stored as private member
    odo_ds.read(RadarScenesSource::odo_);
    radar_ds.read(RadarScenesSource::radar_);
}

std::optional<rt::SensorMessage> RadarScenesSource::next()
{
    // bounds check
    bool odo_done = false;
    bool radar_done = false;

    if (RadarScenesSource::odo_idx_ >= RadarScenesSource::odo_.size())
    {
        odo_done = true;
    }

    if (RadarScenesSource::radar_idx_ >= RadarScenesSource::radar_.size())
    {
        radar_done = true;
    }

    // both done, return null
    if (odo_done && radar_done)
    {
        return std::nullopt;
    }

    // read radar
    if (!radar_done &&
        (odo_done || RadarScenesSource::odo_.at(RadarScenesSource::odo_idx_).t_us >=
                         RadarScenesSource::radar_.at(RadarScenesSource::radar_idx_).t_us))
    {
        rt::Measurement curr_radar = RadarScenesSource::radar_.at(RadarScenesSource::radar_idx_);

        std::int64_t curr_scan_ts = curr_radar.t_us;
        std::uint8_t curr_sensor_id = curr_radar.sensor_id;

        rt::Scan scan{};
        scan.sensor_id = curr_radar.sensor_id;
        scan.t_us = curr_scan_ts;

        // catch out of order measurements
        if (curr_scan_ts < RadarScenesSource::prev_ts)
        {
            throw std::runtime_error(
                "ERROR: encountered out of order timestamp in loader (radar): curr_timestamp=" +
                std::to_string(curr_scan_ts) +
                ", prev_timestamp=" + std::to_string(RadarScenesSource::prev_ts));
        }

        // while curr row is from same radar and same timestamp
        while (RadarScenesSource::radar_idx_ < RadarScenesSource::radar_.size())
        {
            curr_radar = RadarScenesSource::radar_.at(RadarScenesSource::radar_idx_);

            if (curr_radar.t_us != curr_scan_ts || curr_radar.sensor_id != curr_sensor_id)
            {
                break;
            }

            // add measurement
            scan.measurements.push_back(curr_radar);

            // advance curr_radar
            RadarScenesSource::radar_idx_++;
        }

        RadarScenesSource::prev_ts = curr_scan_ts;
        return scan;
    }

    // read odo
    rt::OdometrySample curr_odo = RadarScenesSource::odo_.at(RadarScenesSource::odo_idx_);

    if (curr_odo.t_us < RadarScenesSource::prev_ts)
    {
        throw std::runtime_error(
            "ERROR: encountered out of order timestamp in loader (odo): curr_timestamp=" +
            std::to_string(curr_odo.t_us) +
            ", prev_timestamp=" + std::to_string(RadarScenesSource::prev_ts));
    }

    // odo wins
    RadarScenesSource::odo_idx_++;
    RadarScenesSource::prev_ts = curr_odo.t_us;
    return curr_odo;
}

// load in sensor data from sensors.json
std::array<rt::SensorMount, kMaxSensors> load_sensor_data(const std::string& sensor_path)
{
    std::array<SensorMount, rt::kMaxSensors> sensors{};
    std::bitset<rt::kMaxSensors> occupied_slots{};

    std::ifstream i(sensor_path);

    if (!i.is_open())
    {
        throw std::runtime_error("ERROR: could not open sensors data at: " + sensor_path);
    }

    nlohmann::json data;

    i >> data;

    for (const auto& dat : data.items())
    {
        SensorMount new_sensor;

        new_sensor.x = dat.value().at("x").get<double>();
        new_sensor.y = dat.value().at("y").get<double>();
        new_sensor.yaw = dat.value().at("yaw").get<double>();

        auto curr_id = dat.value().at("id").get<int>();

        if (curr_id < 1 || std::cmp_greater(curr_id, rt::kMaxSensors))
        {
            throw std::runtime_error(
                "ERROR: encountered out of range sensor in loader: sensor_id=" +
                std::to_string(curr_id));
        }

        if (occupied_slots.test(curr_id - 1))
        {
            throw std::runtime_error("ERROR: encountered two sensors with the same id");
        }

        sensors[curr_id - 1] = new_sensor;
        occupied_slots.set(curr_id - 1, true);
    }

    if (!occupied_slots.all())
    {
        throw std::runtime_error("ERROR: sensor id missing after load");
    }

    return sensors;
}

} // namespace rt
