#include "rt/io/loader.hpp"
#include "rt/types.hpp"
#include <fstream>
#include <highfive/H5DataType.hpp>
#include <highfive/H5File.hpp>
#include <nlohmann/json.hpp>
#include <optional>

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

    // both done, reutrn null
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

        std::cout << "Num scan: " << scan.measurements.size() << "\n";
        return scan;
    }

    // read odo
    rt::OdometrySample curr_odo = RadarScenesSource::odo_.at(RadarScenesSource::odo_idx_);

    // odo wins
    RadarScenesSource::odo_idx_++;
    return curr_odo;
}

// load in sensor data from sensors.json
std::array<rt::SensorMount, kMaxSensors> load_sensor_data(const std::string& sensor_path)
{
    std::array<SensorMount, kMaxSensors> sensors;

    std::ifstream i(sensor_path);

    nlohmann::json data;

    i >> data;

    size_t idx = 0;

    for (const auto& dat : data.items())
    {
        SensorMount new_sensor;

        new_sensor.x = dat.value().at("x").get<int>();
        new_sensor.y = dat.value().at("y").get<int>();
        new_sensor.yaw = dat.value().at("yaw").get<int>();

        sensors[idx] = new_sensor;

        idx++;
    }

    return sensors;
}

} // namespace rt
