#include "rt/loader.hpp"
#include <highfive/H5File.hpp>
#include <nlohmann/json.hpp>

namespace rt
{

std::size_t count_detections(const std::string& path)
{
    HighFive::File f(path, HighFive::File::ReadOnly);
    return f.getDataSet("radar_data").getElementCount();
}

} // namespace rt
