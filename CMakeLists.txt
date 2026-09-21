cmake_minimum_required(VERSION 3.21)
project(radar_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# dependencies
find_package(HDF5 REQUIRED COMPONENTS C)
find_package(Eigen3 REQUIRED NO_MODULE)

include(FetchContent)
FetchContent_Declare(HighFive
  GIT_REPOSITORY https://github.com/BlueBrain/HighFive.git
  GIT_TAG v2.10.1
  SOURCE_SUBDIR no-cmake
)
FetchContent_MakeAvailable(HighFive)

FetchContent_Declare(json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# library, future me add new .cpp files here as they are made
add_library(rt
  src/io/radarscenes_loader.cpp)

target_include_directories(rt
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(rt SYSTEM PRIVATE ${highfive_SOURCE_DIR}/include)

target_link_libraries(rt
  PUBLIC Eigen3::Eigen
  PRIVATE HDF5::HDF5 nlohmann_json::nlohmann_json
)

target_compile_options(rt PRIVATE -Wall -Wextra -Wpedantic)

# executable
add_executable(run tools/run.cpp)
target_link_libraries(run PRIVATE rt)
target_compile_options(run PRIVATE -Wall -Wextra -Wpedantic)

