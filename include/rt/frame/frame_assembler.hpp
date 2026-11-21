#pragma once

#include "rt/types.hpp"

#include <cstdint>
#include <deque>
#include <optional>

namespace rt
{

class FrameAssembler
{
  public:
    void push(std::optional<rt::SensorMessage> msg);
    std::optional<Frame> pop_ready();

  private:
    // queue of frames that are ready to be released
    std::deque<Frame> frame_queue_;
    // index of next frame to release
    std::int64_t k_{};
    // starting time (time of first scan timestamp, in us)
    std::optional<std::int64_t> t0_{std::nullopt};
    // buffered scans that have been read
    std::deque<Scan> scan_buff_;
    // buffered odo that have been read
    std::deque<OdometrySample> odo_buff_;

    void release();
};

} // namespace rt
