#pragma once

#include "rt/types.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace rt
{

class FrameAssembler
{
  public:
    void push(rt::SensorMessage msg);
    [[nodiscard]]
    std::optional<Frame> pop_ready();
    [[nodiscard]]
    std::int64_t dropped_frames() const;

  private:
    // queue of frames that are ready to be released
    std::deque<Frame> frame_queue_;
    // index of next frame to release
    std::int64_t k_{};
    // starting time (time of first scan timestamp, in us)
    std::optional<std::int64_t> t0_;
    // buffered scans that have been read
    std::deque<Scan> scan_buff_;
    // buffered odo that have been read
    std::deque<OdometrySample> odo_buff_;
    // frames discarded for lack of a front bracket
    std::int64_t discarded_{};

    void release(std::int64_t t0);

    [[nodiscard]]
    std::vector<Detection> create_dets(std::int64_t window_end, std::int64_t t0);
    [[nodiscard]]
    std::vector<EgoState> create_egos(std::int64_t window_end, std::int64_t t0) const;
    void discard(std::int64_t window_end);
    void trim_odo(std::int64_t window_start);
};

} // namespace rt
