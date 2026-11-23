#include "rt/frame/frame_assembler.hpp"
#include "rt/types.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace
{

constexpr double kUsecInSec = 1E6;

[[nodiscard]]
double us_to_s(std::int64_t t_us, std::int64_t t0)
{
    return static_cast<double>(t_us - t0) / kUsecInSec;
}

[[nodiscard]]
rt::Detection meas_to_det(const rt::Measurement& meas, std::int64_t t0)
{
    return rt::Detection{
        .t_s = us_to_s(meas.t_us, t0),
        .sensor_id = meas.sensor_id,
        .range_sc = meas.range_sc,
        .azimuth_sc = meas.azimuth_sc,
        .rcs = meas.rcs,
        .vr = meas.vr,
    };
}

[[nodiscard]]
rt::EgoState odo_to_ego(const rt::OdometrySample& odo, std::int64_t t0)
{
    return rt::EgoState{
        .t_s = us_to_s(odo.t_us, t0),
        .x_seq = odo.x_seq,
        .y_seq = odo.y_seq,
        .yaw_seq = odo.yaw_seq,
        .vx = odo.vx,
        .yaw_rate = odo.yaw_rate,
    };
}

} // namespace

namespace rt
{

void FrameAssembler::push(SensorMessage msg)
{
    // if scan
    if (auto* scan = std::get_if<Scan>(&msg))
    {
        // set t0 if not already done
        if (!t0_.has_value())
        {
            t0_ = scan->t_us;
        }

        // a scan before window k's start belongs to a frame already released or discarded
        const std::int64_t window_start = *t0_ + (k_ * kFrameWindowSizeUs);
        if (scan->t_us < window_start)
        {
            throw std::runtime_error("ERROR: assembler encountered late scan: curr window start=" +
                                     std::to_string(window_start) +
                                     ", scan time=" + std::to_string(scan->t_us));
        }

        // add to scan buffer
        scan_buff_.emplace_back(std::move(*scan));
    }
    // if odo more work!
    else if (const auto* odo = std::get_if<OdometrySample>(&msg))
    {
        // odometry before the window start is normal (front bracket), but it must not go
        // backwards relative to what is already buffered
        if (!odo_buff_.empty() && odo->t_us < odo_buff_.back().t_us)
        {
            throw std::runtime_error(
                "ERROR: assembler encountered out of order odometry: prev odo time=" +
                std::to_string(odo_buff_.back().t_us) + ", odo time=" + std::to_string(odo->t_us));
        }

        // add to odo buffer
        odo_buff_.emplace_back(*odo);

        // release only if t0_ has been set
        if (t0_.has_value())
        {
            release(t0_.value());
        }
    }
}

std::optional<Frame> FrameAssembler::pop_ready()
{
    if (!frame_queue_.empty())
    {
        auto returned = std::move(frame_queue_.front());
        frame_queue_.pop_front();

        return returned;
    }

    return std::nullopt;
}

std::int64_t FrameAssembler::dropped_frames() const
{
    // every window from k through the last buffered scan's window is still waiting
    std::int64_t pending = 0;
    if (t0_.has_value() && !scan_buff_.empty())
    {
        pending = ((scan_buff_.back().t_us - *t0_) / kFrameWindowSizeUs) - k_ + 1;
    }

    return discarded_ + pending;
}

void FrameAssembler::release(const std::int64_t t0)
{
    while (true)
    {
        // get window start and end for k (in us)
        const std::int64_t window_start = t0 + (k_ * kFrameWindowSizeUs);
        const std::int64_t window_end = window_start + kFrameWindowSizeUs;

        // break if last odo < window end -1, last odo released no new frames
        if (odo_buff_.empty() || odo_buff_.back().t_us < (window_end - 1))
        {
            return;
        }

        // either create frame k (has front odo bracket) or discard frame k (no front bracket)
        if (odo_buff_.front().t_us > window_start)
        {
            // on discard discard k's scans, increment k, and continue with release check
            discard(window_end);
        }
        else
        {
            // create detections for new frame
            std::vector<Detection> dets = create_dets(window_end, t0);

            // keep the latest sample <= window start, drop all else
            trim_odo(window_start);

            // create EgoStates for new frame
            std::vector<EgoState> egos = create_egos(window_end, t0);

            // add new frame to frame_queue_
            frame_queue_.push_back(Frame{
                .frame_idx = k_,
                .frame_time = us_to_s(window_start + (kFrameWindowSizeUs / 2), t0),
                .detections = std::move(dets),
                .ego_states = std::move(egos),
            });
        }

        // increment k and loop back in case multiple frames are released
        k_++;
    }
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
std::vector<Detection> FrameAssembler::create_dets(std::int64_t window_end, std::int64_t t0)

{
    std::vector<Detection> dets{};
    while (!scan_buff_.empty() && scan_buff_.front().t_us < window_end)
    {
        const Scan& curr_scan = scan_buff_.front();

        for (const auto& m : curr_scan.measurements)
        {
            // while popping off, create Detection for each scan, convert to s
            dets.push_back(meas_to_det(m, t0));
        }

        scan_buff_.pop_front();
    }

    return dets;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
std::vector<EgoState> FrameAssembler::create_egos(std::int64_t window_end, std::int64_t t0) const
{
    std::vector<EgoState> egos{};

    // get back odo bracket of current window
    const auto back = std::ranges::find_if(odo_buff_, [window_end](const OdometrySample& odo)
                                           { return odo.t_us >= window_end - 1; });

    egos.reserve(std::distance(odo_buff_.begin(), std::next(back)));

    // convert each odo in range to ego, write to egos
    std::ranges::transform(odo_buff_.begin(), std::next(back), std::back_inserter(egos),
                           [t0](const OdometrySample& odo) { return odo_to_ego(odo, t0); });

    return egos;
}

void FrameAssembler::discard(std::int64_t window_end)
{
    while (!scan_buff_.empty() && scan_buff_.front().t_us < window_end)
    {
        scan_buff_.pop_front();
    }

    ++discarded_;
}

void FrameAssembler::trim_odo(std::int64_t window_start)
{
    while (odo_buff_.size() > 1 && (odo_buff_.at(1).t_us <= window_start))
    {
        odo_buff_.pop_front();
    }
}

} // namespace rt
