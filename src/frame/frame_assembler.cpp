#include "rt/frame/frame_assembler.hpp"
#include "rt/types.hpp"

#include <cstdint>
#include <iostream>
#include <optional>
#include <variant>
#include <vector>

namespace rt
{

void FrameAssembler::push(std::optional<rt::SensorMessage> msg)
{
    // check msg type
    if (!msg)
    {
        std::cout << "msg was null!\n";
    }

    // if scan
    if (const auto* scan = std::get_if<Scan>(&(*msg)))
    {
        // set t0 if not already done, convert us to s
        if (FrameAssembler::t0_ == std::nullopt)
        {
            FrameAssembler::t0_ = scan->t_us;
        }

        // add to scan buffer
        FrameAssembler::scan_buff_.emplace_back(*scan);
    }
    // if odo more work!
    else if (const auto* odo = std::get_if<OdometrySample>(&(*msg)))
    {
        // add to odo buffer
        FrameAssembler::odo_buff_.emplace_back(*odo);

        // released release is skipped if an odo message arrives and t0 is not yet set after
        if (FrameAssembler::t0_ != std::nullopt)
        {
            FrameAssembler::release();
        }
    }
}

void FrameAssembler::release()
{
    std::int64_t window_start{};
    std::int64_t window_end{};

    while (true)
    {

        // get window start and end for k (in us)
        window_start = FrameAssembler::t0_.value() + (FrameAssembler::k_ * kFrameWindowSize);
        window_end = window_start + kFrameWindowSize;

        // break if last odo < window end -1, last odo released no new frames
        if (FrameAssembler::odo_buff_.empty() ||
            FrameAssembler::odo_buff_.back().t_us < (window_end - 1))
        {
            return;
        }

        // if yes then either create frame k (has front odo bracket) or discarding frame k (does not
        // have a front bracket)
        if (!FrameAssembler::odo_buff_.empty() &&
            FrameAssembler::odo_buff_.front().t_us > window_start)
        {
            // on discard discard k's scans, increment k, and continue with release check
            while (!FrameAssembler::scan_buff_.empty() &&
                   FrameAssembler::scan_buff_.front().t_us < window_end)
            {
                FrameAssembler::scan_buff_.pop_front();
            }
        }
        else
        {
            // on create new frame...

            // create detections for new frame
            std::vector<Detection> dets{};
            // pop off front of scan deque and collect until hit scan w/ ts >= window end - 1
            // stopping condition scan above still gets copied!
            while (!FrameAssembler::scan_buff_.empty() &&
                   FrameAssembler::scan_buff_.front().t_us < window_end)
            {
                const auto curr_scan = FrameAssembler::scan_buff_.front();

                for (const auto& m : curr_scan.measurements)
                {
                    // while popping off, create Detection for each scan, convert to s
                    dets.push_back(Detection{
                        .t_s = static_cast<double>(m.t_us - FrameAssembler::t0_.value()) / 1E6,
                        .sensor_id = m.sensor_id,
                        .range_sc = m.range_sc,
                        .azimuth_sc = m.azimuth_sc,
                        .rcs = m.rcs,
                        .vr = m.vr,
                    });
                }

                FrameAssembler::scan_buff_.pop_front();
            }

            // trim odo buff for current window k
            // keep the latest sample <= window start, drop all else
            while (FrameAssembler::odo_buff_.size() > 1 &&
                   (FrameAssembler::odo_buff_.at(1).t_us <= window_start))
            {
                FrameAssembler::odo_buff_.pop_front();
            }

            // create EgoStates for new frame
            std::vector<EgoState> egos{};
            bool hit_back = false;

            for (const auto& odo : FrameAssembler::odo_buff_)
            {
                if (hit_back)
                {
                    break;
                }

                egos.push_back(EgoState{
                    .t_s = static_cast<double>(odo.t_us - FrameAssembler::t0_.value()) / 1E6,
                    .x_seq = odo.x_seq,
                    .y_seq = odo.y_seq,
                    .yaw_seq = odo.yaw_seq,
                    .vx = odo.vx,
                    .yaw_rate = odo.yaw_rate,
                });

                if (odo.t_us >= window_end - 1)
                {
                    hit_back = true;
                }
            }

            // when done add this new frame to frame_queue_
            FrameAssembler::frame_queue_.push_back(Frame{
                .frame_idx = FrameAssembler::k_,
                .frame_time = ((static_cast<double>((window_start + window_end)) / 2) -
                               static_cast<double>(FrameAssembler::t0_.value())) /
                              1E6,
                .detections = dets,
                .ego_states = egos,
            });
        }

        // increment k and loop back to top in case multiple frames are released
        FrameAssembler::k_++;
    }
}

std::optional<Frame> FrameAssembler::pop_ready()
{
    if (!FrameAssembler::frame_queue_.empty())
    {
        auto returned = std::move(FrameAssembler::frame_queue_.front());
        FrameAssembler::frame_queue_.pop_front();

        return returned;
    }

    return std::nullopt;
}

} // namespace rt
