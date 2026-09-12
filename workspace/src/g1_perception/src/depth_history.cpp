#include "g1_perception/depth_history.hpp"

#include <cmath>
#include <limits>

namespace g1_perception
{

DepthHistory::DepthHistory(double history_s, double tolerance_s)
    : history_s_(history_s), tolerance_s_(tolerance_s)
{
}

double DepthHistory::stampSeconds(const std_msgs::msg::Header& header)
{
    return static_cast<double>(header.stamp.sec)
           + (static_cast<double>(header.stamp.nanosec) * 1e-9);
}

void DepthHistory::push(sensor_msgs::msg::Image::ConstSharedPtr frame)
{
    if (frame == nullptr)
    {
        return;
    }
    const double newest = stampSeconds(frame->header);
    frames_.push_back(std::move(frame));
    while (!frames_.empty() && newest - stampSeconds(frames_.front()->header) > history_s_)
    {
        frames_.pop_front();
    }
}

sensor_msgs::msg::Image::ConstSharedPtr DepthHistory::at(double stamp_s) const
{
    sensor_msgs::msg::Image::ConstSharedPtr best;
    double                                  best_gap = std::numeric_limits<double>::max();
    for (const sensor_msgs::msg::Image::ConstSharedPtr& frame : frames_)
    {
        const double gap = std::abs(stampSeconds(frame->header) - stamp_s);
        if (gap < best_gap)
        {
            best_gap = gap;
            best     = frame;
        }
    }
    return best_gap <= tolerance_s_ ? best : nullptr;
}

}  // namespace g1_perception
