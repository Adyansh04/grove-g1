#ifndef G1_PERCEPTION__DEPTH_HISTORY_HPP_
#define G1_PERCEPTION__DEPTH_HISTORY_HPP_

/**
 * @file depth_history.hpp
 * @brief Keeps recent depth frames so a late mask can be paired with the frame it was cut from.
 *
 * The detector takes over a second per frame. Pairing its masks with the newest depth instead
 * would deproject an object's outline onto whatever has since moved under it, which reads as a
 * confident pose in the wrong place rather than as a failure.
 */

#include <cstddef>
#include <deque>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace g1_perception
{

class DepthHistory
{
public:
    /**
     * @param history_s   How far back frames are kept. Sized by the detector's worst latency.
     * @param tolerance_s How closely a frame's stamp must match the mask's. The relay stamps
     *                    depth and colour identically from one render, so this is tight.
     */
    DepthHistory(double history_s, double tolerance_s);

    /// Stores a frame and drops anything older than @p history_s behind the newest one.
    void push(sensor_msgs::msg::Image::ConstSharedPtr frame);

    /// The frame captured at @p stamp_s, or nullptr when none is within the tolerance.
    [[nodiscard]] sensor_msgs::msg::Image::ConstSharedPtr at(double stamp_s) const;

    [[nodiscard]] std::size_t size() const { return frames_.size(); }

    /// Seconds in a message stamp, which is all this class compares.
    [[nodiscard]] static double stampSeconds(const std_msgs::msg::Header& header);

private:
    double                                            history_s_;
    double                                            tolerance_s_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> frames_;
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__DEPTH_HISTORY_HPP_
