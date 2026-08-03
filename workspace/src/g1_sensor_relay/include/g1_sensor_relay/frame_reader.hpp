#ifndef G1_SENSOR_RELAY__FRAME_READER_HPP_
#define G1_SENSOR_RELAY__FRAME_READER_HPP_

/**
 * @file frame_reader.hpp
 * @brief Framing and validation for the simulator's sensor stream, free of ROS and sockets.
 *
 * Split out so the wire format is testable without a simulator, a socket or a running
 * graph, same discipline as g1_state_estimation's odom_math.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "g1_sensor_relay/sensor_frame.h"

namespace g1_sensor_relay
{

/// Why a frame was rejected. Anything but Ok means the bytes are not trustworthy.
enum class FrameStatus
{
    Ok,
    Incomplete,  ///< Not enough bytes yet; wait for more.
    BadMagic,    ///< Not our stream, or the stream desynchronised.
    BadVersion,  ///< Producer and consumer disagree on the layout.
    BadKind,     ///< A frame type this build does not know.
    BadLength,   ///< payload_bytes disagrees with point_count, or exceeds the sane cap.
};

/// What a validated frame turned out to be.
enum class FrameKind
{
    PointCloud,
    Depth,
};

/// A validated frame. `kind` says which payload interpretation applies: `points` is xyz
/// triples in the sensor frame, `depth` is metres, row-major, top-down.
struct CloudFrame
{
    FrameKind          kind     = FrameKind::PointCloud;
    std::uint32_t      width    = 0;
    std::uint32_t      height   = 0;
    float              fovy_deg = 0.0f;
    std::vector<float> depth;
    /// rgb8, row-major, top-down, same dimensions as `depth`. Empty when colour is off.
    std::vector<std::uint8_t> rgb;
    double                    sim_time_s = 0.0;
    double                    sensor_pos[3]{};
    double                    sensor_quat[4]{};
    std::vector<float>        points;
};

/**
 * @brief Refuses a frame larger than this many points.
 *
 * A desynchronised stream produces a garbage length, and trusting it means a multi-gigabyte
 * allocation. Well above any resolution we would configure.
 */
inline constexpr std::uint32_t kMaxPoints = 4'000'000;

/**
 * @brief Attempts to take one frame from the front of `buffer`.
 *
 * On FrameStatus::Ok the consumed bytes are erased from `buffer` and `out` is filled. On
 * Incomplete nothing is consumed. On any other status the caller must drop the connection:
 * the stream cannot be resynchronised, and pretending otherwise turns a framing bug into
 * plausible-looking point clouds.
 */
FrameStatus tryReadFrame(std::vector<std::uint8_t>& buffer, CloudFrame& out);

/// Human-readable status, for logging.
const char* toString(FrameStatus status);

}  // namespace g1_sensor_relay

#endif  // G1_SENSOR_RELAY__FRAME_READER_HPP_
