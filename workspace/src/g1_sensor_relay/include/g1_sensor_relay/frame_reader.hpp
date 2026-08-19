#ifndef G1_SENSOR_RELAY__FRAME_READER_HPP_
#define G1_SENSOR_RELAY__FRAME_READER_HPP_

/**
 * @file frame_reader.hpp
 * @brief Framing and validation for the simulator's sensor stream, free of ROS and sockets.
 *
 * Split out so the wire format is testable without a simulator, a socket or a running
 * graph, same discipline as g1_state_estimation's odom_math.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "g1_sensor_relay/sensor_frame.h"

namespace g1_sensor_relay
{

/**
 * @brief Why a frame was rejected. Anything but kOk means the bytes are not trustworthy.
 */
enum class FrameStatus
{
    kOk,
    kIncomplete,  ///< Not enough bytes yet; wait for more.
    kBadMagic,    ///< Not our stream, or the stream desynchronised.
    kBadVersion,  ///< Producer and consumer disagree on the layout.
    kBadKind,     ///< A frame type this build does not know.
    kBadLength,   ///< payload_bytes disagrees with point_count, or exceeds the sane cap.
};

/// Refuses an object frame with more records than this. Same reasoning as kMaxPoints, at a
/// scale that suits a hand-listed set of scene bodies.
inline constexpr std::uint32_t kMaxObjects = 1024;

/**
 * @brief What a validated frame turned out to be.
 */
enum class FrameKind
{
    kPointCloud,
    kDepth,
    kObjectPoses,
    kImu,
    kBaseState,
};

/**
 * @brief A validated frame.
 *
 * `kind` says which payload interpretation applies: `points` is xyz triples in the sensor
 * frame, `depth` is metres row-major top-down, and `objects` is ground-truth body poses in
 * the simulator's world frame.
 */
struct CloudFrame
{
    FrameKind          kind     = FrameKind::kPointCloud;
    std::uint32_t      width    = 0;
    std::uint32_t      height   = 0;
    float              fovy_deg = 0.0F;
    std::vector<float> depth;
    /// rgb8, row-major, top-down, same dimensions as `depth`. Empty when colour is off.
    std::vector<std::uint8_t>               rgb;
    double                                  sim_time_s = 0.0;
    std::array<double, 3>                   sensor_pos{};
    std::array<double, 4>                   sensor_quat{};
    std::vector<float>                      points;
    std::vector<grove_g1::ObjectPoseRecord> objects;
    /// Rates at the sensor's own frame. Only meaningful on FrameKind::kImu, where the pose
    /// fields above carry the IMU's attitude.
    grove_g1::ImuSampleRecord imu{};
    /// Body-frame pelvis twist. Only meaningful on FrameKind::kBaseState, where the pose
    /// fields above carry the pelvis pose in the world.
    grove_g1::BaseStateRecord base{};
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
 * On FrameStatus::kOk the consumed bytes are erased from `buffer` and `out` is filled. On
 * kIncomplete nothing is consumed. On any other status the caller must drop the connection:
 * the stream cannot be resynchronised, and pretending otherwise turns a framing bug into
 * plausible-looking point clouds.
 */
FrameStatus tryReadFrame(std::vector<std::uint8_t>& buffer, CloudFrame& out);

/**
 * @brief Human-readable status, for logging.
 */
const char* toString(FrameStatus status);

}  // namespace g1_sensor_relay

#endif  // G1_SENSOR_RELAY__FRAME_READER_HPP_
