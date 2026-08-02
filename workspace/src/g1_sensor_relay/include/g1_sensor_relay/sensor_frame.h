#ifndef GROVE_G1_SENSOR_FRAME_H_
#define GROVE_G1_SENSOR_FRAME_H_

// Wire format between the patched unitree_mujoco and g1_sensor_relay.
//
// Shared verbatim by both sides: the copy under workspace/vendor is the one compiled into
// the simulator, and g1_sensor_relay includes the same file. Keep them identical; a
// test in g1_sensor_relay asserts the two copies match byte for byte.

#include <cstdint>

namespace grove_g1
{

// Bumped whenever the layout below changes. The relay refuses a frame it does not know
// rather than reinterpreting bytes.
inline constexpr uint32_t kSensorFrameVersion = 1;

inline constexpr uint32_t kSensorFrameMagic = 0x47314C44;  // "G1LD"

enum class SensorFrameKind : uint32_t
{
    PointCloud = 1,
};

// Fixed-size header, then `payload_bytes` of body. Length-prefixed so the stream can be
// framed without parsing the body, and so a short read is detectable rather than silently
// producing a truncated cloud.
struct SensorFrameHeader
{
    uint32_t magic;          // kSensorFrameMagic
    uint32_t version;        // kSensorFrameVersion
    uint32_t kind;           // SensorFrameKind
    uint32_t payload_bytes;  // body length that follows this header

    // Sim time of the snapshot the frame was computed from, not wall time: the relay stamps
    // messages with this so downstream TF lookups line up with /clock.
    double sim_time_s;

    // Sensor pose in the world at snapshot time, as position + wxyz quaternion. The relay
    // publishes the cloud in the sensor frame, so this is carried for provenance and for
    // the relay's own sanity checks rather than to transform the points.
    double sensor_pos[3];
    double sensor_quat[4];

    // PointCloud: number of points that follow, each 3 floats (x, y, z) in the sensor frame.
    uint32_t point_count;
    uint32_t reserved;
};

static_assert(sizeof(SensorFrameHeader) == 88, "wire layout changed; bump kSensorFrameVersion");

}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_FRAME_H_
