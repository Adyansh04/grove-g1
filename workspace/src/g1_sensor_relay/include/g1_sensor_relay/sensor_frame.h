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
// rather than reinterpreting bytes. v2 added the depth-image fields; v3 appends colour
// to the depth payload; v4 adds the ObjectPoses kind.
inline constexpr uint32_t kSensorFrameVersion = 4;

inline constexpr uint32_t kSensorFrameMagic = 0x47314C44;  // "G1LD"

enum class SensorFrameKind : uint32_t
{
    PointCloud = 1,
    Depth      = 2,
    // Ground-truth poses of scene bodies, for the sim-only object source. Not a sensor: it
    // rides this socket because it is the only channel out of the simulator process, and it
    // exists for the same reason (the poses live in mjData, which no DDS topic carries).
    ObjectPoses = 3,
};

// One tracked body's ground-truth pose, in the MuJoCo world frame.
//
// Fixed-size so the payload stays a flat POD array the relay validates by length alone,
// like the point and depth payloads. Names are short MuJoCo body names; the simulator
// declines at startup to track one that does not fit, rather than silently truncating it
// into a name no consumer will match.
struct ObjectPoseRecord
{
    char   name[32];  // always NUL-terminated, so at most 31 characters
    double pos[3];
    double quat[4];  // wxyz, MuJoCo's own order
};

static_assert(sizeof(ObjectPoseRecord) == 88, "wire layout changed; bump kSensorFrameVersion");

// Fixed-size header, then `payload_bytes` of body. Length-prefixed so the stream can be
// framed without parsing the body, and so a short read is detectable rather than silently
// producing a truncated cloud.
struct SensorFrameHeader
{
    uint32_t magic;          // kSensorFrameMagic
    uint32_t version;        // kSensorFrameVersion
    uint32_t kind;           // SensorFrameKind
    uint32_t payload_bytes;  // body length that follows this header

    // Sim time of the snapshot the frame was computed from. Carried for provenance; the
    // relay stamps messages with its own clock, because this track publishes no /clock.
    double sim_time_s;

    // Sensor pose in the world at snapshot time, as position + wxyz quaternion. Zero and
    // identity on an ObjectPoses frame, which has no sensor: its records carry world poses
    // directly.
    double sensor_pos[3];
    double sensor_quat[4];

    // PointCloud: number of points, each 3 floats (x, y, z) in the sensor frame.
    //
    // ObjectPoses does NOT use this: its count follows from payload_bytes over the record
    // size, so there is no second number that can disagree with the first.
    uint32_t point_count;

    // Depth: image dimensions and the vertical field of view the render used. fovy is
    // carried rather than assumed so the relay's camera_info cannot drift from the MJCF.
    uint32_t width;
    uint32_t height;
    float    fovy_deg;

    // Depth: bytes of rgb8 colour appended after the depth floats, or 0 when colour is
    // off. Both come from one mjr_readPixels of one render, so they share a pose, a
    // timestamp and a frustum by construction rather than by the relay pairing them up.
    uint32_t rgb_bytes;
};

static_assert(sizeof(SensorFrameHeader) == 104, "wire layout changed; bump kSensorFrameVersion");

}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_FRAME_H_
