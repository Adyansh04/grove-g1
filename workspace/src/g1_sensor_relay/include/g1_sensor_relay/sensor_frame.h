#ifndef GROVE_G1_SENSOR_FRAME_H_
#define GROVE_G1_SENSOR_FRAME_H_

// Wire format between the patched unitree_mujoco and g1_sensor_relay.
//
// Both sides compile their own copy of this file; a g1_sensor_relay test keeps the two
// byte-identical.

#include <cstdint>

namespace grove_g1
{

// The spellings and C arrays below are the wire layout, pinned by the sizeof assertions.
// NOLINTBEGIN(readability-identifier-naming,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)

// Bumped whenever the layout below changes; the relay refuses a version it does not know.
inline constexpr uint32_t kSensorFrameVersion = 7;

inline constexpr uint32_t kSensorFrameMagic = 0x47314C44;  // "G1LD"

enum class SensorFrameKind : uint32_t
{
    PointCloud = 1,
    Depth      = 2,
    // Ground-truth poses of scene bodies for the sim-only object source; mjData has no other
    // way out of the simulator.
    ObjectPoses = 3,
    // The Mid360's own IMU, which on the robot reports over the Livox link rather than LowState.
    Imu = 4,
    // Exact pelvis pose and twist for the sim-only ground-truth odometry source.
    BaseState = 5,
};

// Rates at the sensor's own frame; the header's sensor_pos/sensor_quat are the IMU's pose.
struct ImuSampleRecord
{
    double gyro[3];  // rad/s about the sensor's own axes
    double acc[3];   // m/s^2, proper acceleration: gravity included, as a real IMU reads
};

static_assert(sizeof(ImuSampleRecord) == 48, "wire layout changed; bump kSensorFrameVersion");

// The pelvis twist in the body frame; the header's sensor_pos/sensor_quat are the pelvis pose in
// the world, from the site the robot's IMU sits on.
struct BaseStateRecord
{
    double lin_vel[3];  // m/s along the body axes
    double ang_vel[3];  // rad/s about the body axes
};

static_assert(sizeof(BaseStateRecord) == 48, "wire layout changed; bump kSensorFrameVersion");

// One tracked body's ground-truth pose in the MuJoCo world frame. Fixed-size, so the payload is
// a flat array validated by length; a body name that does not fit is refused at startup.
struct ObjectPoseRecord
{
    char   name[32];  // always NUL-terminated, so at most 31 characters
    double pos[3];
    double quat[4];  // wxyz, MuJoCo's own order

    // Axis-aligned extents in the body frame, full widths as vision_msgs/BoundingBox3D means,
    // so consumers need no table of object sizes.
    double size[3];
};

static_assert(sizeof(ObjectPoseRecord) == 112, "wire layout changed; bump kSensorFrameVersion");

// Fixed-size header, then `payload_bytes` of body, so the stream frames without parsing and a
// short read is detectable.
struct SensorFrameHeader
{
    uint32_t magic;          // kSensorFrameMagic
    uint32_t version;        // kSensorFrameVersion
    uint32_t kind;           // SensorFrameKind
    uint32_t payload_bytes;  // body length that follows this header

    // Sim time of the snapshot, for provenance; the relay stamps with its own clock.
    double sim_time_s;

    // Sensor pose in the world at snapshot time, position and wxyz quaternion. Zero and
    // identity on an ObjectPoses frame, whose records carry world poses.
    double sensor_pos[3];
    double sensor_quat[4];

    // PointCloud: number of points, each x, y, z floats in the sensor frame. ObjectPoses
    // derives its count from payload_bytes instead.
    uint32_t point_count;

    // Depth: image size and the render's vertical field of view, so camera_info cannot drift
    // from the MJCF.
    uint32_t width;
    uint32_t height;
    float    fovy_deg;

    // Depth: bytes of rgb8 colour after the depth floats, or 0. Both come from one render, so
    // they share pose, time and frustum.
    uint32_t rgb_bytes;
};

static_assert(sizeof(SensorFrameHeader) == 104, "wire layout changed; bump kSensorFrameVersion");

// NOLINTEND(readability-identifier-naming,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)

}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_FRAME_H_
