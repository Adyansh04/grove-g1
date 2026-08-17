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

// The enum spellings and C arrays below are the wire layout itself, fixed by the sizeof
// assertions and shared with the simulator, so they are not ours to restyle.
// NOLINTBEGIN(readability-identifier-naming,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)

// Bumped whenever the layout below changes. The relay refuses a frame it does not know
// rather than reinterpreting bytes. v2 added the depth-image fields; v3 appends colour
// to the depth payload; v4 adds the ObjectPoses kind; v5 gives its records a size; v6 adds
// the Imu kind; v7 adds the BaseState kind.
inline constexpr uint32_t kSensorFrameVersion = 7;

inline constexpr uint32_t kSensorFrameMagic = 0x47314C44;  // "G1LD"

enum class SensorFrameKind : uint32_t
{
    PointCloud = 1,
    Depth      = 2,
    // Ground-truth poses of scene bodies, for the sim-only object source. Not a sensor: it
    // rides this socket because it is the only channel out of the simulator process, and it
    // exists for the same reason (the poses live in mjData, which no DDS topic carries).
    ObjectPoses = 3,
    // The IMU inside the Mid360. It rides this socket rather than LowState because
    // unitree_hg::LowState has exactly one imu_state field, matching a robot that has exactly
    // one IMU on its DDS interface -- the Livox unit reports over its own Ethernet link, and
    // on this side the sensor socket is the equivalent private channel.
    Imu = 4,
    // Exact pelvis pose and twist out of mjData, for the sim-only ground-truth odometry
    // source. Like ObjectPoses it is not a sensor: it rides this socket because the socket
    // is the only channel out of the simulator process that does not depend on which
    // middleware ROS happens to be running.
    BaseState = 5,
};

// Rates of change at the sensor's own frame, to go with the pose the header already carries.
//
// The header's sensor_quat/sensor_pos ARE the IMU's attitude and position: a MuJoCo framequat
// on the same site, filled by the same code path as every other frame, so there is no second
// place for the pose to come from and disagree.
struct ImuSampleRecord
{
    double gyro[3];  // rad/s about the sensor's own axes
    double acc[3];   // m/s^2, PROPER acceleration -- gravity included, as a real IMU reads
};

static_assert(sizeof(ImuSampleRecord) == 48, "wire layout changed; bump kSensorFrameVersion");

// The pelvis twist that goes with the pose in the header, both in the body frame so the relay
// can publish a nav_msgs/Odometry without rotating anything.
//
// The header's sensor_pos/sensor_quat ARE the pelvis pose here, in the world frame, sampled
// from the same MuJoCo site the robot's own IMU sits on.
struct BaseStateRecord
{
    double lin_vel[3];  // m/s along the body axes
    double ang_vel[3];  // rad/s about the body axes
};

static_assert(sizeof(BaseStateRecord) == 48, "wire layout changed; bump kSensorFrameVersion");

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

    // Axis-aligned extents of the body's geometry in its own frame, FULL widths rather than
    // MuJoCo's half-sizes, because that is what vision_msgs/BoundingBox3D means by size.
    //
    // Carried rather than configured downstream: a real 6D-pose detector reports a bounding
    // box too, so a consumer that builds its collision geometry from this keeps working when
    // one replaces this source. Without it every consumer needs its own table of object
    // dimensions, which is a second place for the scene to be described and disagree.
    double size[3];
};

static_assert(sizeof(ObjectPoseRecord) == 112, "wire layout changed; bump kSensorFrameVersion");

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

// NOLINTEND(readability-identifier-naming,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)

}  // namespace grove_g1

#endif  // GROVE_G1_SENSOR_FRAME_H_
