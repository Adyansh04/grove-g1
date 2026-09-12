#include "g1_manipulation/grasp_filter.hpp"

#include <cmath>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace g1_manipulation
{
namespace
{

tf2::Transform asTransform(const geometry_msgs::msg::Pose& pose)
{
    tf2::Transform out;
    tf2::fromMsg(pose, out);
    return out;
}

}  // namespace

geometry_msgs::msg::Pose applyGripperOffset(
    const geometry_msgs::msg::Pose& generated, const std::array<double, 6>& xyz_rpy, bool is_left)
{
    const double    mirror = is_left ? -1.0 : 1.0;
    tf2::Quaternion rotation;
    rotation.setRPY(mirror * xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);
    const tf2::Transform offset(
        rotation,
        tf2::Vector3(xyz_rpy[0], mirror * xyz_rpy[1], mirror * xyz_rpy[2]));

    geometry_msgs::msg::Pose out;
    tf2::toMsg(asTransform(generated) * offset, out);
    return out;
}

std::array<double, 3> approachAxis(const geometry_msgs::msg::Pose& generated)
{
    const tf2::Matrix3x3 rotation(asTransform(generated).getRotation());
    // Third column: where the pose's own +z points once it is expressed in the outer frame.
    return { rotation[0][2], rotation[1][2], rotation[2][2] };
}

double approachTiltRad(const geometry_msgs::msg::Pose& generated)
{
    const std::array<double, 3> axis = approachAxis(generated);
    // Against straight down, so a grasp reaching from above scores zero however it is yawed.
    return std::acos(std::clamp(-axis[2], -1.0, 1.0));
}

}  // namespace g1_manipulation
