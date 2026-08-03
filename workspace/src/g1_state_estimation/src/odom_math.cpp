#include "g1_state_estimation/odom_math.hpp"

#include <cmath>
#include <string>

namespace g1_state_estimation
{

bool parseOdometrySource(const std::string& name, OdometrySource& out)
{
    if (name == "sim_ground_truth")
    {
        out = OdometrySource::SimGroundTruth;
        return true;
    }
    if (name == "sim_sportmodestate")
    {
        out = OdometrySource::SimSportModeState;
        return true;
    }
    if (name == "hardware")
    {
        out = OdometrySource::Hardware;
        return true;
    }
    return false;
}

Quaternion yawToQuaternion(double yaw)
{
    Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

double quaternionToYaw(const Quaternion& q) { return wrapAngle(2.0 * std::atan2(q.z, q.w)); }

double wrapAngle(double angle)
{
    // remainder() lands in [-pi, pi]; the shift moves the -pi endpoint up so the interval
    // is half-open and a given rotation has exactly one representation.
    const double wrapped = std::remainder(angle, 2.0 * M_PI);
    return (wrapped <= -M_PI) ? wrapped + 2.0 * M_PI : wrapped;
}

PlanarTwist toBodyTwist(const PlanarTwist& world_twist, double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    PlanarTwist  body;
    body.vx    = world_twist.vx * c + world_twist.vy * s;
    body.vy    = -world_twist.vx * s + world_twist.vy * c;
    body.omega = world_twist.omega;
    return body;
}

bool isStale(double elapsed_s, double timeout_s)
{
    if (timeout_s <= 0.0)
    {
        return false;
    }
    return elapsed_s > timeout_s;
}

std::array<double, 36> diagonalCovariance(double value)
{
    std::array<double, 36> covariance{};
    for (std::size_t i = 0; i < 6; ++i)
    {
        covariance[i * 6 + i] = value;
    }
    return covariance;
}

}  // namespace g1_state_estimation
