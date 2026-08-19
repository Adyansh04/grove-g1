/**
 * @file odom_math.cpp
 * @brief Frame and staleness math for the odom -> base publisher.
 */

#include "g1_state_estimation/odom_math.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace g1_state_estimation
{

bool parseOdometrySource(const std::string& name, OdometrySource& out)
{
    if (name == "ground_truth")
    {
        out = OdometrySource::kGroundTruth;
        return true;
    }
    if (name == "fast_lio")
    {
        out = OdometrySource::kFastLio;
        return true;
    }
    if (name == "hardware")
    {
        out = OdometrySource::kHardware;
        return true;
    }
    return false;
}

bool isUsablePose(const Pose3d& pose)
{
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z))
    {
        return false;
    }
    const double norm2 =
        pose.q.w * pose.q.w + pose.q.x * pose.q.x + pose.q.y * pose.q.y + pose.q.z * pose.q.z;
    // Same 0.5 floor the IMU path uses: it admits an unnormalised quaternion but rejects the
    // all-zero default and anything that has collapsed toward it.
    return std::isfinite(norm2) && norm2 >= 0.5;
}

namespace
{
Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return Quaternion{ a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                       a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                       a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                       a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

/// q * (0, v) * q^-1, for a unit q.
void rotate(const Quaternion& q, double& x, double& y, double& z)
{
    // t = 2 * (q_vec x v), then v' = v + q.w * t + q_vec x t. Two cross products rather than
    // building a matrix: fewer operations and no intermediate to keep consistent.
    const double tx = 2.0 * (q.y * z - q.z * y);
    const double ty = 2.0 * (q.z * x - q.x * z);
    const double tz = 2.0 * (q.x * y - q.y * x);

    const double rx = x + q.w * tx + q.y * tz - q.z * ty;
    const double ry = y + q.w * ty + q.z * tx - q.x * tz;
    const double rz = z + q.w * tz + q.x * ty - q.y * tx;
    x               = rx;
    y               = ry;
    z               = rz;
}
}  // namespace

Pose3d composePose(const Pose3d& a, const Pose3d& b)
{
    Pose3d out;
    out.x = b.x;
    out.y = b.y;
    out.z = b.z;
    rotate(a.q, out.x, out.y, out.z);
    out.x += a.x;
    out.y += a.y;
    out.z += a.z;
    out.q = multiply(a.q, b.q);
    return out;
}

Pose3d invertPose(const Pose3d& pose)
{
    Pose3d out;
    // Conjugate, which is the inverse for the unit quaternions this deals in.
    out.q = Quaternion{ -pose.q.x, -pose.q.y, -pose.q.z, pose.q.w };
    out.x = -pose.x;
    out.y = -pose.y;
    out.z = -pose.z;
    rotate(out.q, out.x, out.y, out.z);
    return out;
}

Quaternion yawToQuaternion(double yaw)
{
    Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

double quaternionToYaw(const Quaternion& q)
{
    return wrapAngle(std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
}

double tiltFromVertical(const Quaternion& q)
{
    // R22 of the rotation matrix is the body +z projected onto the world +z.
    const double cos_tilt = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return std::acos(std::clamp(cos_tilt, -1.0, 1.0));
}

GroundSplit splitGroundProjection(double x, double y, double z, const Quaternion& q, double yaw)
{
    GroundSplit out;
    out.footprint.x   = x;
    out.footprint.y   = y;
    out.footprint.yaw = yaw;
    // Not a rotated offset: the footprint sits directly beneath the body by construction, so
    // inverting it leaves exactly (0, 0, z). Asserted in test because it reads like a missing
    // term.
    out.child_z = z;

    // Rz(-yaw) * q. A pure -z left operand collapses the Hamilton product to these four terms.
    const double s = std::sin(-yaw * 0.5);
    const double c = std::cos(-yaw * 0.5);
    out.tilt.w     = c * q.w - s * q.z;
    out.tilt.x     = c * q.x - s * q.y;
    out.tilt.y     = c * q.y + s * q.x;
    out.tilt.z     = c * q.z + s * q.w;
    return out;
}

Quaternion composeAttitude(double yaw, const Quaternion& tilt)
{
    // Rz(yaw) * tilt, the exact inverse of the Rz(-yaw) * q that split it.
    return multiply(yawToQuaternion(yaw), tilt);
}

Quaternion composeRotation(const Quaternion& a, const Quaternion& b) { return multiply(a, b); }

Quaternion invertRotation(const Quaternion& q) { return Quaternion{ -q.x, -q.y, -q.z, q.w }; }

Quaternion slerp(const Quaternion& from, const Quaternion& to, double t)
{
    t = std::clamp(t, 0.0, 1.0);

    double dot = from.x * to.x + from.y * to.y + from.z * to.z + from.w * to.w;
    // q and -q are the same rotation; flip so the interpolation takes the short way.
    Quaternion end = to;
    if (dot < 0.0)
    {
        end = Quaternion{ -to.x, -to.y, -to.z, -to.w };
        dot = -dot;
    }

    // Near-parallel: sin(theta) underflows and the general form divides by ~0. Straight lerp
    // is accurate to well past what matters here, and this is the common case: the
    // correction being interpolated is a fraction of a degree.
    double scale_from = 1.0 - t;
    double scale_to   = t;
    if (dot < 0.9995)
    {
        const double theta     = std::acos(std::clamp(dot, -1.0, 1.0));
        const double sin_theta = std::sin(theta);
        scale_from             = std::sin((1.0 - t) * theta) / sin_theta;
        scale_to               = std::sin(t * theta) / sin_theta;
    }

    Quaternion   out{ scale_from * from.x + scale_to * end.x,
                    scale_from * from.y + scale_to * end.y,
                    scale_from * from.z + scale_to * end.z,
                    scale_from * from.w + scale_to * end.w };
    const double norm = std::sqrt(out.w * out.w + out.x * out.x + out.y * out.y + out.z * out.z);
    if (norm < 1e-9)
    {
        return from;
    }
    return Quaternion{ out.x / norm, out.y / norm, out.z / norm, out.w / norm };
}

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
