#include "g1_hardware_interface/lowstate_joint_states.hpp"

#include <cassert>

namespace g1_hardware_interface
{

void initLowerBodyJointState(sensor_msgs::msg::JointState& msg)
{
    msg.name.clear();
    msg.name.reserve(kNumLowerMotors);
    for (const char* name : kLowerMotorJointNames)
    {
        msg.name.emplace_back(name);
    }
    msg.position.assign(kNumLowerMotors, 0.0);
    msg.velocity.assign(kNumLowerMotors, 0.0);
}

void fillLowerBodyJointState(
    const unitree_hg::msg::LowState& state, sensor_msgs::msg::JointState& msg)
{
    assert(msg.position.size() == kNumLowerMotors && msg.velocity.size() == kNumLowerMotors);
    for (std::size_t i = 0; i < kNumLowerMotors; ++i)
    {
        msg.position[i] = state.motor_state[i].q;
        msg.velocity[i] = state.motor_state[i].dq;
    }
}

}  // namespace g1_hardware_interface
