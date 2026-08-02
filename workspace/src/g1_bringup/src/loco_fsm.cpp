/**
 * @file loco_fsm.cpp
 * @brief Sim-only LocoClient FSM legality table.
 */
#include "g1_bringup/loco_fsm.hpp"

namespace g1_bringup
{

namespace
{
// Only Damp/StandUp/Start ever appear on the left of a legal edge (see the header's table) -- a
// current value outside that set (Squat/Sit/ZeroTorque, etc.) falls through to the switch's
// default, no need to name it here.
bool isLegalSetFsmIdEdge(int current, int requested)
{
    switch (current)
    {
        case kFsmDamp:
            return requested == kFsmStandUp;
        case kFsmStandUp:
            return requested == kFsmStart || requested == kFsmDamp;
        case kFsmStart:
            return requested == kFsmStandUp || requested == kFsmDamp;
        default:
            return false;
    }
}
}  // namespace

SetFsmIdResult applySetFsmId(int current, int requested)
{
    if (!isLegalSetFsmIdEdge(current, requested))
    {
        return SetFsmIdResult{ current, kLocoFsmCodeInvalidFsmId };
    }
    return SetFsmIdResult{ requested, kLocoFsmCodeSuccess };
}

std::int32_t checkVelocityAllowed(int current)
{
    return current == kFsmStart ? kLocoFsmCodeSuccess : kLocoFsmCodeLocoStateNotAvailable;
}

}  // namespace g1_bringup
