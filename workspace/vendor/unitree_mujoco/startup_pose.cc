#include "startup_pose.h"

#include <cstdio>
#include <string_view>

namespace grove_g1
{
namespace
{

constexpr std::string_view kStartupHoldPrefix = "startup_hold_";

}  // namespace

void ApplyStartupPose(const mjModel* model, mjData* data)
{
    if (model == nullptr || data == nullptr)
    {
        return;
    }
    int applied = 0;
    for (int eq = 0; eq < model->neq; ++eq)
    {
        // Single-joint equalities only. mjEQ_JOINT with no second joint means
        // qpos - qpos0 == data[0], which is the angle the scene is asking for.
        if (model->eq_type[eq] != mjEQ_JOINT || model->eq_obj2id[eq] >= 0)
        {
            continue;
        }
        const char* name = mj_id2name(model, mjOBJ_EQUALITY, eq);
        if (name == nullptr || std::string_view(name).compare(0, kStartupHoldPrefix.size(),
                                                              kStartupHoldPrefix) != 0)
        {
            continue;
        }
        const int joint = model->eq_obj1id[eq];
        if (joint < 0)
        {
            continue;
        }
        const int adr   = model->jnt_qposadr[joint];
        data->qpos[adr] = model->qpos0[adr] + model->eq_data[eq * mjNEQDATA];
        ++applied;
    }
    if (applied > 0)
    {
        std::fprintf(
            stderr, "[grove_g1] startup pose: %d joints spawned at their held angle\n", applied);
    }
}

}  // namespace grove_g1
