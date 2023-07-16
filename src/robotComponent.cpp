//--------------------------------------------------
// Monte Carlo SLAM
// robotComponent.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "robotComponent.h"

template <>
cmp::ComponentDescription& cmp::TypedRegistry<RobotComponent>::getDescription() {
    static cmp::ComponentDescription desc = {
        "RobotComponent",
        {
            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, particles), "particles"},
            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, pos), "pos"},
            {AttributeType::FLOAT32, offsetof(RobotComponent, ori), "ori"},
            {AttributeType::CUSTOM, offsetof(RobotComponent, path), "path"},
            {AttributeType::CUSTOM, offsetof(RobotComponent, grid), "grid"},
            {AttributeType::CUSTOM, offsetof(RobotComponent, collisionGrid), "collisionGrid"},
        },
        // Max instances
        1024,
        // Serialize
        {{"path", [](std::ostream& os, void* data) {}},
         {"grid", [](std::ostream& os, void* data) {}},
         {"collisionGrid", [](std::ostream& os, void* data) {}}},
        // Deserialize
        {{"path", [](std::istream& is, void* data) {}},
         {"grid", [](std::istream& is, void* data) {}},
         {"collisionGrid", [](std::istream& is, void* data) {}}},
    };

    return desc;
}
