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
            {AttributeType::UINT32, offsetof(RobotComponent, state), "state", {}, {}, {}, {"Mapping", "Localization"}},
            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, pos), "pos"},
            {AttributeType::FLOAT32, offsetof(RobotComponent, ori), "ori"},
            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, particles), "particles"},
            {AttributeType::CUSTOM, offsetof(RobotComponent, path), "path"},
            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, goalPos), "goalPos"},
            {AttributeType::CUSTOM, offsetof(RobotComponent, grid), "grid"},
        },
        // Max instances
        1024,
        // Serialize
        {{"path", [](std::ostream& os, void* data) {}},
         {"grid", [](std::ostream& os, void* data) {}}},
        // Deserialize
        {{"path", [](std::istream& is, void* data) {}},
         {"grid", [](std::istream& is, void* data) {}}},
    };

    return desc;
}
