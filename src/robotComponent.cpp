//--------------------------------------------------
// Monte Carlo SLAM
// robotComponent.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "robotComponent.h"

//template <>
//cmp::ComponentDescription& cmp::TypedComponentRegistry<RobotComponent>::getDescription() {
//    static cmp::ComponentDescription desc = {
//        "Robot",
//        {
//            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, particles), "particles"},
//            {AttributeType::VECTOR_FLOAT32, offsetof(RobotComponent, pos), "pos"},
//            {AttributeType::FLOAT32, offsetof(RobotComponent, ori), "ori"},
//        },
//        // Max instances
//        1024,
//    };
//
//    return desc;
//}
