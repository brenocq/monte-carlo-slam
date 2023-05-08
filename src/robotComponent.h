//--------------------------------------------------
// Monte Carlo SLAM
// robotComponent.h
// Date: 2023-04-30
//--------------------------------------------------
#ifndef ROBOT_COMPONENT_H
#define ROBOT_COMPONENT_H
#include <atta/component/interface.h>

struct RobotComponent final : public cmp::Component {
    struct Particle {
        atta::vec2 pos;
        float ori;
        float weight;
    };
    static constexpr uint32_t numParticles = 1000;
    std::array<Particle, numParticles> particles; // Monte Carlo particles

    atta::vec2 pos; ///< Estimated position
    float ori;      ///< Estimated orientation
};
ATTA_REGISTER_COMPONENT(RobotComponent);
template <>
cmp::ComponentDescription& cmp::TypedComponentRegistry<RobotComponent>::getDescription();

#endif // ROBOT_COMPONENT_H
