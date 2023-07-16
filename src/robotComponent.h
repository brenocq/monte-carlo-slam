//--------------------------------------------------
// Monte Carlo SLAM
// robotComponent.h
// Date: 2023-04-30
//--------------------------------------------------
#ifndef ROBOT_COMPONENT_H
#define ROBOT_COMPONENT_H
#include <atta/component/interface.h>
#include <queue>

struct RobotComponent final : public atta::component::Component {
    //---------- Monte Carlo ----------//
    struct Particle {
        atta::vec2 pos;
        float ori;
        float weight;
    };

    static constexpr uint32_t numParticles = 1000;
    std::array<Particle, numParticles> particles; // Monte Carlo particles

    atta::vec2 pos; ///< Estimated position
    float ori;      ///< Estimated orientation

    //---------- A* ----------//
    std::queue<atta::vec2> path; ///< Path from estimated position to goal

    //---------- Map ----------//
    static constexpr uint32_t width = 100;
    static constexpr uint32_t height = 100;
    static constexpr uint32_t size = width * height;
    static constexpr float cellSize = 0.10f; // Each cell is 10cm by 10cm
    using Grid = std::array<bool, size>;
    Grid grid;          ///< Occupancy grid
    Grid collisionGrid; ///< Collision grid. If false, robot can be in any position inside the cell
};

ATTA_REGISTER_COMPONENT(RobotComponent);

template <>
cmp::ComponentDescription& cmp::TypedRegistry<RobotComponent>::getDescription();

#endif // ROBOT_COMPONENT_H
