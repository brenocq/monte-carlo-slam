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
    //---------- State ----------//
    enum State : uint32_t { MAPPING, LOCALIZATION };
    State state = MAPPING;

    //---------- Monte Carlo ----------//
    struct Particle {
        atta::vec2 pos;
        float ori;
        float weight;
    };

    atta::vec2 pos; ///< Estimated position
    float ori;      ///< Estimated orientation

    static constexpr uint32_t numParticles = 1000;
    std::array<Particle, numParticles> particles; // Monte Carlo particles

    //---------- A* ----------//
    std::queue<atta::vec2> path; ///< Path from estimated position to goal

    //---------- Map ----------//
    enum GridState : uint32_t {
        UNKNOWN = 0, FREE, COLLISION, WALL
    };
    static constexpr uint32_t width = 100;
    static constexpr uint32_t height = 100;
    static constexpr uint32_t size = width * height;
    static constexpr float cellSize = 0.05f; // Each cell is 5cm by 5cm
    using Grid = std::array<GridState, size>;
    Grid grid; ///< Occupancy grid
};

ATTA_REGISTER_COMPONENT(RobotComponent);

template <>
cmp::ComponentDescription& cmp::TypedRegistry<RobotComponent>::getDescription();

#endif // ROBOT_COMPONENT_H
