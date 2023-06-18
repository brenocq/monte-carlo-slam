//--------------------------------------------------
// Monte Carlo SLAM
// mapComponent.h
// Date: 2023-05-08
//--------------------------------------------------
#ifndef MAP_COMPONENT_H
#define MAP_COMPONENT_H
#include <atta/component/interface.h>

struct MapComponent final : public atta::component::Component {
    static constexpr uint32_t width = 50;
    static constexpr uint32_t height = 50;
    static constexpr uint32_t size = width * height;
    static constexpr float cellSize = 0.10f; // Each cell is 10cm by 10cm
    using Grid = std::array<bool, size>;
    Grid grid;          ///< Occupancy grid
    Grid collisionGrid; ///< Collision grid. If false, robot can be in any position inside the cell
};

ATTA_REGISTER_COMPONENT(MapComponent);

template <>
cmp::ComponentDescription& cmp::TypedRegistry<MapComponent>::getDescription();

#endif // MAP_COMPONENT_H
