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
    using Grid = std::array<bool, size>;
    Grid grid; ///< Occupancy grid

    bool getCell(int x, int y);
    bool getCell(atta::vec2i pos);
    float calcDistance(atta::vec2 pos, float angle);
};

ATTA_REGISTER_COMPONENT(MapComponent);

//template <>
//cmp::ComponentDescription& cmp::TypedComponentRegistry<MapComponent>::getDescription();

#endif // MAP_COMPONENT_H
