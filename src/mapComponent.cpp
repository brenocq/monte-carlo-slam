//--------------------------------------------------
// Monte Carlo SLAM
// mapComponent.cpp
// Date: 2023-05-08
//--------------------------------------------------
#include "mapComponent.h"

template <>
cmp::ComponentDescription& cmp::TypedRegistry<MapComponent>::getDescription() {
    static cmp::ComponentDescription desc = {
        "MapComponent",
        {{AttributeType::CUSTOM, offsetof(MapComponent, grid), "grid"}},
        // Max instances
        1024,
    };

    return desc;
}

bool MapComponent::getCell(int x, int y) {
    if (x >= width || y >= height || x < 0 || y < 0)
        return true;
    return grid[y * width + x];
}
bool MapComponent::getCell(atta::vec2i pos) { return getCell(pos.x, pos.y); }

float MapComponent::calcDistance(atta::vec2 pos, float angle) {
    constexpr int cellSize = 10.0f;
    atta::vec2i posi = pos * cellSize;

    // Check position over occupied cell
    if (getCell(posi))
        return 0.0f;

    // TODO Calculate distance to grid cell

    return 1.0f;
}
