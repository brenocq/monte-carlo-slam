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
