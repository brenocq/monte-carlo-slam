//--------------------------------------------------
// Monte Carlo SLAM
// world.h
// Date: 2023-06-05
//--------------------------------------------------
#ifndef WORLD_H
#define WORLD_H
#include <atta/script/registry/worldRegistry.h>
#include <atta/script/scripts/world.h>

class World : public atta::script::World {
  public:
    void onStart() override;
    void onStop() override;
    void onUpdateBefore() override;
    void onUpdateAfter() override;
};

#ifndef __NVCC__
ATTA_REGISTER_WORLD(World)
#endif

#endif // WORLD_H
