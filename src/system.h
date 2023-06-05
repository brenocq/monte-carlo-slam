//--------------------------------------------------
// Monte Carlo SLAM
// system.h
// Date: 2023-04-30
//--------------------------------------------------
#ifndef SYSTEM_H
#define SYSTEM_H
#include <atta/script/registry/systemRegistry.h>
#include <atta/script/scripts/system.h>

class System : public atta::script::System {
  public:
    void onLoad() override;
    void onUnload() override;
    void onAttaLoop() override;
    void onUIRender() override;

  private:
    void loadMap(std::string name);
    void resetMap();
};

#ifndef __NVCC__
ATTA_REGISTER_SYSTEM(System)
#endif

#endif // SYSTEM_H
