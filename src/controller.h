//--------------------------------------------------
// Monte Carlo SLAM
// controller.h
// Date: 2023-04-30
//--------------------------------------------------
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "robotComponent.h"
#include <atta/script/registry/controllerRegistry.h>
#include <atta/script/scripts/controller.h>

class Controller : public atta::script::Controller {
  public:
    void update() override;

  private:
    // Map
    void updateGrid();
    void calculateCollision();

    // A*
    void updateAStar();
    void processControl(atta::vec2 control, float* linVel, float* angVel);
    void move(atta::vec2 control);

    // Monte Carlo
    void particlesUpdate();
    void particlesPredict(atta::vec2 control);

    RobotComponent* _robot;
    std::array<float, 8> _irs;
};

#ifndef __NVCC__
ATTA_REGISTER_CONTROLLER(Controller)
#endif

#endif // CONTROLLER_H
