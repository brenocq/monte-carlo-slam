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
    void updateParticleWeight(RobotComponent::Particle& particle);
    void moveParticle(RobotComponent::Particle& particle);

    RobotComponent* _robot;
    std::array<float, 8> _irs;
};

#ifndef __NVCC__
ATTA_REGISTER_CONTROLLER(Controller)
#endif

#endif // CONTROLLER_H
