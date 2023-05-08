//--------------------------------------------------
// Monte Carlo SLAM
// robotScript.h
// Date: 2023-04-30
//--------------------------------------------------
#ifndef ROBOT_SCRIPT_H
#define ROBOT_SCRIPT_H
#include "robotComponent.h"
#include <atta/component/components/components.h>
#include <atta/script/script.h>

class RobotScript : public scr::Script {
  public:
    void update(cmp::Entity entity, float dt) override;

  private:
    void updateParticleWeight(RobotComponent::Particle& particle);
    void moveParticle(RobotComponent::Particle& particle);

    cmp::Entity _entity;
    float _dt;

    RobotComponent* _robot;
    std::array<float, 8> _irs;
};

ATTA_REGISTER_SCRIPT(RobotScript)

#endif // ROBOT_SCRIPT_H
