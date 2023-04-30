//--------------------------------------------------
// Monte Carlo SLAM
// robotScript.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "robotScript.h"

void RobotScript::update(cmp::Entity entity, float dt) {
    PROFILE();
    _entity = entity;
    _dt = dt;

    // Get infrareds
    cmp::Entity infrareds = _entity.getChild(0);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;
    LOG_DEBUG("RobotScript", "run $0", entity.getId());
}
