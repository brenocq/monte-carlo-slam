//--------------------------------------------------
// Monte Carlo SLAM
// robotScript.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "robotScript.h"
#include <atta/physics/interface.h>

void RobotScript::update(cmp::Entity entity, float dt) {
    PROFILE();
    _entity = entity;
    _dt = dt;
    _robot = _entity.get<RobotComponent>();

    // Get infrareds
    cmp::Entity infrareds = _entity.getChild(0);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    // Update particle weights
    for (RobotComponent::Particle& particle : _robot->particles)
        updateParticleWeight(particle);

    // TODO Move

    // TODO Move particles
    for (RobotComponent::Particle& particle : _robot->particles)
        moveParticle(particle);

    // TODO Update belief
}

void RobotScript::updateParticleWeight(RobotComponent::Particle& particle) {
    const float irRange = 5.0f;
    particle.weight = 0.0f;

    for (int i = 0; i < 8; i++) {
        float angle = -i * (M_PI * 2 / 8) + M_PI * 0.5f + particle.ori;
        atta::vec3 begin = particle.pos;
        atta::vec3 end = atta::vec3(particle.pos, 0.0f) + irRange * atta::vec3(cos(angle), sin(angle), 0.0f);
        std::vector<phy::RayCastHit> hits = phy::rayCast(begin, end);

        // Process hits
        float distance = 5.0f;
        for (phy::RayCastHit hit : hits) {
            if (hit.entity == _entity)
                continue;
            if (hit.distance < distance)
                distance = hit.distance;
        }

        // Handle no hit
        if (hits.empty()) {
            // LOG_WARN("RobotScript", "No ray cast hit from $0 to $1", begin, end);
            distance = 0.0f;
        }

        particle.weight += std::abs(distance - _irs[i]);
    }
}

void RobotScript::moveParticle(RobotComponent::Particle& particle) {
    particle.pos.x += cos(particle.ori - M_PI / 2) * 0.1f * _dt;
    particle.pos.y += sin(particle.ori - M_PI / 2) * 0.1f * _dt;
    particle.ori += 0.1f * _dt;
}
