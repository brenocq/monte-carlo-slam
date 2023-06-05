//--------------------------------------------------
// Monte Carlo SLAM
// controller.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "controller.h"
#include <atta/component/components/infraredSensor.h>
#include <atta/physics/interface.h>
#include <atta/processor/interface.h>

void Controller::update() {
    PROFILE();
    _robot = entity.get<RobotComponent>();

    // Get infrareds
    cmp::Entity infrareds = entity.getChild(0);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    // Update particle weights
    for (RobotComponent::Particle& particle : _robot->particles) {
        updateParticleWeight(particle);
    }

    // TODO Move

    // TODO Move particles
    for (RobotComponent::Particle& particle : _robot->particles)
        moveParticle(particle);

    // TODO Update belief
}

void Controller::updateParticleWeight(RobotComponent::Particle& particle) {
    particle.weight = 0.0f;

    for (int i = 0; i < 8; i++) {
        const float irRange = entity.getChild(0).getChild(i).get<cmp::InfraredSensor>()->upperLimit;

        // Cast ray
        float angle = -i * (M_PI * 2 / 8) + M_PI * 0.5f + particle.ori;
        atta::vec3 begin = particle.pos;
        atta::vec3 end = atta::vec3(particle.pos, 0.0f) + irRange * atta::vec3(cos(angle), sin(angle), 0.0f);
        std::vector<phy::RayCastHit> hits = phy::rayCast(begin, end);

        // Process hits
        float distance = irRange;
        for (phy::RayCastHit hit : hits) {
            if (hit.entity == entity)
                continue;
            if (hit.distance < distance)
                distance = hit.distance;
        }

        particle.weight += std::abs(distance - _irs[i]);
    }
}

void Controller::moveParticle(RobotComponent::Particle& particle) {
    float dt = atta::processor::getDt();
    particle.ori += 0.1f * dt;
    particle.pos.x += cos(particle.ori) * 0.1f * dt;
    particle.pos.y += sin(particle.ori) * 0.1f * dt;
}
