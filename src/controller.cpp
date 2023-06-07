//--------------------------------------------------
// Monte Carlo SLAM
// controller.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "controller.h"
#include <atta/component/components/infraredSensor.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>
#include <atta/physics/interface.h>
#include <atta/processor/interface.h>

void Controller::update() {
    PROFILE();
    _robot = entity.get<RobotComponent>();

    // Get infrareds
    cmp::Entity infrareds = entity.getChild(0);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    // Update particles with IR measurements
    particlesUpdate();

    // Calculate control given measurements and estimated state
    atta::vec2 control = calcControl();

    // Move robot
    move(control);

    // Predict new particles given control
    particlesPredict(control);
}

atta::vec2 Controller::calcControl() {
    atta::vec2 dir = atta::vec2(1.0f, 0.0f);

    const float maxDist = 1.0f;
    for (int i = 0; i < 8; i++) {
        float ir = _irs[i];
        float angle = M_PI / 4 * i;
        if (ir < maxDist)
            dir -= (1.0f - ir / maxDist) * atta::vec2(cos(angle), sin(angle));
    }

    return dir;
}

void Controller::move(atta::vec2 control) {
    constexpr float wheelD = 0.06f; // Wheel distance
    constexpr float wheelR = 0.01f; // Wheel radius
    constexpr float maxPwr = 5.0f;  // Motor maximum power (max 0.5m/s)

    auto r = entity.get<cmp::RigidBody2D>();

    // If break motors
    if (control.x == 0.0f && control.y == 0.0f) {
        r->setLinearVelocity(atta::vec2(0.0f));
        r->setAngularVelocity(0.0f);
        return;
    }

    // Calculate motor velocities
    control.normalize();
    float dirAngle = atan2(control.y, control.x);                                 // Direction angle
    atta::vec2 pwr(cos(dirAngle) - sin(dirAngle), cos(dirAngle) + sin(dirAngle)); // Power
    pwr.normalize();
    pwr *= maxPwr;

    // Calculate linear/angular velocities (differential drive robot)
    float linVel = wheelR / 2.0f * (pwr.x + pwr.y);
    float angVel = wheelR / wheelD * (pwr.x - pwr.y);

    // Apply velocities
    float angle = entity.get<cmp::Transform>()->orientation.get2DAngle();
    r->setLinearVelocity(atta::vec2(linVel * cos(angle), linVel * sin(angle)));
    r->setAngularVelocity(angVel);
}

#define RANDOM(x) ((rand() / float(RAND_MAX) - 0.5f) * x)

void Controller::particlesPredict(atta::vec2 control) {
    const float speed = 0.03535;
    for (RobotComponent::Particle& particle : _robot->particles) {
        float dt = atta::processor::getDt();

        // TODO incorporate control
        particle.ori += RANDOM(0.5f);

        particle.pos.x += cos(particle.ori) * speed * dt;
        particle.pos.y += sin(particle.ori) * speed * dt;
        particle.pos.x += RANDOM(0.05f);
        particle.pos.y += RANDOM(0.05f);
    }
}

void Controller::particlesUpdate() {
    // Update particle weights
    float sumWeights = 0.0f;
    for (RobotComponent::Particle& particle : _robot->particles) {
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
        // Normalize weight between 0 and 1
        particle.weight = 1 / (1 + particle.weight);
        sumWeights += particle.weight;
    }

    // Normalize particles
    for (RobotComponent::Particle& particle : _robot->particles)
        particle.weight /= sumWeights;

    // Resampling
    std::array<RobotComponent::Particle, RobotComponent::numParticles> newParticles;
    for (int i = 0; i < RobotComponent::numParticles; i++) {
        float r = rand() / float(RAND_MAX);
        float sum = 0.0f;
        // Choose particle proportional to their heights
        for (int j = 0; j < RobotComponent::numParticles; j++) {
            sum += _robot->particles[j].weight;
            if (sum >= r) {
                newParticles[i] = _robot->particles[j];
                break;
            }
        }
    }
    _robot->particles = newParticles;

    // TODO Estimate robot state from particles
}
