//--------------------------------------------------
// Monte Carlo SLAM
// world.cpp
// Date: 2023-06-05
//--------------------------------------------------
#include "world.h"
#include "robotComponent.h"

const cmp::Entity robot(7);

void World::onStart() {
    // Initialize particles
    RobotComponent* r = robot.get<RobotComponent>();
    for (RobotComponent::Particle& p : r->particles) {
        p.pos = atta::vec2(rand() / float(RAND_MAX) * 4.0f, rand() / float(RAND_MAX) * 4.0f) + 0.5f;
        p.ori = rand() / float(RAND_MAX) * M_PI * 2;
    }
}

void World::onStop() {
    // Delete particles
    RobotComponent* r = robot.get<RobotComponent>();
    r->path = {};
    for (RobotComponent::Particle& p : r->particles)
        p = {};
}

void World::onUpdateBefore() {}

void World::onUpdateAfter() {}
