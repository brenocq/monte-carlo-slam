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
    float maxX = RobotComponent::width * RobotComponent::cellSize;
    float maxY = RobotComponent::height * RobotComponent::cellSize;
    atta::vec2 center = atta::vec2(maxX * 0.5f, maxY * 0.5f);
    r->pos = center;
    r->ori = 0.0f;
    for (RobotComponent::Particle& p : r->particles) {
        p.pos = atta::vec2(rand() / float(RAND_MAX) * (maxX - 1.0f), rand() / float(RAND_MAX) * (maxY - 1.0f)) + 0.5f;
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
