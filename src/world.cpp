//--------------------------------------------------
// Monte Carlo SLAM
// world.cpp
// Date: 2023-06-05
//--------------------------------------------------
#include "world.h"
#include "robotComponent.h"

const cmp::Entity robot(7);

void World::onStart() {
    RobotComponent* r = robot.get<RobotComponent>();
    int w = RobotComponent::width;
    int h = RobotComponent::height;
    float maxX = w * RobotComponent::cellSize;
    float maxY = h * RobotComponent::cellSize;
    atta::vec2 center = atta::vec2(maxX * 0.5f, maxY * 0.5f);
    // Initialize estimation
    r->pos = center;
    r->ori = 0.0f;
    // Initialize particles
    for (RobotComponent::Particle& p : r->particles) {
        p.pos = atta::vec2(rand() / float(RAND_MAX) * (maxX - 1.0f), rand() / float(RAND_MAX) * (maxY - 1.0f)) + 0.5f;
        p.ori = rand() / float(RAND_MAX) * M_PI * 2;
    }
    // Initialize grid
    for (int i = 0; i < w * h; i++)
        r->grid[i] = RobotComponent::UNKNOWN;
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
