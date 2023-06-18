//--------------------------------------------------
// Monte Carlo SLAM
// controller.cpp
// Date: 2023-04-30
//--------------------------------------------------
#include "controller.h"
#include "mapComponent.h"
#include <atta/component/components/infraredSensor.h>
#include <atta/component/components/rigidBody2D.h>
#include <atta/component/components/transform.h>
#include <atta/physics/interface.h>
#include <atta/processor/interface.h>

const cmp::Entity map(17);
const cmp::Entity goal(150);

void Controller::update() {
    PROFILE();
    _robot = entity.get<RobotComponent>();

    // Get infrareds
    cmp::Entity infrareds = entity.getChild(0);
    for (int i = 0; i < 8; i++)
        _irs[i] = infrareds.getChild(i).get<cmp::InfraredSensor>()->measurement;

    // Update particles and estimate current state
    particlesUpdate();

    // Generate path given current state and goal
    updateAStar();

    // Calculate control given next path goal
    atta::vec2 control = calcControl();

    // Move robot
    move(control);

    // Predict new particles given control
    particlesPredict(control);
}

struct Node {
    atta::vec2i position;
    float fScore;
    float gScore;
    Node* parent;

    Node(const atta::vec2i& pos, float f, float g, Node* par) : position(pos), fScore(f), gScore(g), parent(par) {}
};

std::queue<atta::vec2> findPathAStar(const atta::vec2& start, const atta::vec2& end, const MapComponent::Grid& grid) {
    const int width = MapComponent::width;
    const int height = MapComponent::height;
    const float cellSize = MapComponent::cellSize;
    const atta::vec2i starti = start / cellSize;
    const atta::vec2i endi = end / cellSize;

    // Initialize the open and closed sets
    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;
    Node* startNode = new Node(starti, 0.0f, 0.0f, nullptr);
    openSet.push_back(startNode);

    // Create a helper function to calculate the heuristic (estimated) distance between two positions
    auto heuristic = [](const atta::vec2i& pos1, const atta::vec2i& pos2) { return (pos2 - pos1).length(); };

    // Create a helper function to check if a position is within the map boundaries
    auto isWithinMapBounds = [](const atta::vec2i& pos) { return pos.x >= 0 && pos.x < width && pos.y >= 0 && pos.y < height; };

    // Create a helper function to check if a position is blocked (occupied) in the collision grid
    auto isBlocked = [&grid](const atta::vec2i& pos) { return grid[pos.y * width + pos.x]; };

    // Perform the A* algorithm
    while (!openSet.empty()) {
        // Find the node with the lowest fScore in the openSet
        Node* current = openSet[0];
        int currentIndex = 0;
        for (int i = 1; i < openSet.size(); ++i) {
            if (openSet[i]->fScore < current->fScore) {
                current = openSet[i];
                currentIndex = i;
            }
        }

        // Check if the current node is the goal
        if (current->position == endi) {
            std::queue<atta::vec2> path;

            //----- Reconstruct the path -----//
            // TODO optimize diagonal
            while (current != nullptr) {
                path.push(atta::vec2(current->position.x, current->position.y) * cellSize);
                current = current->parent;
            }

            //----- Reverse path -----//
            std::stack<atta::vec2> auxStack;
            // Transfer from queue to stack
            while (!path.empty()) {
                auxStack.push(path.front());
                path.pop();
            }
            // Transfer from stack to queue (reversed)
            while (!auxStack.empty()) {
                path.push(auxStack.top());
                auxStack.pop();
            }

            //----- Clean up memory -----//
            for (Node* node : openSet)
                delete node;
            for (Node* node : closedSet)
                delete node;

            return path;
        }

        // Move the current node from openSet to closedSet
        openSet.erase(openSet.begin() + currentIndex);
        closedSet.push_back(current);

        // Generate neighbor positions
        const std::vector<atta::vec2i> neighbors = {
            atta::vec2(-1, 0), // Left
            atta::vec2(1, 0),  // Right
            atta::vec2(0, -1), // Down
            atta::vec2(0, 1)   // Up
        };

        for (const atta::vec2i& offset : neighbors) {
            atta::vec2i neighborPos = current->position + offset;

            // Skip if the neighbor position is outside the map boundaries or blocked
            if (!isWithinMapBounds(neighborPos) || isBlocked(neighborPos))
                continue;

            // Calculate the tentative gScore for the neighbor
            float tentativeGScore = current->gScore + offset.length();

            // Check if the neighbor is already evaluated (in the closedSet) and has a higher gScore
            bool skipNeighbor = false;
            for (Node* node : closedSet) {
                if (node->position == neighborPos && tentativeGScore >= node->gScore) {
                    skipNeighbor = true;
                    break;
                }
            }

            if (skipNeighbor)
                continue;

            // Check if the neighbor is in the openSet and has a higher gScore
            Node* neighbor = nullptr;
            for (Node* node : openSet) {
                if (node->position == neighborPos) {
                    neighbor = node;
                    break;
                }
            }

            if (neighbor == nullptr || tentativeGScore < neighbor->gScore) {
                // Create a new neighbor node or update the existing one
                if (neighbor == nullptr) {
                    neighbor = new Node(neighborPos, 0.0f, 0.0f, nullptr);
                    openSet.push_back(neighbor);
                }

                neighbor->gScore = tentativeGScore;
                neighbor->fScore = tentativeGScore + heuristic(neighborPos, endi);
                neighbor->parent = current;
            }
        }
    }

    // No path found, clean up memory and return an empty queue
    for (Node* node : openSet)
        delete node;
    for (Node* node : closedSet)
        delete node;

    return std::queue<atta::vec2>();
}

void Controller::updateAStar() {
    atta::vec2 start = _robot->pos;
    atta::vec2 end = atta::vec2(goal.get<cmp::Transform>()->position);

    if (_robot->path.empty() ||                                              // If empty
        (_robot->path.back() - end).length() > 2 * MapComponent::cellSize || // End too far
        (_robot->path.front() - start).length() > 2 * MapComponent::cellSize // Start too far
    ) {
        _robot->path = findPathAStar(start, end, map.get<MapComponent>()->collisionGrid);
    }
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
        particle.ori += atta::random::normal(0.0f, 0.03f);
        particle.pos.x += cos(particle.ori) * speed * dt + atta::random::normal(0.0f, 0.002f);
        particle.pos.y += sin(particle.ori) * speed * dt + atta::random::normal(0.0f, 0.002f);
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
            float angle = -i * (M_PI * 2 / 8) + particle.ori;
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

    // Estimate robot state from particles (best particle method)
    RobotComponent::Particle bestParticle = _robot->particles[0];
    for (const RobotComponent::Particle& particle : _robot->particles) {
        if (particle.weight > bestParticle.weight)
            bestParticle = particle;
    }
    _robot->pos = bestParticle.pos;
    _robot->ori = bestParticle.ori;

    // Normalize particles
    for (RobotComponent::Particle& particle : _robot->particles)
        particle.weight /= sumWeights;

    // Particle resampling
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
}
