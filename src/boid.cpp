#include "boid.hpp"
#include "enemy_boid.hpp"
#include "behavior_tree.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

boid::boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin)
    : arrive(new Arrive()), align(new Align())
{
    window = w;
    kinematic = boidKin;
    speed = 1.0f;
    drop_timer = 100.f;
    crumb_idx = 0;
    target_idx = 0;
    sprite.setTexture(tex);
    sprite.setScale(0.02f, 0.02f);
    breadcrumbs = crumbs;
    sprite.setPosition(kinematic.position);
    
    // Initialize decision tree
    buildDecisionTree();
}

boid::~boid() {
    delete arrive;
    delete align;
    delete decisionTree;
}

// Other methods remain unchanged...

// Build the decision tree for intelligent behavior
void boid::buildDecisionTree() {
    // ...existing code...

    // Add path deviation check
    auto isPathBlocked = [](const Kinematic& k, const Kinematic& t,
                           const Kinematic& /*e*/,
                           const std::vector<std::vector<int>>& mapData,
                           int tileSize) -> bool {
        // Check if there's a wall between current position and target
        sf::Vector2f direction = t.position - k.position;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        sf::Vector2f normalized = direction / distance;

        // Check several points along the path
        for (float d = 0; d < distance; d += tileSize / 2.0f) {
            sf::Vector2f checkPos = k.position + normalized * d;
            int checkX = static_cast<int>(checkPos.x / tileSize);
            int checkY = static_cast<int>(checkPos.y / tileSize);

            if (checkX >= 0 && static_cast<size_t>(checkX) < mapData[0].size() &&
                checkY >= 0 && static_cast<size_t>(checkY) < mapData.size() &&
                mapData[checkY][checkX] == 1) {
                return true; // Path is blocked
            }
        }
        return false;
    };

    auto pathDeviationNode = new PathDeviationNode();
    auto normalPathNode = new ActionNode([this](const Kinematic& k, const Kinematic& t,
                                              const Kinematic& e,
                                              const std::vector<std::vector<int>>& map,
                                              int size) -> sf::Vector2f {
        // Normal path following behavior
        return arrive->calculate(k, t);
    });

    auto pathChoice = new DecisionBranch(isPathBlocked, pathDeviationNode, normalPathNode);

    // Add to your existing behavior tree
    root = pathChoice;
}
// Helper functions implementation
float boid::length(const sf::Vector2f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

sf::Vector2f boid::normalize(const sf::Vector2f& v) {
    float len = length(v);
    if (len > 0) {
        return sf::Vector2f(v.x / len, v.y / len);
    }
    return v;
}

sf::Vector2f boid::getPosition() const {
    return kinematic.position;
}

void boid::move(float dt) {
    // Update position based on velocity
    kinematic.position += kinematic.velocity * dt;
    
    // Update orientation based on velocity
    if (length(kinematic.velocity) > 0.1f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
}

void boid::draw() {
    sprite.setPosition(kinematic.position);
    sprite.setRotation(kinematic.orientation * 180.0f / M_PI);
    window->draw(sprite);
}

void boid::update(float dt, const Kinematic& targetKinematic) {
    // Get normal steering behavior
    sf::Vector2f steering = arrive->calculate(kinematic, targetKinematic);
    
    // Add boundary avoidance
    sf::Vector2f boundaryForce = avoidBoundary();
    steering += boundaryForce;
    
    // Apply steering
    kinematic.velocity += steering * dt;
    
    // Clamp position to window bounds
    kinematic.position.x = std::clamp(kinematic.position.x, 0.0f, WINDOW_WIDTH);
    kinematic.position.y = std::clamp(kinematic.position.y, 0.0f, WINDOW_HEIGHT);
    
    // Limit speed
    float speed = length(kinematic.velocity);
    if (speed > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }
}

void boid::updateWithEnemy(float dt, Kinematic targetKin, const Kinematic& enemyKin,
                          const std::vector<std::vector<int>>& mapData, int tileSize) {
    // Execute decision tree to get steering behavior
    sf::Vector2f steering = decisionTree->makeDecision(kinematic, targetKin, enemyKin, mapData, tileSize);
    
    // Apply steering
    kinematic.velocity += steering * dt;
    
    // Limit speed
    if (length(kinematic.velocity) > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }
    
    move(dt);
}