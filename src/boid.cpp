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
    // Create action nodes
    auto fleeEnemyAction = new ActionNode(
        [this](const Kinematic& k, const Kinematic& /*t*/, const Kinematic& e, 
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            sf::Vector2f fleeDirection = k.position - e.position;
            float distance = length(fleeDirection);
            if (distance > 0.1f) {
                fleeDirection = normalize(fleeDirection) * k.maxAcceleration;
            }
            return fleeDirection;
        }
    );

    auto avoidWallAction = new ActionNode(
        [this](const Kinematic& k, const Kinematic& /*t*/, const Kinematic& /*e*/, 
               const std::vector<std::vector<int>>& mapData, int tileSize) -> sf::Vector2f {
            sf::Vector2f futurePos = k.position + normalize(k.velocity) * 50.0f;
            int tileX = static_cast<int>(futurePos.x / tileSize);
            int tileY = static_cast<int>(futurePos.y / tileSize);
            if (tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() && 
                tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() && 
                mapData[tileY][tileX] == 1) {
                sf::Vector2f wallCenter((tileX + 0.5f) * tileSize, (tileY + 0.5f) * tileSize);
                return normalize(k.position - wallCenter) * k.maxAcceleration;
            }
            return sf::Vector2f(0, 0);
        }
    );

    auto seekTargetAction = new ActionNode(
        [this](const Kinematic& k, const Kinematic& t, const Kinematic& /*e*/, 
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            return arrive->calculate(const_cast<Kinematic&>(k), const_cast<Kinematic&>(t));
        }
    );

    // Create condition nodes
    auto isNearEnemy = [this](const Kinematic& k, const Kinematic& /*t*/, const Kinematic& e, 
                             const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> bool {
        return length(k.position - e.position) < 200.0f;
    };

    auto isNearWall = [this](const Kinematic& k, const Kinematic& /*t*/, const Kinematic& /*e*/, 
                            const std::vector<std::vector<int>>& mapData, int tileSize) -> bool {
        sf::Vector2f futurePos = k.position + normalize(k.velocity) * 50.0f;
        int tileX = static_cast<int>(futurePos.x / tileSize);
        int tileY = static_cast<int>(futurePos.y / tileSize);
        return tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() && 
               tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() && 
               mapData[tileY][tileX] == 1;
    };

    // Build the decision tree
    DecisionNode* wallOrTarget = new DecisionBranch(isNearWall, avoidWallAction, seekTargetAction);
    decisionTree = new DecisionBranch(isNearEnemy, fleeEnemyAction, wallOrTarget);
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

void boid::update(float deltaTime, const Kinematic& targetkin, const Kinematic& enemyKinematic) {
    // Implement the update logic here
    // This should include your steering behaviors
    // Example:
    sf::Vector2f steering = arrive->calculate(kinematic, const_cast<Kinematic&>(targetkin));
    kinematic.velocity += steering * deltaTime;
    
    // Limit speed
    if (length(kinematic.velocity) > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }
    
    move(deltaTime);
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