#include "enemy_boid.hpp"
#include <cmath>

// Helper functions implementation
float EnemyBoid::length(const sf::Vector2f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

sf::Vector2f EnemyBoid::normalize(const sf::Vector2f& v) {
    float len = length(v);
    if (len > 0) {
        return sf::Vector2f(v.x / len, v.y / len);
    }
    return v;
}

sf::Vector2f EnemyBoid::chaseBoid(const Kinematic& boidKinematic) {
    // Calculate direction to player boid
    sf::Vector2f direction = boidKinematic.position - kinematic.position;
    float distance = length(direction);
    
    // Increase chase radius from 300 to 500
    if (distance < 500.0f) { // Chase radius increased
        // Calculate desired velocity (at max speed toward player)
        sf::Vector2f desiredVelocity = normalize(direction) * kinematic.maxSpeed;
        
        // Calculate steering force
        sf::Vector2f steeringForce = desiredVelocity - kinematic.velocity;
        
        // Limit steering force to max acceleration
        float steeringMagnitude = length(steeringForce);
        if (steeringMagnitude > kinematic.maxAcceleration) {
            steeringForce = normalize(steeringForce) * kinematic.maxAcceleration;
        }
        
        return steeringForce;
    }
    
    return sf::Vector2f(0, 0);
}

sf::Vector2f EnemyBoid::goToCenter() {
    // Assuming window center is at (500, 500)
    sf::Vector2f center(500, 500);
    sf::Vector2f direction = center - kinematic.position;
    return normalize(direction) * kinematic.maxAcceleration;
}

void EnemyBoid::spinInPlace(float dt) {
    kinematic.orientation += dt * kinematic.maxRotation;
    // Keep orientation between 0 and 2π
    while (kinematic.orientation > 2 * 3.1459f) {
        kinematic.orientation -= 2 * 3.1459f;
    }
}
sf::Vector2f EnemyBoid::avoidWall(const std::vector<std::vector<int>>& mapData, int tileSize) {
    sf::Vector2f futurePos = kinematic.position + normalize(kinematic.velocity) * 50.0f;
    int tileX = static_cast<int>(futurePos.x / tileSize);
    int tileY = static_cast<int>(futurePos.y / tileSize);
    if (tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() && 
        tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() && 
        mapData[tileY][tileX] == 1) {
        sf::Vector2f wallCenter((tileX + 0.5f) * tileSize, (tileY + 0.5f) * tileSize);
        return normalize(kinematic.position - wallCenter) * kinematic.maxAcceleration;
    }
    return sf::Vector2f(0, 0);
}

void EnemyBoid::update(float dt, const Kinematic& boidKinematic, 
    const std::vector<std::vector<int>>& mapData, 
    int tileSize, bool boidAtGoal) {
    
    if (!root) return;
    
    // Calculate steering force with null checks
    sf::Vector2f steering = root->makeDecision(kinematic, boidKinematic, kinematic, mapData, tileSize);
    
    // Add boundary avoidance
    sf::Vector2f boundaryForce = avoidBoundary();
    steering += boundaryForce;
    
    // Apply steering directly without limiting the time step
    kinematic.velocity += steering * dt;
    
    // Only limit the final speed
    float currentSpeed = length(kinematic.velocity);
    if (currentSpeed > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }

    // Update position
    kinematic.position += kinematic.velocity * dt;

    // Update move call to include mapData
    move(dt, mapData);
}

void EnemyBoid::draw() {
    window->draw(shape);
}
void EnemyBoid::move(float dt, const std::vector<std::vector<int>>& mapData) {
    // Early return if map data is invalid
    if (mapData.empty() || mapData[0].empty()) {
        // Just do basic movement without wall collision
        kinematic.position += kinematic.velocity * dt;
        return;
    }
    
    // Save previous position
    sf::Vector2f prevPosition = kinematic.position;
    
    // Update position
    kinematic.position += kinematic.velocity * dt;
    
    // Get tile positions with bounds checking
    int tileX = static_cast<int>(kinematic.position.x / 100);
    int tileY = static_cast<int>(kinematic.position.y / 100);
    
    if (tileX >= 0 && tileX < static_cast<int>(mapData[0].size()) && 
        tileY >= 0 && tileY < static_cast<int>(mapData.size())) {
        
        if (mapData[tileY][tileX] == 1) {
            kinematic.position = prevPosition;
            kinematic.velocity = -kinematic.velocity * 0.8f;
        }
    }
    
    // Bounce off window boundaries
    if (kinematic.position.x < 0) {
        kinematic.position.x = 0;
        kinematic.velocity.x = -kinematic.velocity.x;
    } else if (kinematic.position.x > WINDOW_WIDTH) {
        kinematic.position.x = WINDOW_WIDTH;
        kinematic.velocity.x = -kinematic.velocity.x;
    }
    
    if (kinematic.position.y < 0) {
        kinematic.position.y = 0;
        kinematic.velocity.y = -kinematic.velocity.y;
    } else if (kinematic.position.y > WINDOW_HEIGHT) {
        kinematic.position.y = WINDOW_HEIGHT;
        kinematic.velocity.y = -kinematic.velocity.y;
    }

    // Update shape position and orientation
    shape.setPosition(kinematic.position);
    
    if (length(kinematic.velocity) > 0.1f) {
        float angle = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        shape.setRotation(angle * 180.0f / 3.14159f);
    }

    shape.setOrigin(0, 0);
}

// Make sure to define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

EnemyBoid::EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k)
    : window(win)
    , kinematic(k)
    , breadcrumbs(crumbs)
    , currentCrumb(0)
    , crumbTimer(0)
    , root(nullptr)  // Initialize root to nullptr
{
    // Initialize shape with larger size
    shape.setPointCount(3);
    shape.setPoint(0, sf::Vector2f(0, -20));    // Top (doubled from -10)
    shape.setPoint(1, sf::Vector2f(-20, 20));   // Bottom left (doubled from -10, 10)
    shape.setPoint(2, sf::Vector2f(20, 20));    // Bottom right (doubled from 10, 10)
    shape.setFillColor(sf::Color::Red);
    shape.setPosition(kinematic.position);
    shape.setOrigin(0, 0);

    buildBehaviorTree();
}

EnemyBoid::~EnemyBoid() {
    if (root) {
        delete root;
        root = nullptr;
    }
}

void EnemyBoid::buildBehaviorTree() {
    // Delete old tree if it exists
    if (root) {
        delete root;
        root = nullptr;
    }
    
    // Create action nodes
    auto wanderAction = new ActionNode(
        [this](const Kinematic& /*k*/, const Kinematic& /*t*/, const Kinematic& /*e*/,
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            return wander();
        }
    );

    auto chaseBoidAction = new ActionNode(
        [this](const Kinematic& /*k*/, const Kinematic& t, const Kinematic& /*e*/,
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            return chaseBoid(t);
        }
    );

    auto goToCenterAction = new ActionNode(
        [this](const Kinematic& /*k*/, const Kinematic& /*t*/, const Kinematic& /*e*/,
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            return goToCenter();
        }
    );

    auto avoidWallAction = new ActionNode(
        [this]([[maybe_unused]] const Kinematic& k, 
               const Kinematic& /*t*/, 
               const Kinematic& /*e*/,
               const std::vector<std::vector<int>>& mapData, 
               int tileSize) -> sf::Vector2f {
            return avoidWall(mapData, tileSize);
        }
    );

    // Create condition nodes
    auto isNearWall = [this](const Kinematic& k, const Kinematic& /*t*/, const Kinematic& /*e*/,
                            const std::vector<std::vector<int>>& mapData, int tileSize) -> bool {
        sf::Vector2f futurePos = k.position + normalize(k.velocity) * 50.0f;
        int tileX = static_cast<int>(futurePos.x / tileSize);
        int tileY = static_cast<int>(futurePos.y / tileSize);
        return tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() &&
               tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() &&
               mapData[tileY][tileX] == 1;
    };

    auto isNearBoid = [this](const Kinematic& k, const Kinematic& t, const Kinematic& /*e*/,
                            const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> bool {
        return length(k.position - t.position) < 500.0f;  // Detection radius increased from 300 to 500
    };

    // Build the behavior tree
    DecisionNode* wallOrChase = new DecisionBranch(isNearWall, avoidWallAction, chaseBoidAction);
    // If boid is near, chase or avoid walls, otherwise wander
    root = new DecisionBranch(isNearBoid, wallOrChase, wanderAction);
}