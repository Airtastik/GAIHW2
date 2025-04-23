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
    
    // Only chase if within certain range
    if (distance < 300.0f) { // Chase radius
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
    [[maybe_unused]] const std::vector<std::vector<int>>& mapData, 
    [[maybe_unused]] int tileSize, 
    [[maybe_unused]] bool boidAtGoal) {
// Calculate steering force
sf::Vector2f steering = root->makeDecision(kinematic, boidKinematic, kinematic, mapData, tileSize);
    
    // Add boundary avoidance
    sf::Vector2f boundaryForce = avoidBoundary();
    steering += boundaryForce;
    
    // Apply steering to velocity
    kinematic.velocity += steering * dt;
    
    // Clamp position to window bounds
    kinematic.position.x = std::clamp(kinematic.position.x, 0.0f, WINDOW_WIDTH);
    kinematic.position.y = std::clamp(kinematic.position.y, 0.0f, WINDOW_HEIGHT);
    
    // Limit speed
    float currentSpeed = length(kinematic.velocity);
    if (currentSpeed > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }
// Update position
//kinematic.position += kinematic.velocity * dt;

// Debug output
std::cout << "Enemy position: (" << kinematic.position.x << ", " << kinematic.position.y << ")\n";
std::cout << "Enemy velocity: (" << kinematic.velocity.x << ", " << kinematic.velocity.y << ")\n";
std::cout << "Enemy steering: (" << steering.x << ", " << steering.y << ")\n";
}

void EnemyBoid::draw() {
    window->draw(shape);
}
void EnemyBoid::move([[maybe_unused]] float dt) {
    // Update shape position
    shape.setPosition(kinematic.position);
    
    // Update orientation if moving
    if (length(kinematic.velocity) > 0.1f) {
        float angle = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
        shape.setRotation(angle * 180.0f / 3.14159f);
    }
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
    , root(nullptr)
{
    // Initialize shape
    shape.setPointCount(3);
    shape.setPoint(0, sf::Vector2f(0, -10));  // Top
    shape.setPoint(1, sf::Vector2f(-10, 10)); // Bottom left
    shape.setPoint(2, sf::Vector2f(10, 10));  // Bottom right
    shape.setFillColor(sf::Color::Red);
    shape.setPosition(kinematic.position);
    shape.setOrigin(0, 0);

    buildBehaviorTree();
}

void EnemyBoid::buildBehaviorTree() {
    // Create action nodes
    auto chaseBoidAction = new ActionNode(
        [this](const Kinematic& /*k*/, const Kinematic& t, const Kinematic& /*e*/,
               const std::vector<std::vector<int>>& /*map*/, int /*tileSize*/) -> sf::Vector2f {
            sf::Vector2f result = chaseBoid(t);
            //std::cout << "Chase steering: (" << result.x << ", " << result.y << ")" << std::endl;
            return result;
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
        return length(k.position - t.position) < 300.0f;  // Detection radius
    };

    // Build the behavior tree
    DecisionNode* wallOrChase = new DecisionBranch(isNearWall, avoidWallAction, chaseBoidAction);
    root = new DecisionBranch(isNearBoid, wallOrChase, goToCenterAction);
}

// Add destructor to clean up the behavior tree
EnemyBoid::~EnemyBoid() {
    delete root;
}