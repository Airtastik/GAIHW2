#include "boid.hpp"
#include "enemy_boid.hpp"
#include "behavior_tree.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

boid::boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin)
    : arrive(new Arrive()), align(new Align()), decisionTree(nullptr)  // Initialize decisionTree
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
    if (decisionTree) {
        delete decisionTree;
        decisionTree = nullptr;
    }
}

// Other methods remain unchanged...

// Build the decision tree for intelligent behavior
void boid::buildDecisionTree() {
    // Delete old tree if it exists
    if (decisionTree) {
        delete decisionTree;
        decisionTree = nullptr;
    }
    
    const float ENEMY_DETECTION_RADIUS = 50.0f; // Move constant definition here
    
    // Check if enemy is too close
    auto isEnemyNearby = [ENEMY_DETECTION_RADIUS](const Kinematic& k, const Kinematic& /*t*/,
                           const Kinematic& e,
                           const std::vector<std::vector<int>>& /*map*/,
                           int /*tileSize*/) -> bool {
        sf::Vector2f toEnemy = e.position - k.position;
        float distSq = toEnemy.x * toEnemy.x + toEnemy.y * toEnemy.y;
        return distSq < ENEMY_DETECTION_RADIUS * ENEMY_DETECTION_RADIUS;
    };

    // Create behavior nodes
    auto enemyAvoidNode = new EnemyAvoidanceNode();
    auto pathNode = new PathDeviationNode();
    auto normalNode = new ActionNode(
        [this](const Kinematic& k, const Kinematic& t,
               [[maybe_unused]] const Kinematic& e,
               [[maybe_unused]] const std::vector<std::vector<int>>& map,
               [[maybe_unused]] int size) -> sf::Vector2f {
            return arrive->calculate(k, t);
        }
    );

    // Build the tree
    decisionTree = new DecisionBranch(isEnemyNearby, 
                                     enemyAvoidNode,  // If enemy is near, avoid
                                     new DecisionBranch(isPathBlocked,
                                                      pathNode,    // If path blocked, deviate
                                                      normalNode   // Otherwise normal movement
                                     ));
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
    // Save previous position
    sf::Vector2f prevPosition = kinematic.position;
    
    // Update position
    kinematic.position += kinematic.velocity * dt;
    
    try {
        if (!mapData.empty() && !mapData[0].empty()) {
            int tileX = static_cast<int>(kinematic.position.x / 100);
            int tileY = static_cast<int>(kinematic.position.y / 100);
            
            // Bounds checking
            if (tileX >= 0 && tileX < static_cast<int>(mapData[0].size()) &&
                tileY >= 0 && tileY < static_cast<int>(mapData.size())) {
                
                if (mapData[tileY][tileX] == 1) {  // Wall collision
                    kinematic.position = prevPosition;
                    kinematic.velocity = -kinematic.velocity * 0.8f;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in move: " << e.what() << std::endl;
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

    // Move the boid
    move(dt);
}

void boid::update(float dt, const Kinematic& targetKinematic, [[maybe_unused]] const Kinematic& enemyKinematic) {
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

    // Move the boid
    move(dt);
}

void boid::updateWithEnemy(float dt, Kinematic targetKin, const Kinematic& enemyKin,
                          const std::vector<std::vector<int>>& newMapData, int tileSize) {
    if (!decisionTree) return;

    try {
        // Deep copy the map data
        mapData.resize(newMapData.size());
        for (size_t i = 0; i < newMapData.size(); ++i) {
            mapData[i] = newMapData[i];
        }

        // Calculate steering
        sf::Vector2f steering = decisionTree->makeDecision(kinematic, targetKin, enemyKin, mapData, tileSize);
        
        // Apply steering
        kinematic.velocity += steering * dt;
        
        // Limit speed
        float currentSpeed = length(kinematic.velocity);
        if (currentSpeed > kinematic.maxSpeed) {
            kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
        }

        move(dt);
    } catch (const std::exception& e) {
        std::cerr << "Error in updateWithEnemy: " << e.what() << std::endl;
    }
}