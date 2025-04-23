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
    sf::Vector2f direction = boidKinematic.position - kinematic.position;
    return normalize(direction) * kinematic.maxAcceleration;
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
    while (kinematic.orientation > 2 * 3.1459) {
        kinematic.orientation -= 2 * 3.1459;
    }
}

void EnemyBoid::update(float dt, const Kinematic& boidKinematic, 
                      const std::vector<std::vector<int>>& mapData, 
                      int tileSize, bool boidAtGoal) {
    // Execute behavior tree to get steering
    sf::Vector2f steering = root->makeDecision(kinematic, boidKinematic, kinematic, mapData, tileSize);
    
    // Apply steering
    kinematic.velocity += steering * dt;
    
    // Limit speed
    if (length(kinematic.velocity) > kinematic.maxSpeed) {
        kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
    }
    
    // Update position
    kinematic.position += kinematic.velocity * dt;
    
    // Update orientation based on velocity
    if (length(kinematic.velocity) > 0.1f) {
        kinematic.orientation = std::atan2(kinematic.velocity.y, kinematic.velocity.x);
    }
    
    // Update shape position and rotation
    shape.setPosition(kinematic.position);
    shape.setRotation(kinematic.orientation * 180.0f / 3.1459);
}

void EnemyBoid::draw() {
    window->draw(shape);
}