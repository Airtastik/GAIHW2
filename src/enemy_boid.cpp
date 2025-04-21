#include "enemy_boid.hpp"
#include <cmath>

EnemyBoid::EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k) 
    : window(win), breadcrumbs(crumbs), kinematic(k), currentCrumb(0), crumbTimer(0)
{
    // Create triangle shape
    shape.setPointCount(3);
    shape.setPoint(0, sf::Vector2f(0, -15)); // Top point
    shape.setPoint(1, sf::Vector2f(-10, 10)); // Bottom left
    shape.setPoint(2, sf::Vector2f(10, 10)); // Bottom right
    shape.setFillColor(sf::Color::Red);
    shape.setPosition(kinematic.position);
    shape.setOrigin(0, 0);
}

sf::Vector2f EnemyBoid::getPosition() const {
    return kinematic.position;
}

void EnemyBoid::update(float dt, const Kinematic& targetKinematic) {
    // Seek behavior toward target
    sf::Vector2f direction = targetKinematic.position - kinematic.position;
    float distance = length(direction);
    
    if (distance > 0) {
        direction = normalize(direction); // Normalize
        
        // Calculate desired velocity (seeking)
        sf::Vector2f desiredVelocity = direction * kinematic.maxSpeed;
        
        // Calculate steering force
        sf::Vector2f steeringForce = desiredVelocity - kinematic.velocity;
        
        // Apply acceleration
        sf::Vector2f acceleration = steeringForce;
        if (length(acceleration) > kinematic.maxAcceleration) {
            acceleration = normalize(acceleration) * kinematic.maxAcceleration;
        }
        
        // Update velocity
        kinematic.velocity += acceleration * dt;
        if (length(kinematic.velocity) > kinematic.maxSpeed) {
            kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
        }
    }
    
    // Update rotation to face direction of movement
    if (length(kinematic.velocity) > 0.1f) {
        float angle = atan2(kinematic.velocity.y, kinematic.velocity.x) * 180 / 3.1459;
        shape.setRotation(angle + 90); // +90 because triangle points up by default
    }
    
    // Update crumb dropping
    crumbTimer += dt;
    if (crumbTimer >= 0.1f) { // Drop crumb every 0.1 seconds
        breadcrumbs->at(currentCrumb).drop(kinematic.position);
        currentCrumb = (currentCrumb + 1) % breadcrumbs->size();
        crumbTimer = 0;
    }
}

void EnemyBoid::move(float dt) {
    // Move based on velocity
    kinematic.position += kinematic.velocity * dt;
    
    // Handle screen wrapping
    if(kinematic.position.x > 1002){
        kinematic.position.x = 0;
    }
    if(kinematic.position.x < -2){
        kinematic.position.x = 1000;
    }
    if(kinematic.position.y < -2){
        kinematic.position.y = 1000;
    }
    if(kinematic.position.y > 1002){
        kinematic.position.y = 0;
    }
    
    shape.setPosition(kinematic.position);
}

void EnemyBoid::draw() {
    window->draw(shape);
}

// Helper functions
float EnemyBoid::length(const sf::Vector2f& v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

sf::Vector2f EnemyBoid::normalize(const sf::Vector2f& v) {
    float len = length(v);
    if (len > 0) {
        return v / len;
    }
    return v;
}