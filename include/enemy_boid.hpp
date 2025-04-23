#ifndef ENEMY_BOID_HPP
#define ENEMY_BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include "crumb.hpp"
#include "behavior_tree.hpp"
#include "game_constants.hpp"

class EnemyBoid {
protected:  // Change private to protected for inheritance
    sf::RenderWindow* window;
    Kinematic kinematic;
    std::vector<crumb>* breadcrumbs;
    sf::ConvexShape shape;
    int currentCrumb;
    float crumbTimer;
    DecisionNode* root;

    // Remove length and normalize declarations

    // Make avoidBoundary() inline in header
    sf::Vector2f avoidBoundary() {
        sf::Vector2f steering(0, 0);
        
        if (kinematic.position.x < BOUNDARY_RADIUS)
            steering.x = kinematic.maxAcceleration;
        else if (kinematic.position.x > WINDOW_WIDTH - BOUNDARY_RADIUS)
            steering.x = -kinematic.maxAcceleration;
            
        if (kinematic.position.y < BOUNDARY_RADIUS)
            steering.y = kinematic.maxAcceleration;
        else if (kinematic.position.y > WINDOW_HEIGHT - BOUNDARY_RADIUS)
            steering.y = -kinematic.maxAcceleration;
            
        return steering;
    }

public:
    EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k);
    virtual ~EnemyBoid() = default;  // Add virtual destructor

    // Public interface methods
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }
    virtual void update(float dt, const Kinematic& boidKinematic,  // Add virtual keyword
                const std::vector<std::vector<int>>& mapData, 
                int tileSize, bool boidAtGoal);
    void move(float dt, const std::vector<std::vector<int>>& mapData);  // Updated signature
    void draw();
    void buildBehaviorTree();
    sf::Vector2f avoidWall(const std::vector<std::vector<int>>& mapData, int tileSize);
    sf::Vector2f chaseBoid(const Kinematic& boidKinematic);
    sf::Vector2f goToCenter();
    void spinInPlace(float dt);

    // Add new method for wandering behavior
    sf::Vector2f wander() {
        // Change direction randomly
        float angle = kinematic.orientation + 
                     (((float)rand() / RAND_MAX) * 2.0f - 1.0f) * 0.5f;
        
        // Calculate new direction vector
        sf::Vector2f direction(std::cos(angle), std::sin(angle));
        
        return direction * kinematic.maxAcceleration;
    }
};

#endif // ENEMY_BOID_HPP