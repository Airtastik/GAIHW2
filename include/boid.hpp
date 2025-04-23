#ifndef BOID_HPP
#define BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.hpp"
#include "crumb.hpp"
#include "game_constants.hpp"
#include "behavior_tree.hpp"  // Include complete type definition
#include "utils.hpp"  // Include utility functions

// Forward declarations
class EnemyBoid;
class DecisionNode;  // Add forward declaration for DecisionNode

class boid {
public:
    // ...existing code...
    boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin);
    ~boid();
    void executeDecisionTree(const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData, int tileSize);
    void draw();
    void update(float deltaTime, const Kinematic& targetkin);
    void update(float deltaTime, const Kinematic& targetkin, const Kinematic& enemyKinematic);
    void updateWithEnemy(float deltaTime, Kinematic targetkin, const Kinematic& enemyKinematic, 
                         const std::vector<std::vector<int>>& mapData, int tileSize);
    void move(float deltaTime);
    bool reached(sf::Vector2f pos, sf::Vector2f tar);
    sf::Vector2f getPosition() const;
    
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }
    
    sf::Vector2f normalize(const sf::Vector2f& v);
    float length(const sf::Vector2f& v);

    void buildDecisionTree();

private:
    // Add safety checks for decision tree
    bool hasValidDecisionTree() const {
        return decisionTree != nullptr && window != nullptr && breadcrumbs != nullptr;
    }

    // Make sure this is first in the private section
    sf::RenderWindow* window = nullptr;
    std::vector<crumb>* breadcrumbs = nullptr;
    DecisionNode* decisionTree = nullptr;
    Arrive* arrive = nullptr;
    Align* align = nullptr;
    
    // Reorder member variables to match initialization order
    sf::Sprite sprite;
    Kinematic kinematic;
    Kinematic targetKinematic;
    std::vector<std::vector<int>> mapData{10, std::vector<int>(10, 0)};
    
    bool isActive = true;
    int crumb_idx = 0;
    int target_idx = 0;
    float drop_timer = 0.0f;
    float speed = 0.0f;

    const sf::Vector2f TOP_LEFT = sf::Vector2f(100, 100);
    const sf::Vector2f TOP_RIGHT = sf::Vector2f(900, 100);
    const sf::Vector2f BOT_LEFT = sf::Vector2f(100, 900);
    const sf::Vector2f BOT_RIGHT = sf::Vector2f(900, 900);

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
    static bool isPathBlocked(const Kinematic& k, const Kinematic& t,
        const Kinematic& /*e*/,
        const std::vector<std::vector<int>>& mapData,
        int tileSize) {
        sf::Vector2f direction = t.position - k.position;
        float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        sf::Vector2f normalized = direction / distance;

        for (float d = 0; d < distance; d += tileSize / 2.0f) {
            sf::Vector2f checkPos = k.position + normalized * d;
            int checkX = static_cast<int>(checkPos.x / tileSize);
            int checkY = static_cast<int>(checkPos.y / tileSize);

            if (checkX >= 0 && static_cast<size_t>(checkX) < mapData[0].size() &&
            checkY >= 0 && static_cast<size_t>(checkY) < mapData.size() &&
            mapData[checkY][checkX] == 1) 
            {
            return true;
            }
        }
        return false;
    }

    // Add new helper methods
    sf::Vector2f dodgeEnemy(const Kinematic& enemyKinematic) {
        sf::Vector2f toEnemy = enemyKinematic.position - kinematic.position;
        sf::Vector2f perpendicular(-toEnemy.y, toEnemy.x);  // Get perpendicular vector
        return normalize(perpendicular) * kinematic.maxAcceleration;
    }

    sf::Vector2f slowDown() {
        return -normalize(kinematic.velocity) * kinematic.maxAcceleration * 0.5f;
    }

    // Add random behavior options
    sf::Vector2f randomFlank() {
        static float direction = 1.0f;  // 1 for right, -1 for left
        static float lastChangeTime = 0.0f;
        
        // Randomly change direction every ~0.5-1.5 seconds
        float currentTime = clock.getElapsedTime().asSeconds();
        if (currentTime - lastChangeTime > (0.5f + (rand() % 1000) / 1000.0f)) {
            direction = (rand() % 2) * 2.0f - 1.0f;  // Random -1 or 1
            lastChangeTime = currentTime;
        }
        
        if (length(kinematic.velocity) < 0.1f) {
            return sf::Vector2f(direction * kinematic.maxAcceleration, 0);
        }
        
        sf::Vector2f perpendicular(-kinematic.velocity.y, kinematic.velocity.x);
        return normalize(perpendicular) * direction * kinematic.maxAcceleration;
    }

    sf::Vector2f randomDodge() {
        // Random angle between -45 and 45 degrees
        float angle = (rand() % 90 - 45) * M_PI / 180.0f;
        sf::Vector2f dir(std::cos(angle), std::sin(angle));
        return dir * kinematic.maxAcceleration * 2.0f;
    }

    sf::Vector2f randomBurst() {
        static bool bursting = false;
        static float burstTimer = 0.0f;
        
        if (!bursting && (rand() % 100 < 10)) {  // 10% chance to start burst
            bursting = true;
            burstTimer = 0.5f;  // Burst for 0.5 seconds
        }
        
        if (bursting) {
            burstTimer -= clock.restart().asSeconds();
            if (burstTimer <= 0) {
                bursting = false;
            }
            return normalize(kinematic.velocity) * kinematic.maxAcceleration * 3.0f;
        }
        
        return sf::Vector2f(0, 0);
    }
    
    sf::Clock clock;  // Add clock member variable

    // Add safety check method
    bool isValid() const {
        return window != nullptr && breadcrumbs != nullptr;
    }

    // Add memory cleanup method
    void cleanup() {
        if (decisionTree) {
            delete decisionTree;
            decisionTree = nullptr;
        }
        delete arrive;
        arrive = nullptr;
        delete align;
        align = nullptr;
    }
};

#endif // BOID_HPP