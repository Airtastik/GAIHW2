#ifndef BOID_HPP
#define BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.hpp"
#include "crumb.hpp"
#include "game_constants.hpp"

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
    bool isActive = true;
    int crumb_idx;
    int target_idx;
    float drop_timer;
    float speed;

    sf::Sprite sprite;
    sf::RenderWindow* window;
    std::vector<crumb>* breadcrumbs;
    DecisionNode* decisionTree;  // Now DecisionNode is recognized
    
    Kinematic kinematic;
    Kinematic targetKinematic;
    Arrive* arrive;
    Align* align;

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
        };

#endif // BOID_HPP