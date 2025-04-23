#ifndef BOID_HPP
#define BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.hpp"
#include "crumb.hpp"

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
    void update(float deltaTime, const Kinematic& targetkin) {
        update(deltaTime, targetkin, Kinematic()); // Pass default enemy kinematic
    }
    
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

    void buildDecisionTree();
    const sf::Vector2f TOP_LEFT = sf::Vector2f(100, 100);
    const sf::Vector2f TOP_RIGHT = sf::Vector2f(900, 100);
    const sf::Vector2f BOT_LEFT = sf::Vector2f(100, 900);
    const sf::Vector2f BOT_RIGHT = sf::Vector2f(900, 900);
};

#endif // BOID_HPP