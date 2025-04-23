#ifndef ENEMY_BOID_HPP
#define ENEMY_BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.hpp"
#include "crumb.hpp"
#include "behavior_tree.hpp"

class EnemyBoid {
private:
    sf::RenderWindow* window;
    sf::ConvexShape shape;
    Kinematic kinematic;
    std::vector<crumb>* breadcrumbs;
    int currentCrumb;
    float crumbTimer;
    DecisionNode* root; // Root of the behavior tree

    // Helper functions
    float length(const sf::Vector2f& v);
    sf::Vector2f normalize(const sf::Vector2f& v);

public:
    EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k);
    
    sf::Vector2f getPosition() const;
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }
    void update(float dt, const Kinematic& boidKinematic, const std::vector<std::vector<int>>& mapData, int tileSize, bool boidAtGoal);
    void move(float dt);
    void draw();
    void buildBehaviorTree();

    // Actions
    sf::Vector2f avoidWall(const std::vector<std::vector<int>>& mapData, int tileSize);
    sf::Vector2f chaseBoid(const Kinematic& boidKinematic);
    sf::Vector2f goToCenter();
    void spinInPlace(float dt);
};

#endif // ENEMY_BOID_HPP