#ifndef ENEMY_BOID_HPP
#define ENEMY_BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include "crumb.hpp"
#include "behavior_tree.hpp"

class EnemyBoid {
private:
    sf::RenderWindow* window;
    Kinematic kinematic;
    std::vector<crumb>* breadcrumbs;
    sf::ConvexShape shape;
    int currentCrumb;
    float crumbTimer;
    DecisionNode* root;

    // Helper functions
    float length(const sf::Vector2f& v);
    sf::Vector2f normalize(const sf::Vector2f& v);

public:
    // Constructor declaration only
    EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k);
    ~EnemyBoid();

    // Public interface methods
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }
    void update(float dt, const Kinematic& boidKinematic, 
                const std::vector<std::vector<int>>& mapData, 
                int tileSize, bool boidAtGoal);
    void move(float dt);
    void draw();
    void buildBehaviorTree();
    sf::Vector2f avoidWall(const std::vector<std::vector<int>>& mapData, int tileSize);
    sf::Vector2f chaseBoid(const Kinematic& boidKinematic);
    sf::Vector2f goToCenter();
    void spinInPlace(float dt);
};

#endif