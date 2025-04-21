#ifndef ENEMY_BOID_HPP
#define ENEMY_BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.h"
#include "crumb.hpp"

class EnemyBoid {
private:
    sf::RenderWindow* window;
    sf::ConvexShape shape;
    Kinematic kinematic;
    std::vector<crumb>* breadcrumbs;
    int currentCrumb;
    float crumbTimer;

    // Helper functions
    float length(const sf::Vector2f& v);
    sf::Vector2f normalize(const sf::Vector2f& v);

public:
    EnemyBoid(sf::RenderWindow* win, std::vector<crumb>* crumbs, Kinematic k);
    
    sf::Vector2f getPosition() const;
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }
    void update(float dt, const Kinematic& targetKinematic);
    void move(float dt);
    void draw();
};

#endif // ENEMY_BOID_HPP