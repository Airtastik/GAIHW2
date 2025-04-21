#ifndef BOID_HPP
#define BOID_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include "steering.h"
#include "crumb.hpp"

class boid
{
public:
    boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin);
    ~boid();
    
    void draw();
    void update(float deltaTime, Kinematic targetkin);
    void move(float deltaTime);
    bool reached(sf::Vector2f pos, sf::Vector2f tar);
    sf::Vector2f getPosition() const;
    
    // Added for wall avoidance
    Kinematic getKinematic() const { return kinematic; }
    void setKinematic(const Kinematic& kin) { kinematic = kin; }

private:
    int crumb_idx;
    int target_idx;
    float drop_timer;
    float speed;

    sf::Sprite sprite;
    sf::RenderWindow* window;
    std::vector<crumb>* breadcrumbs;

    Kinematic kinematic;
    Kinematic targetKinematic;
    Arrive* arrive;
    Align* align;
    
    // Constants for target positions
    const sf::Vector2f TOP_LEFT = sf::Vector2f(100, 100);
    const sf::Vector2f TOP_RIGHT = sf::Vector2f(900, 100);
    const sf::Vector2f BOT_LEFT = sf::Vector2f(100, 900);
    const sf::Vector2f BOT_RIGHT = sf::Vector2f(900, 900);
};

#endif // BOID_HPP