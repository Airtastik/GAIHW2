#include "boid.hpp"

boid::boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin)
    : arrive(new Arrive()), align(new Align())
{
    window = w;
    kinematic = boidKin;
    speed = 1.0f;
    drop_timer = 100.f;
    crumb_idx = 0;
    target_idx = 0;
    sprite.setTexture(tex);
    sprite.setScale(0.02f, 0.02f);
    breadcrumbs = crumbs;
    sprite.setPosition(kinematic.position);
}

boid::~boid() {
    delete arrive;
    delete align;
}

void boid::draw() {
    window->draw(sprite);
}

void boid::update(float deltaTime, Kinematic targetkin) {
    targetKinematic = targetkin;
    move(deltaTime);
}

void boid::move(float deltaTime)
{
    if (drop_timer > 0)
    {
        drop_timer -= 0.1f;
    }
    else
    {
        drop_timer = 100.f;
        breadcrumbs->at(crumb_idx).drop(kinematic.position);

        if (crumb_idx < 14)
            crumb_idx++;
        else
            crumb_idx = 0;
    }

    //basic timer for leaving breadcrumbs
    //if reached target, switch to next target
    if (reached(kinematic.position, targetKinematic.position))
    {
        switch (target_idx)
        {
            case 0:
                targetKinematic.position = TOP_RIGHT;
                target_idx = 1;
                targetKinematic.velocity = sf::Vector2f(1, 0);
                targetKinematic.orientation = 0;                     
                break;
            case 1:
                targetKinematic.position = BOT_RIGHT;
                target_idx = 2;
                targetKinematic.velocity = sf::Vector2f(0, 1);
                targetKinematic.orientation = 1.5708;
                break;
            case 2:
                targetKinematic.position = BOT_LEFT;
                target_idx = 3;
                targetKinematic.velocity = sf::Vector2f(-1, 0);
                targetKinematic.orientation= 3.14159;
                break;
            case 3:
                targetKinematic.position = TOP_LEFT;
                target_idx = 0;
                targetKinematic.velocity = sf::Vector2f(0, -1);
                targetKinematic.orientation = 4.71239;
                break;
        }
    }
    //else move toward target
    else
    {
        sf::Vector2f acceleration = arrive->calculate(kinematic, targetKinematic);
        float angularAcceleration = align->calculate(kinematic, targetKinematic);

        // Update velocity and position
        kinematic.velocity += acceleration * deltaTime;
        if (std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) > kinematic.maxSpeed)
            kinematic.velocity /= std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) / kinematic.maxSpeed;

        kinematic.position += kinematic.velocity * deltaTime;

        // Update rotation
        kinematic.rotation +=  kinematic.maxRotation * deltaTime;
        if (std::abs(kinematic.rotation) > kinematic.maxRotation){
            kinematic.rotation = kinematic.maxRotation * (kinematic.rotation / std::abs(kinematic.rotation));
            
        kinematic.orientation += kinematic.rotation * deltaTime;}
        float angle = std::atan2(kinematic.velocity.y, kinematic.velocity.x) * (180.0f / 3.1459); 
        sprite.setRotation(angle);
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
        sprite.setPosition(kinematic.position);
    }
}

//check if boid has reached target position
bool boid::reached(sf::Vector2f pos, sf::Vector2f tar)        
{
    sf::Vector2f dis = tar - pos;
    if (abs(dis.x + dis.y) <= 1)
        return true;
    return false;
}

sf::Vector2f boid::getPosition() const {
    return kinematic.position;
}