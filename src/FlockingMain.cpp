#include "steering.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>

const float SEPARATION_RADIUS = 100.0f;
const float ALIGNMENT_RADIUS = 100.0f;
const float COHESION_RADIUS = 100.0f;
const float SEPARATION_WEIGHT = 10.5f;
const float ALIGNMENT_WEIGHT = 3.0f;
const float COHESION_WEIGHT = 2.0f;

class boid {
public:
    boid(sf::RenderWindow* w, sf::Texture& tex, Kinematic boidKin)
        : arrive(new Arrive()), align(new Align()) {
        window = w;
        kinematic = boidKin;
        sprite.setTexture(tex);
        sprite.setScale(0.02f, 0.02f);
        sprite.setOrigin(tex.getSize().x / 2, tex.getSize().y / 2);
    }

    ~boid() {
        delete arrive;
        delete align;
    }

    void draw() {
        window->draw(sprite);
    }

    void update(float deltaTime, const std::vector<std::unique_ptr<boid>>& boids, Kinematic targetKinematic) {
        sf::Vector2f separation = computeSeparation(boids);
        sf::Vector2f alignment = computeAlignment(boids);
        sf::Vector2f cohesion = computeCohesion(boids);
        sf::Vector2f arriveForce = arrive->calculate(kinematic, targetKinematic);

        sf::Vector2f steering = arriveForce + 
                                SEPARATION_WEIGHT * separation +
                                ALIGNMENT_WEIGHT * alignment +
                                COHESION_WEIGHT * cohesion;
        
        kinematic.velocity += steering * deltaTime;
        if (std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) > kinematic.maxSpeed)
            kinematic.velocity /= std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) / kinematic.maxSpeed;

        kinematic.position += kinematic.velocity * deltaTime;

        float angle = std::atan2(kinematic.velocity.y, kinematic.velocity.x) * (180.0f / 3.14159f);
        sprite.setRotation(angle);
        sprite.setPosition(kinematic.position);
    }

private:
    sf::Vector2f computeSeparation(const std::vector<std::unique_ptr<boid>>& boids) {
        sf::Vector2f force(0, 0);
        for (const auto& other : boids) {
            if (other.get() == this) continue;
            sf::Vector2f diff = kinematic.position - other->kinematic.position;
            float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
            if (dist < SEPARATION_RADIUS && dist > 0) {
                diff /= dist;
                force += diff;
            }
        }
        return force;
    }

    sf::Vector2f computeAlignment(const std::vector<std::unique_ptr<boid>>& boids) {
        sf::Vector2f avgVelocity(0, 0);
        int count = 0;
        for (const auto& other : boids) {
            if (other.get() == this) continue;
            float dist = std::sqrt(pow(kinematic.position.x - other->kinematic.position.x, 2) +
                                   pow(kinematic.position.y - other->kinematic.position.y, 2));
            if (dist < ALIGNMENT_RADIUS) {
                avgVelocity += other->kinematic.velocity;
                count++;
            }
        }
        if (count > 0) avgVelocity /= (float)count;
        return avgVelocity;
    }

    sf::Vector2f computeCohesion(const std::vector<std::unique_ptr<boid>>& boids) {
        sf::Vector2f centerOfMass(0, 0);
        int count = 0;
        for (const auto& other : boids) {
            if (other.get() == this) continue;
            float dist = std::sqrt(pow(kinematic.position.x - other->kinematic.position.x, 2) +
                                   pow(kinematic.position.y - other->kinematic.position.y, 2));
            if (dist < COHESION_RADIUS) {
                centerOfMass += other->kinematic.position;
                count++;
            }
        }
        if (count > 0) {
            centerOfMass /= (float)count;
            return centerOfMass - kinematic.position;
        }
        return sf::Vector2f(0, 0);
    }

    sf::Sprite sprite;
    sf::RenderWindow* window;
    Kinematic kinematic;
    Arrive* arrive;
    Align* align;
};

int main() {
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Boids Simulation");
    sf::Texture texture;
    if (!texture.loadFromFile("boid.png")) {
        return -1;
    }

    Kinematic targetKinematic;
    targetKinematic.position = sf::Vector2f(500, 500);

    std::vector<std::unique_ptr<boid>> boids;
    for (int j = 0; j < 20; j++) {
        Kinematic boidKinematic;
        boidKinematic.position = sf::Vector2f(rand() % 1000, rand() % 1000);
        boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        boidKinematic.maxSpeed = 200.0f;
        boidKinematic.arrivalRadius = 5.0f;
        boids.push_back(std::make_unique<boid>(&window, texture, boidKinematic));
    }

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Time deltaTime = clock.restart();
        float dt = deltaTime.asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        targetKinematic.position = sf::Vector2f(mousePos);

        window.clear(sf::Color(255, 255, 255, 255));
        for (auto& b : boids) {
            b->update(dt, boids, targetKinematic);
            b->draw();
        }
        window.display();
    }
    return 0;
}
