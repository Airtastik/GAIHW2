#ifndef LEARNING_ENEMY_HPP
#define LEARNING_ENEMY_HPP

#include "enemy_boid.hpp"
#include "learned_behavior.hpp"
#include "utils.hpp"  // Add this include

class LearningEnemy : public EnemyBoid {
private:
    LearnedBehavior* learner;

public:
    LearningEnemy(sf::RenderWindow* win, std::vector<crumb>* crumbs,
                  Kinematic k, LearnedBehavior* l)
        : EnemyBoid(win, crumbs, k), learner(l) {}

    void update(float dt, const Kinematic& boidKinematic,
                const std::vector<std::vector<int>>& mapData,
                int tileSize, bool boidAtGoal) override {
        if (!learner) {
            EnemyBoid::update(dt, boidKinematic, mapData, tileSize, boidAtGoal);
            return;
        }

        // Use learned behavior
        sf::Vector2f action = learner->makeDecision(kinematic, boidKinematic, mapData);
        kinematic.velocity = action;
        
        // Apply basic physics and constraints
        float currentSpeed = length(kinematic.velocity);
        if (currentSpeed > kinematic.maxSpeed) {
            kinematic.velocity = normalize(kinematic.velocity) * kinematic.maxSpeed;
        }

        move(dt, mapData);
    }
};

#endif // LEARNING_ENEMY_HPP
