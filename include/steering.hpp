
#ifndef STEERING // Include guard to prevent multiple inclusions
#define STEERING
#include <cmath>
#include <SFML/Graphics.hpp>
#include <vector>
#include <iostream> // Standard library header for input/output

// Function declaration

class Kinematic {
    public:
      
        // main varables 
        sf::Vector2f position;
        sf::Vector2f velocity;
        float orientation; // In radians
        float rotation;    // Angular velocity (radians per second)

        float maxSpeed;
        float maxAcceleration;
        float maxAngularAcceleration;
        float maxRotation;
        float arrivalRadius;
        float slowRadius;
        float fleeRadius;
        
};

struct SteeringData {
    sf::Vector2f linearAcceleration;
    float angularAcceleration;
};

class SteeringBehavior {
    public:
        virtual sf::Vector2f calculateSteering(const Kinematic& character, const Kinematic& target) = 0;
        virtual ~SteeringBehavior() = default; 
    };
    
    class PositionSteering : public SteeringBehavior {
    public:
        sf::Vector2f calculateSteering(const Kinematic& character, const Kinematic& target){
            return target.position - character.position;
        }
        
        
    };
    class OrientationSteering : public SteeringBehavior {
    public:
        sf::Vector2f calculateSteering(const Kinematic& character, const Kinematic& target){
            float angleDiff = target.orientation - character.orientation;
            return sf::Vector2f(std::cos(angleDiff), std::sin(angleDiff));
        }
    };
    
    class VelocitySteering : public SteeringBehavior {
    public:
        sf::Vector2f  calculateSteering(const Kinematic& character, const Kinematic& target){
            return target.velocity - character.velocity;
        }
    };
    
    class RotationSteering : public SteeringBehavior {
    public:
        sf::Vector2f calculateSteering(const Kinematic& character, const Kinematic& target){
            float rotationDiff = target.rotation - character.rotation;
            return sf::Vector2f(rotationDiff, rotationDiff); 
       
    }
};
class Arrive {
    public:
    sf::Vector2f calculate(const Kinematic& character, const Kinematic& target){
            sf::Vector2f direction = target.position - character.position;
            float distance = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    
            if (distance < character.arrivalRadius)
                return sf::Vector2f(0, 0); // Stop when close enough
    
            float targetSpeed = (distance > character.slowRadius) ? character.maxSpeed : character.maxSpeed * (distance / character.slowRadius);
            sf::Vector2f targetVelocity = direction / distance * targetSpeed;
    
            return targetVelocity - character.velocity; // Steering force
        }
};
class Align {
    public:
        float calculate(Kinematic& character, const Kinematic& target) {
            float rotation = target.orientation - character.orientation;
            rotation = std::atan2(std::sin(rotation), std::cos(rotation)); // Normalize to [-π, π]
    
            // If the rotation is within the arrival radius, stop rotating
            if (std::abs(rotation) < character.arrivalRadius)
                return -character.rotation;
    
            // Determine the desired rotation speed
            float targetRotation = character.maxRotation;
            if (std::abs(rotation) < character.slowRadius)
                targetRotation *= (std::abs(rotation) / character.slowRadius); // Slow down near target
            
            // Apply rotation direction
            targetRotation *= (rotation > 0) ? 1 : -1;
    
            // Compute final steering torque (difference between desired and current rotation)
            return targetRotation - character.rotation;
        }
};
class flockingboid {
    public:
        flockingboid(sf::RenderWindow* w, sf::Texture& tex, Kinematic boidKin)
            : arrive(new Arrive()), align(new Align()) {
            window = w;
            kinematic = boidKin;
            sprite.setTexture(tex);
            sprite.setScale(0.02f, 0.02f);
            sprite.setOrigin(tex.getSize().x / 2, tex.getSize().y / 2);
        }
    
        ~flockingboid() {
            delete arrive;
            delete align;
        }
    
        void draw() {
            window->draw(sprite);
        }
    
        void update(float deltaTime, const std::vector<std::unique_ptr<flockingboid>>& flockingboids, Kinematic targetKinematic) {
            sf::Vector2f separation = computeSeparation(flockingboids);
            sf::Vector2f alignment = computeAlignment(flockingboids);
            sf::Vector2f cohesion = computeCohesion(flockingboids);
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
    
    private:
        sf::Vector2f computeSeparation(const std::vector<std::unique_ptr<flockingboid>>& flockingboids) {
            sf::Vector2f force(0, 0);
            for (const auto& other : flockingboids) {
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
    
        sf::Vector2f computeAlignment(const std::vector<std::unique_ptr<flockingboid>>& flockingboids) {
            sf::Vector2f avgVelocity(0, 0);
            int count = 0;
            for (const auto& other : flockingboids) {
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
    
        sf::Vector2f computeCohesion(const std::vector<std::unique_ptr<flockingboid>>& flockingboids) {
            sf::Vector2f centerOfMass(0, 0);
            int count = 0;
            for (const auto& other : flockingboids) {
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
        const float SEPARATION_RADIUS = 130.0f;
        const float ALIGNMENT_RADIUS = 100.0f;
        const float COHESION_RADIUS = 100.0f;
        const float SEPARATION_WEIGHT = 14.5f;
        const float ALIGNMENT_WEIGHT = 3.0f;
        const float COHESION_WEIGHT = 3.0f;
    };
    

#endif // STEERING_BEHAVIOR_H
