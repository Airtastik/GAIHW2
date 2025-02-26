
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
        sf::Vector2f calculate(Kinematic& character, Kinematic& target) {
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

    

#endif // STEERING_BEHAVIOR_H
