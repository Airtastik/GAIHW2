#ifndef RUN_DATA_HPP
#define RUN_DATA_HPP

#include "steering.hpp"
#include <vector>

struct BehaviorSample {
    Kinematic selfKinematic;
    Kinematic playerKinematic;
    std::vector<std::vector<int>> mapData;
    sf::Vector2f actionTaken;
    float distanceToPlayer;
    float angleToPlayer;
    float relativeSpeed;
    bool isNearWall;
};

struct RunData {
    bool playerWon;
    float runDuration;
    std::vector<BehaviorSample> samples;
    std::vector<std::string> branchesUsed;
    std::vector<sf::Vector2f> boidPath;
    std::vector<sf::Vector2f> enemyPath;
};

#endif // RUN_DATA_HPP
