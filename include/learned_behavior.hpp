#ifndef LEARNED_BEHAVIOR_HPP
#define LEARNED_BEHAVIOR_HPP

#include <vector>
#include <SFML/Graphics.hpp>
#include "behavior_tree.hpp"
#include "steering.hpp"

struct BehaviorSample {
    float distanceToPlayer;
    float angleToPlayer;
    float relativeSpeed;
    bool isNearWall;
    sf::Vector2f actionTaken;
    Kinematic selfKinematic;
    Kinematic playerKinematic;
    std::vector<std::vector<int>> mapData;
};

class LearnedBehavior {
private:
    std::vector<BehaviorSample> trainingData;
    const size_t MAX_SAMPLES = 1000;
    
    struct DecisionNode {
        bool isLeaf;
        float threshold;
        int featureIndex;
        sf::Vector2f action;
        DecisionNode* left;
        DecisionNode* right;
        
        DecisionNode() : isLeaf(false), threshold(0), featureIndex(0), 
                        action(sf::Vector2f(0,0)), left(nullptr), right(nullptr) {}
    };
    
    DecisionNode* rootNode;

public:
    LearnedBehavior() : rootNode(nullptr) {}
    ~LearnedBehavior() { deleteTree(rootNode); }
    
    void recordBehavior(const Kinematic& self, const Kinematic& player, 
                       const std::vector<std::vector<int>>& mapData,
                       const sf::Vector2f& action);
                       
    sf::Vector2f makeDecision(const Kinematic& self, const Kinematic& player,
                             const std::vector<std::vector<int>>& mapData);
                             
    void trainDecisionTree();

private:
    void deleteTree(DecisionNode* node);
    DecisionNode* buildTree(const std::vector<BehaviorSample>& samples, int depth);
    std::vector<float> extractFeatures(const BehaviorSample& sample);

    // Add helper methods
    float length(const sf::Vector2f& v) const {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }
};

// Add global operator- outside the class
inline sf::Vector2f operator-(const sf::Vector2f& a, const sf::Vector2f& b) {
    return sf::Vector2f(a.x - b.x, a.y - b.y);
}

#endif // LEARNED_BEHAVIOR_HPP
