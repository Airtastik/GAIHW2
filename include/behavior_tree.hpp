#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <functional>
#include "steering.hpp"

// Base class for decision tree nodes
class DecisionNode {
public:
    virtual ~DecisionNode() {}
    virtual sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                                      const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                                      int tileSize) = 0;
};
class DecisionBranch : public DecisionNode {
    private:
        std::function<bool(const Kinematic&, const Kinematic&, const Kinematic&, 
                          const std::vector<std::vector<int>>&, int)> condition;
        DecisionNode* trueNode;
        DecisionNode* falseNode;
        
    public:
        DecisionBranch(std::function<bool(const Kinematic&, const Kinematic&, const Kinematic&, 
                                         const std::vector<std::vector<int>>&, int)> cond, 
                       DecisionNode* t, DecisionNode* f)
            : condition(cond), trueNode(t), falseNode(f) {}
            
        ~DecisionBranch() {
            delete trueNode;
            delete falseNode;
        }
        
        sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                                 const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                                 int tileSize) override {
            if (condition(kinematic, targetKinematic, enemyKinematic, mapData, tileSize)) {
                return trueNode->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
            } else {
                return falseNode->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
            }
        }
    };

// Leaf node for decision tree
class ActionNode : public DecisionNode {
private:
    std::function<sf::Vector2f(const Kinematic&, const Kinematic&, const Kinematic&, 
                              const std::vector<std::vector<int>>&, int)> action;
    
public:
    ActionNode(std::function<sf::Vector2f(const Kinematic&, const Kinematic&, const Kinematic&, 
                                        const std::vector<std::vector<int>>&, int)> act)
        : action(act) {}
        
    sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                             const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                             int tileSize) override {
        return action(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
    }
};

// Composite node: Sequence
class SequenceNode : public DecisionNode {
private:
    std::vector<DecisionNode*> children;

public:
    SequenceNode(const std::vector<DecisionNode*>& nodes) : children(nodes) {}

    sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                              const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                              int tileSize) override {
        for (auto child : children) {
            sf::Vector2f result = child->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
            if (result == sf::Vector2f(0, 0)) {
                return sf::Vector2f(0, 0); // Fail if any child fails
            }
        }
        return sf::Vector2f(1, 1); // Success if all children succeed
    }
};

// Composite node: Selector
class SelectorNode : public DecisionNode {
private:
    std::vector<DecisionNode*> children;

public:
    SelectorNode(const std::vector<DecisionNode*>& nodes) : children(nodes) {}

    sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                              const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                              int tileSize) override {
        for (auto child : children) {
            sf::Vector2f result = child->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
            if (result != sf::Vector2f(0, 0)) {
                return result; // Succeed if any child succeeds
            }
        }
        return sf::Vector2f(0, 0); // Fail if all children fail
    }
};

// Decorator node: Inverter
class InverterNode : public DecisionNode {
private:
    DecisionNode* child;

public:
    InverterNode(DecisionNode* node) : child(node) {}

    sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                              const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                              int tileSize) override {
        sf::Vector2f result = child->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
        return (result == sf::Vector2f(0, 0)) ? sf::Vector2f(1, 1) : sf::Vector2f(0, 0); // Invert the result
    }
};

#endif // BEHAVIOR_TREE_HPP