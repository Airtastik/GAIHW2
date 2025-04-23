#ifndef BEHAVIOR_TREE_HPP
#define BEHAVIOR_TREE_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <functional>
#include <queue>
#include <set>
#include "steering.hpp"

// Base class for decision tree nodes
class DecisionNode {
public:
    virtual ~DecisionNode() {}
    virtual sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                                      const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                                      int tileSize) = 0;
};
class PathDeviationNode : public DecisionNode {
    private:
        static const int MAX_SEARCH_RADIUS = 3;  // How far to look for alternative paths
    
        struct Node {
            int x, y;
            float cost;
            Node(int _x, int _y, float _cost) : x(_x), y(_y), cost(_cost) {}
            bool operator>(const Node& other) const { return cost > other.cost; }
        };
    
        sf::Vector2f findAlternativePath(const sf::Vector2f& currentPos, 
                                        const std::vector<std::vector<int>>& mapData,
                                        int tileSize) {
            int currentX = static_cast<int>(currentPos.x / tileSize);
            int currentY = static_cast<int>(currentPos.y / tileSize);
    
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
            std::set<std::pair<int, int>> closedSet;
    
            // Add initial position
            openSet.push(Node(currentX, currentY, 0));
    
            // Direction vectors for adjacent cells
            const int dx[] = {-1, 0, 1, 0, -1, -1, 1, 1};
            const int dy[] = {0, -1, 0, 1, -1, 1, -1, 1};
    
            while (!openSet.empty()) {
                Node current = openSet.top();
                openSet.pop();
    
                // Skip if already visited
                if (closedSet.find({current.x, current.y}) != closedSet.end()) {
                    continue;
                }
    
                // Mark as visited
                closedSet.insert({current.x, current.y});
    
                // Check if this is a valid path node (open space)
                if (current.x >= 0 && current.y >= 0 && 
                    static_cast<size_t>(current.y) < mapData.size() &&
                    static_cast<size_t>(current.x) < mapData[0].size() &&
                    mapData[current.y][current.x] == 0) {
                    
                    // Found valid position, return world coordinates
                    return sf::Vector2f((current.x + 0.5f) * tileSize, 
                                      (current.y + 0.5f) * tileSize);
                }
    
                // Check adjacent cells
                for (int i = 0; i < 8; ++i) {
                    int newX = current.x + dx[i];
                    int newY = current.y + dy[i];
                    float newCost = current.cost + 1.0f;
    
                    // Only consider nodes within search radius
                    if (abs(newX - currentX) <= MAX_SEARCH_RADIUS && 
                        abs(newY - currentY) <= MAX_SEARCH_RADIUS) {
                        openSet.push(Node(newX, newY, newCost));
                    }
                }
            }
    
            // No alternative found, return current position
            return currentPos;
        }
    
    public:
        sf::Vector2f makeDecision(const Kinematic& kinematic, 
                                 [[maybe_unused]] const Kinematic& targetKinematic,
                                 [[maybe_unused]] const Kinematic& enemyKinematic,
                                 const std::vector<std::vector<int>>& mapData,
                                 int tileSize) override {
            return findAlternativePath(kinematic.position, mapData, tileSize);
        }
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
    
    ~SequenceNode() override {
        for (auto child : children) {
            delete child;
        }
    }

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
    
    ~SelectorNode() override {
        for (auto child : children) {
            delete child;
        }
    }

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
    
    ~InverterNode() override {
        delete child;
    }

    sf::Vector2f makeDecision(const Kinematic& kinematic, const Kinematic& targetKinematic, 
                              const Kinematic& enemyKinematic, const std::vector<std::vector<int>>& mapData,
                              int tileSize) override {
        sf::Vector2f result = child->makeDecision(kinematic, targetKinematic, enemyKinematic, mapData, tileSize);
        return (result == sf::Vector2f(0, 0)) ? sf::Vector2f(1, 1) : sf::Vector2f(0, 0); // Invert the result
    }
};
// Add after the InverterNode class but before the final #endif
class EnemyAvoidanceNode : public DecisionNode {
    private:
        static constexpr float AVOIDANCE_RADIUS = 100.0f;
        static constexpr float SIDE_OFFSET = 50.0f;
    
        sf::Vector2f findAvoidancePoint(const Kinematic& kinematic, 
                                       const Kinematic& enemyKinematic,
                                       const std::vector<std::vector<int>>& mapData,
                                       int tileSize) {
            // Get perpendicular direction to enemy
            sf::Vector2f toEnemy = enemyKinematic.position - kinematic.position;
            sf::Vector2f perp(-toEnemy.y, toEnemy.x);
            perp = normalize(perp);
    
            // Try both sides
            std::vector<sf::Vector2f> candidates = {
                kinematic.position + perp * SIDE_OFFSET,
                kinematic.position - perp * SIDE_OFFSET
            };
    
            // Find best valid position
            for (const auto& pos : candidates) {
                int tileX = static_cast<int>(pos.x / tileSize);
                int tileY = static_cast<int>(pos.y / tileSize);
    
                if (tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() &&
                    tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() &&
                    mapData[tileY][tileX] == 0) {
                    return pos;
                }
            }
    
            // If no valid side position, back away from enemy
            return kinematic.position - normalize(toEnemy) * SIDE_OFFSET;
        }
    
        sf::Vector2f normalize(const sf::Vector2f& v) {
            float len = std::sqrt(v.x * v.x + v.y * v.y);
            if (len > 0) {
                return sf::Vector2f(v.x / len, v.y / len);
            }
            return v;
        }
    
    public:
        sf::Vector2f makeDecision(const Kinematic& kinematic, 
                                 [[maybe_unused]] const Kinematic& targetKinematic,
                                 const Kinematic& enemyKinematic,
                                 const std::vector<std::vector<int>>& mapData,
                                 int tileSize) override {
            return findAvoidancePoint(kinematic, enemyKinematic, mapData, tileSize);
        }
    };

#endif // BEHAVIOR_TREE_HPP