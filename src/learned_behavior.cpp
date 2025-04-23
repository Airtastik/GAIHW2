#include "learned_behavior.hpp"
#include "utils.hpp"
#include <algorithm>
#include <numeric>

void LearnedBehavior::recordBehavior(const Kinematic& self, const Kinematic& player,
                                   const std::vector<std::vector<int>>& mapData,
                                   const sf::Vector2f& action) {
    if (trainingData.size() >= MAX_SAMPLES) return;
    
    BehaviorSample sample;
    sf::Vector2f toPlayer = player.position - self.position;
    sample.distanceToPlayer = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    sample.angleToPlayer = std::atan2(toPlayer.y, toPlayer.x);
    sample.relativeSpeed = std::sqrt(self.velocity.x * self.velocity.x + 
                                   self.velocity.y * self.velocity.y);
    
    // Check if near wall
    int tileX = static_cast<int>(self.position.x / 100);
    int tileY = static_cast<int>(self.position.y / 100);
    sample.isNearWall = (tileX >= 0 && static_cast<size_t>(tileX) < mapData[0].size() &&
                        tileY >= 0 && static_cast<size_t>(tileY) < mapData.size() &&
                        mapData[tileY][tileX] == 1);
    
    sample.selfKinematic = self;
    sample.playerKinematic = player;
    sample.mapData = mapData;
    sample.actionTaken = action;
    trainingData.push_back(sample);
}

sf::Vector2f LearnedBehavior::makeDecision(const Kinematic& self, const Kinematic& player,
                                         [[maybe_unused]] const std::vector<std::vector<int>>& mapData) {
    if (!rootNode) return sf::Vector2f(0, 0);
    
    BehaviorSample currentState;
    sf::Vector2f toPlayer = player.position - self.position;
    currentState.distanceToPlayer = std::sqrt(toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y);
    currentState.angleToPlayer = std::atan2(toPlayer.y, toPlayer.x);
    currentState.relativeSpeed = std::sqrt(self.velocity.x * self.velocity.x + 
                                         self.velocity.y * self.velocity.y);
    
    DecisionNode* current = rootNode;
    while (!current->isLeaf) {
        std::vector<float> features = extractFeatures(currentState);
        if (features[current->featureIndex] <= current->threshold)
            current = current->left;
        else
            current = current->right;
    }
    
    return current->action;
}

void LearnedBehavior::deleteTree(DecisionNode* node) {
    if (node) {
        deleteTree(node->left);
        deleteTree(node->right);
        delete node;
    }
}

void LearnedBehavior::trainDecisionTree() {
    if (trainingData.empty()) return;
    deleteTree(rootNode);
    rootNode = buildTree(trainingData, 0);
}

std::vector<float> LearnedBehavior::extractFeatures(const BehaviorSample& sample) {
    std::vector<float> features;
    features.push_back(sample.distanceToPlayer);
    features.push_back(sample.angleToPlayer);
    features.push_back(sample.relativeSpeed);
    features.push_back(sample.isNearWall ? 1.0f : 0.0f);
    return features;
}

LearnedBehavior::DecisionNode* LearnedBehavior::buildTree(const std::vector<BehaviorSample>& samples, int depth) {
    if (samples.empty() || depth > 10) return nullptr;

    auto node = new DecisionNode();

    // If all samples have similar actions, make a leaf node
    bool similarActions = true;
    const sf::Vector2f& firstAction = samples[0].actionTaken;
    for (const auto& sample : samples) {
        sf::Vector2f diff_vec = sample.actionTaken - firstAction;
        float diff = length(diff_vec);  // Now using class member function
        if (diff > 0.1f) {
            similarActions = false;
            break;
        }
    }

    if (similarActions || samples.size() < 5) {
        node->isLeaf = true;
        // Use average action as the leaf node's action
        sf::Vector2f avgAction(0, 0);
        for (const auto& sample : samples) {
            avgAction += sample.actionTaken;
        }
        node->action = avgAction / static_cast<float>(samples.size());
        return node;
    }

    // Find best split
    float bestScore = std::numeric_limits<float>::max();
    int bestFeature = 0;
    float bestThreshold = 0;

    // Try each feature
    for (int feature = 0; feature < 4; ++feature) {
        // Get all values for this feature
        std::vector<float> values;
        for (const auto& sample : samples) {
            values.push_back(extractFeatures(sample)[feature]);
        }

        // Try median as threshold
        std::sort(values.begin(), values.end());
        float threshold = values[values.size() / 2];

        // Split samples
        std::vector<BehaviorSample> leftSamples, rightSamples;
        for (const auto& sample : samples) {
            if (extractFeatures(sample)[feature] <= threshold) {
                leftSamples.push_back(sample);
            } else {
                rightSamples.push_back(sample);
            }
        }

        // Calculate split score (simple variance of actions)
        float score = 0;
        if (!leftSamples.empty() && !rightSamples.empty()) {
            score = static_cast<float>(leftSamples.size() * rightSamples.size());
            if (score < bestScore) {
                bestScore = score;
                bestFeature = feature;
                bestThreshold = threshold;
            }
        }
    }

    // Create decision node
    node->featureIndex = bestFeature;
    node->threshold = bestThreshold;

    // Split samples
    std::vector<BehaviorSample> leftSamples, rightSamples;
    for (const auto& sample : samples) {
        if (extractFeatures(sample)[bestFeature] <= bestThreshold) {
            leftSamples.push_back(sample);
        } else {
            rightSamples.push_back(sample);
        }
    }

    // Recursively build children
    node->left = buildTree(leftSamples, depth + 1);
    node->right = buildTree(rightSamples, depth + 1);

    return node;
}
