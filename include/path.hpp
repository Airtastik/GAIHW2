// path.hpp
#ifndef PATH_HPP
#define PATH_HPP
#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>

enum class HeuristicType {
    Manhattan,   // Manhattan distance heuristic
    Euclidean   // Euclidean distance heuristic
};

// Node structure to hold the tile's data for pathfinding
struct Node {
    int x, y;           // Tile position
    int cost;           // Cost to reach this node
    int heuristic;      // Heuristic (Manhattan or Euclidean distance)
    Node* parent;       // Parent node in the path

    Node(int x, int y, int cost, int heuristic, Node* parent = nullptr)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

    // The priority queue will sort nodes by cost + heuristic (f-score) for A*, just cost for Dijkstra
    bool operator>(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

// Dijkstra's algorithm to find the shortest path
std::vector<sf::Vector2i> findPath(const std::vector<std::vector<int>>& mapData, sf::Vector2i start, sf::Vector2i goal, bool useAStar, HeuristicType heuristicType);


#endif // path_HPP
