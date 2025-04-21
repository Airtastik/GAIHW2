// Dijkstra.hpp
#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <cmath>

// Node structure to hold the tile's data for pathfinding
struct Node {
    int x, y;           // Tile position
    int cost;           // Cost to reach this node
    int heuristic;      // Heuristic (Manhattan distance)
    Node* parent;       // Parent node in the path

    Node(int x, int y, int cost, int heuristic, Node* parent = nullptr)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

    // The priority queue will sort nodes by cost + heuristic (f-score)
    bool operator>(const Node& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

// Dijkstra's algorithm to find the shortest path
std::vector<sf::Vector2i> dijkstra(const std::vector<std::vector<int>>& mapData, sf::Vector2i start, sf::Vector2i goal);

#endif // DIJKSTRA_HPP
