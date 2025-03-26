// Dijkstra.cpp
#include "path.hpp"

// Calculate the Manhattan distance between two points

int manhattanHeuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

int euclideanHeuristic(int x1, int y1, int x2, int y2) {
    return static_cast<int>(std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)));
}

// Dijkstra's algorithm to find the shortest path
std::vector<sf::Vector2i> findPath(const std::vector<std::vector<int>>& mapData, sf::Vector2i start, sf::Vector2i goal, bool useAStar, HeuristicType heuristicType){
    int rows = mapData.size();
    int cols = mapData[0].size();
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Node*>> nodes(rows, std::vector<Node*>(cols, nullptr));

    // Direction vectors for 4 neighbors (up, down, left, right)
    const std::vector<sf::Vector2i> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    // Priority queue to store nodes, with the lowest cost first
    std::priority_queue<Node*, std::vector<Node*>, std::greater<Node*>> pq;

    // Initialize the start node
    int heuristic = 0;
    if (useAStar) {
        if (heuristicType == HeuristicType::Manhattan) {
            heuristic = manhattanHeuristic(start.x, start.y, goal.x, goal.y);
        } else if (heuristicType == HeuristicType::Euclidean) {
            heuristic = euclideanHeuristic(start.x, start.y, goal.x, goal.y);
        }
    }

    Node* startNode = new Node(start.x, start.y, 0, heuristic);
    pq.push(startNode);
    nodes[start.y][start.x] = startNode;

    while (!pq.empty()) {
        Node* currentNode = pq.top();
        pq.pop();

        int x = currentNode->x;
        int y = currentNode->y;

        // If we reached the goal, reconstruct the path
        if (x == goal.x && y == goal.y) {
            std::vector<sf::Vector2i> path;
            while (currentNode != nullptr) {
                path.push_back({currentNode->x, currentNode->y});
                currentNode = currentNode->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Mark the node as visited
        visited[y][x] = true;

        // Explore neighbors
        for (const auto& dir : directions) {
            int newX = x + dir.x;
            int newY = y + dir.y;

            // Check if the neighbor is within bounds and not visited, and it's passable
            if (newX >= 0 && newX < cols && newY >= 0 && newY < rows && !visited[newY][newX] && mapData[newY][newX] == 0) {
                int newCost = currentNode->cost + 1;
                int newHeuristic = 0;
                if (useAStar) {
                    if (heuristicType == HeuristicType::Manhattan) {
                        newHeuristic = manhattanHeuristic(newX, newY, goal.x, goal.y);
                    } else if (heuristicType == HeuristicType::Euclidean) {
                        newHeuristic = euclideanHeuristic(newX, newY, goal.x, goal.y);
                    }
                }
                Node* neighborNode = new Node(newX, newY, newCost, newHeuristic, currentNode);
                
                // If this is the best path to the neighbor, add it to the queue
                pq.push(neighborNode);
                nodes[newY][newX] = neighborNode;
            }
        }
    }

    // Return an empty path if no path found
    return {};
}
