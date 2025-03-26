#ifndef PATHFIND
#define PATHFIND

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include "vertex.h"
#include "pathmap.h"

class Path {
public:
    // Enum for pathfinding algorithm types
    enum class Algorithm {
        ASTAR_MANHATTAN,
        ASTAR_EUCLIDEAN,
        DIJKSTRA
    };

    // Pathfinding result structure
    struct PathResult {
        std::vector<Vertex> route;
        float totalCost;
        bool success;

        PathResult() : totalCost(0.0f), success(false) {}
    };

private:
    // Struct for priority queue in A* and Dijkstra's
    struct PathNode {
        Vertex* vertex;
        float gCost;      // Cost from start
        float fCost;      // Total estimated cost (for A*)
        Vertex* parent;   // Previous node in the path

        PathNode() : vertex(nullptr), gCost(0.0f), fCost(0.0f), parent(nullptr) {}

        PathNode(Vertex* v, float g, float f, Vertex* p)
            : vertex(v), gCost(g), fCost(f), parent(p) {}

        bool operator>(const PathNode& other) const {
            return fCost > other.fCost;
        }
    };

    // Heuristic function types
    float manhattanDistance(const Vertex& start, const Vertex& goal) const {
        return std::abs(start.gridX - goal.gridX) + 
               std::abs(start.gridY - goal.gridY);
    }

    float euclideanDistance(const Vertex& start, const Vertex& goal) const {
        float dx = start.gridX - goal.gridX;
        float dy = start.gridY - goal.gridY;
        return std::sqrt(dx * dx + dy * dy);
    }

    std::vector<Vertex*> getNeighbors(const PathMap& pathMap, Vertex* currentVertex);

    PathResult aStarSearch(const PathMap& pathMap, Vertex& start, Vertex& goal, float (Path::*heuristic)(const Vertex&, const Vertex&) const);

    PathResult dijkstraSearch(const PathMap& pathMap, Vertex& start, Vertex& goal);

public:
    // Main pathfinding method
    PathResult findPath(const PathMap& pathMap, Vertex& start, Vertex& goal, Algorithm algorithm = Algorithm::ASTAR_MANHATTAN);
};

#endif // PATHFIND
