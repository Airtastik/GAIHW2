#ifndef PATHMAP_H
#define PATHMAP_H

#include <vector>
#include "vertex.h"

class PathMap {
public:
    // Struct to create map from 2D array
    struct CreateMap {
        // Constructor taking a 2D vector representing the map
        CreateMap(const std::vector<std::vector<int>>& mapData) {
            // Convert input map to vertices
            for (int y = 0; y < mapData.size(); ++y) {
                std::vector<Vertex> rowVertices;
                for (int x = 0; x < mapData[y].size(); ++x) {
                    // Create vertex: 0 is accessible, 1 is not
                    rowVertices.emplace_back(x, y, mapData[y][x] == 0);
                }
                vertices.push_back(rowVertices);
            }
        }

        // 2D vector of vertices
        std::vector<std::vector<Vertex>> vertices;

        // Get dimensions
        int getWidth() const {
            return vertices.empty() ? 0 : vertices[0].size();
        }

        int getHeight() const {
            return vertices.size();
        }

        // Const and non-const versions of getVertex
        Vertex& getVertex(int x, int y) {
            return vertices[y][x];
        }

        const Vertex& getVertex(int x, int y) const {
            return vertices[y][x];
        }

        // Check if a position is accessible
        bool isAccessible(int x, int y) const {
            if (x < 0 || y < 0 || y >= vertices.size() || x >= vertices[y].size()) {
                return false;
            }
            return vertices[y][x].accessible;
        }
    };

    // Constructor taking CreateMap
    PathMap(const CreateMap& mapCreator) : mapData(mapCreator) {}

    // Const and non-const getters for map data
    CreateMap& getMapData() {
        return mapData;
    }

    const CreateMap& getMapData() const {
        return mapData;
    }

private:
    CreateMap mapData;
};

#endif // PATHMAP_H