#ifndef VERTEX_H
#define VERTEX_H

#include <cmath>

class Vertex {
public:
    // Constructors
    Vertex() : gridX(0), gridY(0), pixelX(0), pixelY(0), accessible(true) {}
    
    Vertex(int x, int y, bool isAccessible) : 
        gridX(x), 
        gridY(y), 
        pixelX(x * 10),    // Convert grid coordinate to pixel coordinate
        pixelY(y * 10),    // 10 pixels per grid unit
        accessible(isAccessible)
    {}

    // Grid coordinates
    int gridX;
    int gridY;

    // Pixel coordinates (scaled by 10)
    int pixelX;
    int pixelY;

    // Accessibility flag
    bool accessible;

    // Calculate distance between two vertices in pixels
    float distanceTo(const Vertex& other) const {
        int dx = pixelX - other.pixelX;
        int dy = pixelY - other.pixelY;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Comparison operators
    bool operator==(const Vertex& other) const {
        return gridX == other.gridX && gridY == other.gridY;
    }

    bool operator!=(const Vertex& other) const {
        return !(*this == other);
    }
};

#endif // VERTEX_H