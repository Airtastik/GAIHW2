#ifndef UTILS_HPP
#define UTILS_HPP

#include <SFML/System/Vector2.hpp>
#include <cmath>

inline float length(const sf::Vector2f& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

inline sf::Vector2f normalize(const sf::Vector2f& v) {
    float len = length(v);
    if (len > 0) {
        return v / len;
    }
    return v;
}

#endif // UTILS_HPP
