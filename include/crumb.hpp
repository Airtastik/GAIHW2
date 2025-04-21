
#ifndef CRUMB_HPP
#define CRUMB_HPP

#include <SFML/Graphics.hpp>

class crumb : public sf::CircleShape
{
public:
    crumb(int id);
    void draw(sf::RenderWindow* window);
    void drop(float x, float y);
    void drop(sf::Vector2f position);

private:
    int id;
};

#endif // CRUMB_HPP