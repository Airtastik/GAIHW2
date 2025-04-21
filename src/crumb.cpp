#include "crumb.hpp"

crumb::crumb(int id)
{
    //set initial position and size breadcrumbs   
    this->id = id;         
    this->setRadius(5.f);
    this->setFillColor(sf::Color(0, 255, 0, 255));
    this->setPosition(-1, -1);
}

//tell breadcrumb to render self, using current render window
void crumb::draw(sf::RenderWindow* window)
{
    window->draw(*this);
}

//set position of breadcrumb
void crumb::drop(float x, float y)
{
    this->setPosition(x, y);
}

//set position of breadcrumb
void crumb::drop(sf::Vector2f position)
{
    this->setPosition(position);
}