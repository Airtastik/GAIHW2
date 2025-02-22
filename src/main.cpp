// Tristan Hall CSC 584
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

#include <iostream>
#include <vector>
const sf::Vector2f TOP_RIGHT = sf::Vector2f(600, 0);
const sf::Vector2f BOT_RIGHT = sf::Vector2f(600, 600);
const sf::Vector2f BOT_LEFT = sf::Vector2f(0, 600);
const sf::Vector2f TOP_LEFT = sf::Vector2f(0, 0);

// Start from the left side, center vertically

float getAngle(const sf::Vector2f& from, const sf::Vector2f& to) {
    return atan2(to.y - from.y, to.x - from.x) * 180 / 3.14159f;
}

// Kinematic structure
struct Kinematic {
    sf::Vector2f position;
    sf::Vector2f velocity;
    float orientation;
    float rotation;
};

class SteeringBehavior {
public:
    virtual void apply(Kinematic& character, const Kinematic& target, float deltaTime) = 0;
    virtual ~SteeringBehavior() = default;
};

// Position matching behavior
class PositionMatchSteering : public SteeringBehavior {
public:
    void apply(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.position = target.position;
    }
};

// Orientation matching behavior
class OrientationMatchSteering : public SteeringBehavior {
public:
    void apply(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.orientation = target.orientation;
    }
};

// Velocity matching behavior
class VelocityMatchSteering : public SteeringBehavior {
public:
    void apply(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.velocity = target.velocity;
    }
};

// Rotation matching behavior
class RotationMatchSteering : public SteeringBehavior {
public:
    void apply(Kinematic& character, const Kinematic& target, float deltaTime) override {
        character.rotation = target.rotation;
    }
};

class crumb : sf::CircleShape
{
    public:
        crumb(int id)
        {
            //set initial position and size breadcrumbs   
            this->id = id;         
            this->setRadius(10.f);
            this->setFillColor(sf::Color(0, 0, 255, 255));
            this->setPosition({-100, -100});
        }

        //tell breadcrumb to render self, using current render window
        void draw(sf::RenderWindow* window)
        {
            window->draw(*this);
        }

        //set position of breadcrumb
        void drop(float x, float y)
        {
            this->setPosition({x, y});
        }

        //set position of breadcrumb
        void drop(sf::Vector2f position)
        {
            this->setPosition(position);
        }

    private:
        int id;
};
class boid
{
    public:
        boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs)
        {
            window = w;
            speed = 0.1f;
            drop_timer = 100.f;
            velocity = sf::Vector2f(1, 0);
            target_idx = 1;
            crumb_idx = 0;
            target = TOP_RIGHT;
            position = sf::Vector2f(0, 0);            
            sprite.setTexture(tex);
            sprite.setScale(0.1f, 0.1f);
            breadcrumbs = crumbs;
        }

        void draw()
        {            
            window->draw(sprite);
        }  

        void move()
        {
            //basic timer for leaving breadcrumbs
            if (drop_timer > 0)
            {
                drop_timer -= 0.1f;
            }
            else
            {
                drop_timer = 100.f;
                breadcrumbs->at(crumb_idx).drop(position);

                if (crumb_idx < 9)
                    crumb_idx++;
                else
                    crumb_idx = 0;
            }

            //if reached target, switch to next target
            if (reached(position, target))
            {
                switch (target_idx)
                {
                    case 0:
                        target = TOP_RIGHT;
                        target_idx = 1;
                        velocity = sf::Vector2f(1, 0);
                        orientation = 0;                     
                        break;
                    case 1:
                        target = BOT_RIGHT;
                        target_idx = 2;
                        velocity = sf::Vector2f(0, 1);
                        orientation = 90;
                        break;
                    case 2:
                        target = BOT_LEFT;
                        target_idx = 3;
                        velocity = sf::Vector2f(-1, 0);
                        orientation = 180;
                        break;
                    case 3:
                        target = TOP_LEFT;
                        target_idx = 0;
                        velocity = sf::Vector2f(0, -1);
                        orientation = 270;
                        break;
                }

                sprite.setRotation(orientation);
            }
            //else move toward target
            else
            {
                position += velocity * speed;
                sprite.setPosition(position);
            }
        }

        //check if boid has reached target position
        bool reached(sf::Vector2f pos, sf::Vector2f tar)        
        {
            sf::Vector2f dis = tar - pos;
            if (abs(dis.x + dis.y) <= 1)
                return true;
            return false;
        }

    private:
        //indice variables
        int target_idx;
        int crumb_idx;
        
        //float variables
        float drop_timer;
        float speed;
        float orientation;
        
        //renderable objects
        sf::Sprite sprite;
        sf::RenderWindow* window;    
        
        //vector variables
        sf::Vector2f target;    
        sf::Vector2f position;
        sf::Vector2f velocity;

        //point of breadcrumbs
        std::vector<crumb>* breadcrumbs;
};
void moveAndRotateSprite(sf::Sprite& sprite,int orentation, float move)
{
    // Get the deltaTime for this specific sprite (time since last update)
   
    if (orentation == 0)
    {
        sprite.move({move, 0.f});
     
    }
    else if (orentation == 1)  // After 2 seconds, start rotation
    {
        sprite.setRotation((90));
        sprite.move({0.f, move});
  
    }
    else if (orentation == 2)
    {
        sprite.setRotation((180));
        sprite.move({-move, 0.f});
   
    }
    else
    {
        sprite.setRotation((270));
        sprite.move({0.f, -move});

    }

}

int main()
{
    sf::RenderWindow window(sf::VideoMode({600, 600}), "Homework 2");
    sf::Texture texture;
    
    if (!texture.loadFromFile("boid.png")) // Ensure the texture loads properly
        return -1;

    
    sf::Sprite sprite(texture); 
    sf::Clock clock;
    float duration = 2.0f; // Move across the screen in 1 second
    float speed = 640.0f / duration; // Pixels per second
    float speed2 = 480.0f / duration;
    
    sf::Time elapsedTime = sf::Time::Zero; // Track time

    std::vector<sf::Sprite> sprites;
    sprites.push_back(sprite); // Add initial sprite

    while (window.isOpen())
    {
                sf::Event event;
        while (window.pollEvent(event))
        {
            // Request for closing the window
            if (event.type == sf::Event::Closed)
                window.close();
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
            {
               sf::Vector2i position = sf::Mouse::getPosition();
            }
                
           
                
            // The escape key was pressed
            if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))
                window.close();
        }
            
        window.clear(sf::Color::Green);
        for (const auto& s : sprites) {
            window.draw(s);
        }
        window.display();
    }

}