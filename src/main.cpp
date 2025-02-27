// 
// @author Tristan Hall (tdhall6@ncsu.edu)
// I used ChatGPT to assist with creation
#include "steering.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <random>
const sf::Vector2f TOP_RIGHT = sf::Vector2f(550, 0);
const sf::Vector2f BOT_RIGHT = sf::Vector2f(550, 550);
const sf::Vector2f BOT_LEFT = sf::Vector2f(0, 550);
const sf::Vector2f TOP_LEFT = sf::Vector2f(0, 0);

const float SEPARATION_RADIUS = 150.0f;
const float ALIGNMENT_RADIUS = 100.0f;
const float COHESION_RADIUS = 100.0f;
const float SEPARATION_WEIGHT = 20.5f;
const float ALIGNMENT_WEIGHT = 3.0f;
const float COHESION_WEIGHT = 3.0f;
//Breadcrumb class
class crumb : sf::CircleShape
{
    public:
        crumb(int id)
        {
            //set initial position and size breadcrumbs   
            this->id = id;         
            this->setRadius(5.f);
            this->setFillColor(sf::Color(0, 0, 255, 255));
            this->setPosition(-1, -1);
        }

        //tell breadcrumb to render self, using current render window
        void draw(sf::RenderWindow* window)
        {
            window->draw(*this);
        }

        //set position of breadcrumb
        void drop(float x, float y)
        {
            this->setPosition(x, y);
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
            boid(sf::RenderWindow* w, sf::Texture& tex, std::vector<crumb>* crumbs, Kinematic boidKin)
            : arrive(new Arrive()), align(new Align()){
                window = w;
                kinematic = boidKin;
                speed = 1.0f;
                drop_timer = 1000.f;
                crumb_idx = 0;
                sprite.setTexture(tex);
                sprite.setScale(0.02f, 0.02f);
                breadcrumbs = crumbs;
            }
        
            ~boid() {
                delete arrive;
                delete align;
            }
        
            void draw() {
                window->draw(sprite);
            }
            
            void update(float deltaTime, Kinematic targetkin) {
               targetKinematic = targetkin;
                move(deltaTime);
            }
        
         void move(float deltaTime)
        {
            if (drop_timer > 0)
            {
                drop_timer -= 0.1f;
            }
            else
            {
                drop_timer = 1000.f;
                breadcrumbs->at(crumb_idx).drop(kinematic.position);

                if (crumb_idx < 14)
                    crumb_idx++;
                else
                    crumb_idx = 0;
            }


            //basic timer for leaving breadcrumbs
            //if reached target, switch to next target
            if (reached(kinematic.position, targetKinematic.position))
            {
                switch (target_idx)
                {
                    case 0:
                        targetKinematic.position = TOP_RIGHT;
                        target_idx = 1;
                        targetKinematic.velocity = sf::Vector2f(1, 0);
                        targetKinematic.orientation = 0;                     
                        break;
                    case 1:
                        targetKinematic.position = BOT_RIGHT;
                        target_idx = 2;
                        targetKinematic.velocity = sf::Vector2f(0, 1);
                        targetKinematic.orientation = 1.5708;
                        break;
                    case 2:
                        targetKinematic.position = BOT_LEFT;
                        target_idx = 3;
                        targetKinematic.velocity = sf::Vector2f(-1, 0);
                        targetKinematic.orientation= 3.14159;
                        break;
                    case 3:
                        targetKinematic.position = TOP_LEFT;
                        target_idx = 0;
                        targetKinematic.velocity = sf::Vector2f(0, -1);
                        targetKinematic.orientation = 4.71239;
                        break;
                }
            }
            //else move toward target
            else
            {
                sf::Vector2f acceleration = arrive->calculate(kinematic, targetKinematic);
                float angularAcceleration = align->calculate(kinematic, targetKinematic);
        
                // Update velocity and position
                kinematic.velocity += acceleration * deltaTime;
                if (std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) > kinematic.maxSpeed)
                    kinematic.velocity /= std::sqrt(kinematic.velocity.x * kinematic.velocity.x + kinematic.velocity.y * kinematic.velocity.y) / kinematic.maxSpeed;
        
                kinematic.position += kinematic.velocity * deltaTime;
        
                // Update rotation
                kinematic.rotation +=  kinematic.maxRotation * deltaTime;
                if (std::abs(kinematic.rotation) > kinematic.maxRotation){
                    kinematic.rotation = kinematic.maxRotation * (kinematic.rotation / std::abs(kinematic.rotation));
                    
                kinematic.orientation += kinematic.rotation * deltaTime;}
                float angle = std::atan2(kinematic.velocity.y, kinematic.velocity.x) * (180.0f / 3.1459); 
                sprite.setRotation(angle);
                if(kinematic.position.x > 1002){
                    kinematic.position.x = 0;
                }
                if(kinematic.position.x < -2){
                    kinematic.position.x = 1000;
                }
                if(kinematic.position.y < -2){
                    kinematic.position.y = 1000;
                }
                if(kinematic.position.y > 1002){
                    kinematic.position.y = 0;
                }
                sprite.setPosition(kinematic.position);
                  //basic timer for leaving breadcrumb

                std::cout << "m Position: (" << crumb_idx << ", " << kinematic.orientation  << ")" << std::endl;
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
        
        int crumb_idx;
        int target_idx;
        float drop_timer;
        float speed;
    
        sf::Sprite sprite;
        sf::RenderWindow* window;
        std::vector<crumb>* breadcrumbs;
    
        Kinematic kinematic;
        Kinematic targetKinematic;
        Arrive* arrive;
        Align* align;
};
void flocking(){
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Flockingboids Simulation");
    sf::Texture texture;
    if (!texture.loadFromFile("boid.png")) {
        std::cout << "No file found" << std::endl;
    }

    Kinematic targetKinematic;
    targetKinematic.position = sf::Vector2f(500, 500);

    std::vector<std::unique_ptr<flockingboid>> flockingboids;
    for (int j = 0; j < 20; j++) {
        Kinematic boidKinematic;
        boidKinematic.position = sf::Vector2f(rand() % 1000, rand() % 1000);
        boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        boidKinematic.maxSpeed = 200.0f;
        boidKinematic.arrivalRadius = 5.0f;
        flockingboids.push_back(std::make_unique<flockingboid>(&window, texture, boidKinematic));
    }

    sf::Clock clock;
    while (window.isOpen()) {
        sf::Time deltaTime = clock.restart();
        float dt = deltaTime.asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        targetKinematic.position = sf::Vector2f(mousePos);

        window.clear(sf::Color(255, 255, 255, 255));
        for (auto& b : flockingboids) {
            b->update(dt, flockingboids, targetKinematic);
            b->draw();
        }
        window.display();
    }

};
void AlignArriveAndWander(){
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "SFML works!");
    sf::Texture texture;
    sf::Vector2f lastMousePos(400, 300);
    VelocitySteering velocityMatch;
    Kinematic targetKinematic;
    targetKinematic.position = sf::Vector2f(500, 500);
    targetKinematic.velocity = sf::Vector2f(0, 0);

    Kinematic boidKinematic;
    boidKinematic.position = sf::Vector2f(0, 0);
    boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
    boidKinematic.maxSpeed = 350.0f;
    boidKinematic.maxAcceleration = 100.0f;
    boidKinematic.maxRotation = 5.0f;
    boidKinematic.slowRadius = 3.0f;
    boidKinematic.arrivalRadius = 3.0f;
    boidKinematic.maxRotation = 5.0f;
    boidKinematic.fleeRadius = 5.0f;
    float wonderdrop_timer = 1000.f;
    int numBoids = 1; // number of boids on screen
    /* Runtype 1: mouse click targeting
       Runtype 2: mouse movement targeting
       Runtype 3: wonder random
       Runtype 4: wonder edge
    */
    int runtype = 2;
    //
    //
    Arrive arrive;
    Align align;
    // load a 32x32 rectangle that starts at (10, 10)
    if (!texture.loadFromFile("boid.png"))
    {
        // error...
    }
    std::vector<crumb> breadcrumbs;
    for(int i = 0; i < 15; i++)
    {
        crumb c(i);
        breadcrumbs.push_back(c);
    }   
    sf::Clock clock;
    std::vector<std::unique_ptr<boid>> boids;
    for (int j = 0; j < numBoids; j++) {
        boids.push_back(std::make_unique<boid>(&window, texture, &breadcrumbs,boidKinematic));
        boidKinematic.position.x +=100.0f;
    }
    while (window.isOpen())
    {
        sf::Time deltaTime = clock.restart();
        float dt = deltaTime.asSeconds();
        sf::Event event;
        sf::Vector2f prevMousePosition; // 
        bool firstMouseUpdate = true;
        sf::Time elapsed2;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed){
                window.close();          
            }
            if(runtype == 1){
           // mouse click Target
                if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left){
                    std::cout << "mouse" << std::endl;
                    sf::Vector2i localPosition = sf::Mouse::getPosition(window);
                    targetKinematic.position = sf::Vector2f(localPosition);
                    std::cout << "dt: (" << dt* 100 << ")" << std::endl;
                    for (auto& b : boids) {
                        b->update(dt,targetKinematic);
                    }   
                }  
            }
            else if(runtype == 2){
                 // Get the current mouse position
                if (event.type == sf::Event::MouseMoved) {
                    // Get the current mouse position
                    sf::Vector2i localPosition = sf::Mouse::getPosition(window);
                    sf::Vector2f currentMousePosition(localPosition.x, localPosition.y);
                    sf::Vector2f mouseVelocity;
                    if (!firstMouseUpdate) {
                        mouseVelocity = (currentMousePosition - prevMousePosition) / dt; // velocity = displacement / time
                    } else {
                        mouseVelocity = sf::Vector2f(0, 0); // No movement initially
                        firstMouseUpdate = false;
                    }
                    // Update the target kinematic properties
                    targetKinematic.position = currentMousePosition;
                    targetKinematic.velocity = mouseVelocity;
                    // Store current mouse position for next frame
                    prevMousePosition = currentMousePosition;
                    for (auto& b : boids) {
                        b->update(dt,targetKinematic);
                    }
                }

            }else if (runtype == 3 || runtype == 4) { 
                static float accumulatedTime = 0.0f;
                static size_t currentIndex = 0; // For runtype 4
                std::cout << accumulatedTime << std::endl;
               
        
                if (accumulatedTime < 100)
                {
                    accumulatedTime += 1;
                }
                else
                {
                    std::cout << "1 second elapsed" << std::endl;
        
                    if (runtype == 3) {
                        // Generate random position (same as before)
                        static std::random_device rd;
                        static std::mt19937 gen(rd());
                        std::uniform_int_distribution<> distribX(0, window.getSize().x);
                        std::uniform_int_distribution<> distribY(0, window.getSize().y);
                        int randomX = distribX(gen);
                        int randomY = distribY(gen);
                        sf::Vector2f newPosition(randomX, randomY);
                        targetKinematic.position = newPosition;
                        std::cout << "1 second elapsed" << std::endl;
        
        
                    } else if (runtype == 4) {
                        static const std::vector<sf::Vector2f> locatArr = {
                            {500.0f, -500.0f}, {500.0f, 1500.0f}, {1500.0f, 500.0f}, {-500.0f, 1500.0f}
                        };
                        targetKinematic.position = locatArr[currentIndex];
                        currentIndex = (currentIndex + 1) % locatArr.size();
                    }
                    accumulatedTime = 0;
                }
                    std::cout << "Target Position: (" << boidKinematic.position.x << ", " << boidKinematic.position.y << ")" << std::endl;
                   
                    for (auto& b : boids) {
                        b->update(dt,targetKinematic);
                    }
        
                     // Subtract *after* the update!
            }
            
            }
            for(int i = 0; i < breadcrumbs.size(); i++)
            {
                breadcrumbs[i].draw(&window);
            }
            window.clear(sf::Color(255,255,255,255));      
            for (const auto& b : boids) {
                b->move(dt);
                b->draw();
                for(int i = 0; i < breadcrumbs.size(); i++)
                {
                    breadcrumbs[i].draw(&window);
                }
            }
            
        window.display();
            
    }
}
bool flock = false;
int main()
{
    if(flock == true)
     flocking();
    else
    AlignArriveAndWander();
    return 0;
};