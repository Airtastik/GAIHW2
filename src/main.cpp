// 
// @author Tristan Hall (tdhall6@ncsu.edu)
// I used ChatGPT to assist with creation
#include "steering.h"
#include "vertex.h"
#include "pathmap.h"
#include "PATHFIND.h"
#include "path.hpp"
#include "crumb.hpp"
#include "boid.hpp"
#include "enemy_boid.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <queue>

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
/**/
void flocking(){
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Flockingboids Simulation");
    sf::Texture texture;
    if (!texture.loadFromFile("assets/boid.png")) {
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
int testPath() {
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Dijkstra & A* Pathfinding");
    sf::Texture texture;
    sf::Vector2f lastMousePos(400, 300);
    VelocitySteering velocityMatch;
    Kinematic targetKinematic;
    targetKinematic.position = sf::Vector2f(500, 500);
    targetKinematic.velocity = sf::Vector2f(0, 0);

    Kinematic boidKinematic;
    boidKinematic.position = sf::Vector2f(0, 0);
    boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
    boidKinematic.maxSpeed = 60.0f;
    boidKinematic.maxAcceleration = 100.0f;
    boidKinematic.maxRotation = 130.0f;
    boidKinematic.slowRadius = 1.0f;
    boidKinematic.arrivalRadius = 1.0f;
    boidKinematic.fleeRadius = 5.0f;
    float wonderdrop_timer = 1000.f;
    int numBoids = 1; // number of boids on screen

    //
    //
    Arrive arrive;
    Align align;
    // load a 32x32 rectangle that starts at (10, 10)
    if (!texture.loadFromFile("assets/boid.png"))
    {
        // error...
    }
    std::vector<crumb> breadcrumbs;
    for(int i = 0; i < 100; i++)
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

    // Define the 10x10 map
    std::vector<std::vector<int>> mapData = {
        {0,1,0,0,0,0,0,0,0,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,1,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,1,0}    
    };

    // Tile size for visualization
    const int tileSize = 100;

    // Create a vertex array to hold the vertices of the map
    sf::VertexArray vertices(sf::Quads);

    // Iterate through the map and create vertices for each tile
    for (int y = 0; y < mapData.size(); ++y) {
        for (int x = 0; x < mapData[y].size(); ++x) {
            sf::Vector2f topLeft(x * tileSize, y * tileSize);
            sf::Vector2f topRight((x + 1) * tileSize, y * tileSize);
            sf::Vector2f bottomRight((x + 1) * tileSize, (y + 1) * tileSize);
            sf::Vector2f bottomLeft(x * tileSize, (y + 1) * tileSize);

            sf::Vector2i start(0, 0);  // Starting position (top-left corner)
            sf::Vector2i goal(9, 9);   // Goal position (bottom-right corner)
    
            // Check if the current tile is the start or goal
            if (x == start.x && y == start.y) {
                // If it's the start, make it pink
                vertices.append(sf::Vertex(topLeft, sf::Color::Red)); // red
                vertices.append(sf::Vertex(topRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Red));
            } else if (x == goal.x && y == goal.y) {
                // If it's the goal, make it green
                vertices.append(sf::Vertex(topLeft, sf::Color::Green)); // Green
                vertices.append(sf::Vertex(topRight, sf::Color::Green));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Green));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Green));
            }

            // If the tile is passable (0), make it White
            if (mapData[y][x] == 0) {
                vertices.append(sf::Vertex(topLeft, sf::Color::White));
                vertices.append(sf::Vertex(topRight, sf::Color::White));
                vertices.append(sf::Vertex(bottomRight, sf::Color::White));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::White));
            }
            // If the tile is impassable (1), make it Black
            else {
                vertices.append(sf::Vertex(topLeft, sf::Color::Black));
                vertices.append(sf::Vertex(topRight, sf::Color::Black));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Black));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Black));
            }
        }
    }

    // Define start and goal positions
    sf::Vector2i start(0, 0);  // Starting position (top-left corner)
    sf::Vector2i goal(9, 9);   // Goal position (bottom-right corner)// Stores the path
    int currentTargetIndex = 0; // Keeps track of the current target index in the path

// Define start and goal positions (can be fixed or random)


    // Generate the path (can be done during initialization or periodically)

    // Call the findPath function with A* and Euclidean heuristic (change to Manhattan for that heuristic)
    bool useAStar = false; // Set this to false to use Dijkstra
    HeuristicType heuristicType = HeuristicType::Euclidean; // Choose between Manhattan and Euclidean
    std::vector<sf::Vector2i> path = findPath(mapData, start, goal, useAStar, heuristicType);

    // Create a separate vertex array for the path
    sf::VertexArray pathVertices(sf::Quads);
  
    for (const auto& p : path) {
        sf::Vector2f topLeft(p.x * tileSize, p.y * tileSize);
        sf::Vector2f topRight((p.x + 1) * tileSize, p.y * tileSize);
        sf::Vector2f bottomRight((p.x + 1) * tileSize, (p.y + 1) * tileSize);
        sf::Vector2f bottomLeft(p.x * tileSize, (p.y + 1) * tileSize);
        sf::Vector2f center((p.x + 0.5f) * tileSize, (p.y + 0.5f) * tileSize);
        // Path is drawn in blue
 ;
        pathVertices.append(sf::Vertex(topLeft, sf::Color::Blue));
        pathVertices.append(sf::Vertex(topRight, sf::Color::Blue));
        pathVertices.append(sf::Vertex(bottomRight, sf::Color::Blue));
        pathVertices.append(sf::Vertex(bottomLeft, sf::Color::Blue));
    }
    std::vector<sf::Vector2i> bestPathCenters;
    
    for (const sf::Vector2i& node : path) {
        sf::Vector2i center((node.x + 0.5f) * tileSize, (node.y + 0.5f) * tileSize);
        bestPathCenters.push_back(center);
        std::cout << "Calculated Center: (" << center.x << ", " << center.y << ")\n";
    }
    const float positionTolerance = 200.0f;
    while (window.isOpen()) {
        sf::Time deltaTime = clock.restart();
        float dt = deltaTime.asSeconds();
        sf::Event event;
        sf::Vector2f prevMousePosition; 
        bool firstMouseUpdate = true;
        sf::Time elapsed2;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
             
                if (!path.empty() && currentTargetIndex < path.size()) {
                    // Get the current target position from the path
                    sf::Vector2i targetNode = bestPathCenters[currentTargetIndex];
                    targetKinematic.position = sf::Vector2f(targetNode.x , targetNode.y); // Scale to match tile size
                    for (const auto& node : path) {
                        sf::Vector2f worldPosition(targetKinematic.position.x , targetKinematic.position.x );
                        std::cout << "World Position: (" << worldPosition.x << ", " << worldPosition.y << ")\n";
                    }
                
                    // Update boid to move toward the current target node
                    for (auto& b : boids) {
                        b->update(dt, targetKinematic);
                    }
                
                    // Check if the boid reached the current target node (you can add a small buffer to prevent overshooting)
                    sf::Vector2f boidPosition = boids[0]->getPosition();
                    float distance = std::sqrt(std::pow(boidPosition.x - targetKinematic.position.x , 2) +
                                            std::pow(boidPosition.y - targetKinematic.position.y, 2));

                    if (distance <= positionTolerance) { // Within threshold, move to next node
                        currentTargetIndex++;
                    }
                }
        }
        window.clear();

        // Draw the map and path
        window.draw(vertices);
        window.draw(pathVertices);
        for(int i = 0; i < breadcrumbs.size(); i++)
            {
                breadcrumbs[i].draw(&window);
            }
           // window.clear(sf::Color(255,255,255,255));      
            for (const auto& b : boids) {
                b->move(dt);
                b->draw();
                for(int i = 0; i < breadcrumbs.size(); i++)
                {
                    breadcrumbs[i].draw(&window);
                }
        }
        // Clear the window
        // Display the contents of the window
        window.display();
    }
   
    return 0;
}
std::vector<sf::Vector2i> findPath(const std::vector<std::vector<int>>& mapData, 
    sf::Vector2i start, sf::Vector2i goal, 
    bool useAStar, HeuristicType heuristicType);

// Helper function to normalize a vector
sf::Vector2f normalize(const sf::Vector2f& v) {
float length = std::sqrt(v.x * v.x + v.y * v.y);
if (length > 0) {
return sf::Vector2f(v.x / length, v.y / length);
}
return v;
}

int vsPath() {
sf::RenderWindow window(sf::VideoMode(1000, 1000), "Dijkstra & A* Pathfinding");
sf::Texture texture;
sf::Vector2f lastMousePos(400, 300);
VelocitySteering velocityMatch;
Kinematic targetKinematic;
targetKinematic.position = sf::Vector2f(500, 500);
targetKinematic.velocity = sf::Vector2f(0, 0);

// Player boid kinematic
Kinematic boidKinematic;
boidKinematic.position = sf::Vector2f(0, 0);
boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
boidKinematic.maxSpeed = 60.0f;
boidKinematic.maxAcceleration = 100.0f;
boidKinematic.maxRotation = 130.0f;
boidKinematic.slowRadius = 1.0f;
boidKinematic.arrivalRadius = 1.0f;
boidKinematic.fleeRadius = 5.0f;
float wonderdrop_timer = 1000.f;

// Enemy boid kinematic
Kinematic enemyKinematic;
enemyKinematic.position = sf::Vector2f(500, 500); // Start at goal position
enemyKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
enemyKinematic.maxSpeed = 100.0f; // Slightly faster than player
enemyKinematic.maxAcceleration = 15.0f;
enemyKinematic.maxRotation = 180.0f;
enemyKinematic.slowRadius = 1.0f;
enemyKinematic.arrivalRadius = 1.0f;
enemyKinematic.fleeRadius = 5.0f;

Arrive arrive;
Align align;

// load a texture for the player boid
if (!texture.loadFromFile("assets/boid.png"))
{
// error...
std::cerr << "Failed to load boid.png" << std::endl;
return -1;
}

// Create breadcrumbs
std::vector<crumb> breadcrumbs;
for(int i = 0; i < 100; i++)
{
crumb c(i);
breadcrumbs.push_back(c);
}   

sf::Clock clock;

// Create player boid
std::vector<std::unique_ptr<boid>> boids;
boids.push_back(std::make_unique<boid>(&window, texture, &breadcrumbs, boidKinematic));

// Create enemy boid
EnemyBoid enemyBoid(&window, &breadcrumbs, enemyKinematic);

// Define the 10x10 map
std::vector<std::vector<int>> mapData = {
{0,1,0,0,0,0,0,0,0,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,1,0,0,0,0,0,0,1,0},
{0,0,0,0,0,0,0,0,1,0}    
};

// Tile size for visualization
const int tileSize = 100;

// Create a vertex array to hold the vertices of the map
sf::VertexArray vertices(sf::Quads);

// Iterate through the map and create vertices for each tile
for (int y = 0; y < mapData.size(); ++y) {
for (int x = 0; x < mapData[y].size(); ++x) {
sf::Vector2f topLeft(x * tileSize, y * tileSize);
sf::Vector2f topRight((x + 1) * tileSize, y * tileSize);
sf::Vector2f bottomRight((x + 1) * tileSize, (y + 1) * tileSize);
sf::Vector2f bottomLeft(x * tileSize, (y + 1) * tileSize);

sf::Vector2i start(0, 0);  // Starting position (top-left corner)
sf::Vector2i goal(9, 9);   // Goal position (bottom-right corner)

// Check if the current tile is the start or goal
if (x == start.x && y == start.y) {
// If it's the start, make it pink
vertices.append(sf::Vertex(topLeft, sf::Color::Red)); // red
vertices.append(sf::Vertex(topRight, sf::Color::Red));
vertices.append(sf::Vertex(bottomRight, sf::Color::Red));
vertices.append(sf::Vertex(bottomLeft, sf::Color::Red));
} else if (x == goal.x && y == goal.y) {
// If it's the goal, make it green
vertices.append(sf::Vertex(topLeft, sf::Color::Green)); // Green
vertices.append(sf::Vertex(topRight, sf::Color::Green));
vertices.append(sf::Vertex(bottomRight, sf::Color::Green));
vertices.append(sf::Vertex(bottomLeft, sf::Color::Green));
}

// If the tile is passable (0), make it White
if (mapData[y][x] == 0) {
vertices.append(sf::Vertex(topLeft, sf::Color::White));
vertices.append(sf::Vertex(topRight, sf::Color::White));
vertices.append(sf::Vertex(bottomRight, sf::Color::White));
vertices.append(sf::Vertex(bottomLeft, sf::Color::White));
}
// If the tile is impassable (1), make it Black
else {
vertices.append(sf::Vertex(topLeft, sf::Color::Black));
vertices.append(sf::Vertex(topRight, sf::Color::Black));
vertices.append(sf::Vertex(bottomRight, sf::Color::Black));
vertices.append(sf::Vertex(bottomLeft, sf::Color::Black));
}
}
}

// Define start and goal positions
sf::Vector2i start(0, 0);  // Starting position (top-left corner)
sf::Vector2i goal(9, 9);   // Goal position (bottom-right corner)
int currentTargetIndex = 0; // Keeps track of the current target index in the path

// Generate the path
bool useAStar = false; // Set this to false to use Dijkstra
HeuristicType heuristicType = HeuristicType::Euclidean; // Choose between Manhattan and Euclidean
std::vector<sf::Vector2i> path = findPath(mapData, start, goal, useAStar, heuristicType);

// Create a separate vertex array for the path
sf::VertexArray pathVertices(sf::Quads);

for (const auto& p : path) {
sf::Vector2f topLeft(p.x * tileSize, p.y * tileSize);
sf::Vector2f topRight((p.x + 1) * tileSize, p.y * tileSize);
sf::Vector2f bottomRight((p.x + 1) * tileSize, (p.y + 1) * tileSize);
sf::Vector2f bottomLeft(p.x * tileSize, (p.y + 1) * tileSize);
sf::Vector2f center((p.x + 0.5f) * tileSize, (p.y + 0.5f) * tileSize);

// Path is drawn in blue
pathVertices.append(sf::Vertex(topLeft, sf::Color::Blue));
pathVertices.append(sf::Vertex(topRight, sf::Color::Blue));
pathVertices.append(sf::Vertex(bottomRight, sf::Color::Blue));
pathVertices.append(sf::Vertex(bottomLeft, sf::Color::Blue));
}

std::vector<sf::Vector2i> bestPathCenters;

for (const sf::Vector2i& node : path) {
sf::Vector2i center((node.x + 0.5f) * tileSize, (node.y + 0.5f) * tileSize);
bestPathCenters.push_back(center);
std::cout << "Calculated Center: (" << center.x << ", " << center.y << ")\n";
}

const float positionTolerance = 200.0f;

// Add wall avoidance function
auto avoidWalls = [&mapData, tileSize](Kinematic& k, float lookaheadDistance) {
// Check the tiles ahead in the direction of movement
sf::Vector2f futurePos = k.position + normalize(k.velocity) * lookaheadDistance;

// Get the map tile coordinates
int tileX = static_cast<int>(futurePos.x / tileSize);
int tileY = static_cast<int>(futurePos.y / tileSize);

// Check if the future position is within map bounds
if (tileX >= 0 && tileX < mapData[0].size() && tileY >= 0 && tileY < mapData.size()) {
// If the tile is a wall (1), steer away
if (mapData[tileY][tileX] == 1) {
// Find the center of the wall tile
sf::Vector2f wallCenter((tileX + 0.5f) * tileSize, (tileY + 0.5f) * tileSize);

// Calculate direction away from wall
sf::Vector2f awayFromWall = k.position - wallCenter;
float distance = std::sqrt(awayFromWall.x * awayFromWall.x + awayFromWall.y * awayFromWall.y);

if (distance > 0) {
awayFromWall = awayFromWall / distance; // Normalize

// Adjust velocity to steer away from wall
float steeringStrength = 10.0f / (distance + 0.1f); // Stronger steering when closer
k.velocity += awayFromWall * steeringStrength;

// Clamp to max speed
float speed = std::sqrt(k.velocity.x * k.velocity.x + k.velocity.y * k.velocity.y);
if (speed > k.maxSpeed) {
k.velocity = (k.velocity / speed) * k.maxSpeed;
}

return true; // Wall avoidance applied
}
}
}

// Also check if we're about to go out of bounds
if (tileX < 0 || tileX >= mapData[0].size() || tileY < 0 || tileY >= mapData.size()) {
// Steer back towards the center of the map
sf::Vector2f mapCenter((mapData[0].size() * tileSize) / 2.0f, (mapData.size() * tileSize) / 2.0f);
sf::Vector2f toCenter = mapCenter - k.position;
float distance = std::sqrt(toCenter.x * toCenter.x + toCenter.y * toCenter.y);

if (distance > 0) {
toCenter = toCenter / distance; // Normalize
k.velocity += toCenter * 10.0f;

// Clamp to max speed
float speed = std::sqrt(k.velocity.x * k.velocity.x + k.velocity.y * k.velocity.y);
if (speed > k.maxSpeed) {
k.velocity = (k.velocity / speed) * k.maxSpeed;
}

return true;
}
}

return false; // No wall avoidance needed
};

while (window.isOpen()) {
sf::Time deltaTime = clock.restart();
float dt = deltaTime.asSeconds();
sf::Event event;

while (window.pollEvent(event)) {
if (event.type == sf::Event::Closed)
window.close();

if (!path.empty() && currentTargetIndex < path.size()) {
// Get the current target position from the path
sf::Vector2i targetNode = bestPathCenters[currentTargetIndex];
targetKinematic.position = sf::Vector2f(targetNode.x, targetNode.y);

// Update boid to move toward the current target node
for (auto& b : boids) {
b->update(dt, targetKinematic);

// Get current kinematic from the boid
Kinematic currentKinematic = b->getKinematic();

// Apply wall avoidance
avoidWalls(currentKinematic, 30.0f);

// Update the boid with the modified kinematic
b->setKinematic(currentKinematic);
}

// Update enemy boid to chase the player boid
sf::Vector2f playerPos = boids[0]->getPosition();
Kinematic playerTarget;
playerTarget.position = playerPos;
enemyBoid.update(dt, playerTarget);

// Apply wall avoidance to enemy
Kinematic enemyCurrentKinematic = enemyBoid.getKinematic();
avoidWalls(enemyCurrentKinematic, 30.0f);
enemyBoid.setKinematic(enemyCurrentKinematic);

// Check if the boid reached the current target node
sf::Vector2f boidPosition = boids[0]->getPosition();
float distance = std::sqrt(std::pow(boidPosition.x - targetKinematic.position.x, 2) +
          std::pow(boidPosition.y - targetKinematic.position.y, 2));

if (distance <= positionTolerance) { // Within threshold, move to next node
currentTargetIndex++;
}
}
}

window.clear();

// Draw the map and path
window.draw(vertices);
window.draw(pathVertices);

for(int i = 0; i < breadcrumbs.size(); i++) {
breadcrumbs[i].draw(&window);
}

// Update and draw player boid
for (const auto& b : boids) {
b->move(dt);
b->draw();
}

// Update and draw enemy boid
enemyBoid.move(dt);
enemyBoid.draw();

// Display the contents of the window
window.display();
}

return 0;
}
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
    int numBoids = 2; // number of boids on screen
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
    if (!texture.loadFromFile("assets/boid.png"))
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
    //testPath();
    vsPath();
   /*if(flock == true)
     flocking();
    else
    AlignArriveAndWander();
   */
    return 0;
};