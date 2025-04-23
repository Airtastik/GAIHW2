// 
// @author Tristan Hall (tdhall6@ncsu.edu)
// I used ChatGPT to assist with creation
#include "steering.hpp"
#include "vertex.hpp"
#include "pathmap.hpp"
#include "PATHFIND.hpp"
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
sf::VertexArray createColoredMap(const std::vector<std::vector<int>>& mapData, int tileSize, sf::Vector2i start, sf::Vector2i goal) {
    sf::VertexArray vertices(sf::Quads);

    for (int y = 0; y < mapData.size(); ++y) {
        for (int x = 0; x < mapData[y].size(); ++x) {
            sf::Vector2f topLeft(x * tileSize, y * tileSize);
            sf::Vector2f topRight((x + 1) * tileSize, y * tileSize);
            sf::Vector2f bottomRight((x + 1) * tileSize, (y + 1) * tileSize);
            sf::Vector2f bottomLeft(x * tileSize, (y + 1) * tileSize);

            // Check if the current tile is the start or goal
            if (x == start.x && y == start.y) {
                // If it's the start, make it red
                vertices.append(sf::Vertex(topLeft, sf::Color::Red));
                vertices.append(sf::Vertex(topRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Red));
            } else if (x == goal.x && y == goal.y) {
                // If it's the goal, make it green
                vertices.append(sf::Vertex(topLeft, sf::Color::Green));
                vertices.append(sf::Vertex(topRight, sf::Color::Green));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Green));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Green));
            } else if (mapData[y][x] == 0) {
                // If the tile is passable (0), make it white
                vertices.append(sf::Vertex(topLeft, sf::Color::White));
                vertices.append(sf::Vertex(topRight, sf::Color::White));
                vertices.append(sf::Vertex(bottomRight, sf::Color::White));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::White));
            } else if (mapData[y][x] == 1) {
                // If the tile is impassable (1), make it black
                vertices.append(sf::Vertex(topLeft, sf::Color::Black));
                vertices.append(sf::Vertex(topRight, sf::Color::Black));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Black));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Black));
            } else {
                // Handle unexpected values in mapData
                vertices.append(sf::Vertex(topLeft, sf::Color::Yellow)); // Debug color for unexpected values
                vertices.append(sf::Vertex(topRight, sf::Color::Yellow));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Yellow));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Yellow));
            }
        }
    }

    return vertices;
}
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

    // Load the texture for the boid
    if (!texture.loadFromFile("assets/boid.png")) {
        std::cerr << "Failed to load boid.png" << std::endl;
        return -1;
    }
    Kinematic boidKinematic;
    boidKinematic.position = sf::Vector2f(0, 0);
    boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
    boidKinematic.maxSpeed = 200.0f;         // Increased from 200.0f
    boidKinematic.maxAcceleration = 100.0f;  // Increased from 100.0f

    // Initialize enemy boid kinematic
    Kinematic enemyKinematic;
    enemyKinematic.position = sf::Vector2f(500, 500);
    enemyKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
    enemyKinematic.maxSpeed = 300.0f;         // Increased from 300.0f
    enemyKinematic.maxAcceleration = 150.0f;  // Increased from 100.0f
    enemyKinematic.maxRotation = 90.0f;
    enemyKinematic.maxAngularAcceleration = 50.0f;
    enemyKinematic.arrivalRadius = 30.0f;
    enemyKinematic.slowRadius = 60.0f;
    enemyKinematic.fleeRadius = 100.0f;

    // Create breadcrumbs
    std::vector<crumb> breadcrumbs;
    for (int i = 0; i < 100; i++) {
        crumb c(i);
        breadcrumbs.push_back(c);
    }

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

    // Define start and goal positions
    sf::Vector2i start(0, 0);  // Starting position
    sf::Vector2i goal(9, 9);   // Goal position

    // Create the map using the helper function
    sf::VertexArray pathVertices = createColoredMap(mapData, tileSize, start, goal);

    // Generate the path using the pathfinding algorithm
    bool useAStar = true;      // Use A* algorithm
    HeuristicType heuristicType = HeuristicType::Euclidean; // Choose between Manhattan and Euclidean
    std::vector<sf::Vector2i> path = findPath(mapData, start, goal, useAStar, heuristicType);

    // Convert path nodes to world positions
    std::vector<sf::Vector2f> pathWorldPositions;
    for (const auto& node : path) {
        pathWorldPositions.emplace_back((node.x + 0.5f) * tileSize, (node.y + 0.5f) * tileSize);
    }

    // Create a vertex array for the path visualization

    for (const auto& node : path) {
        sf::Vector2f topLeft(node.x * tileSize, node.y * tileSize);
        sf::Vector2f topRight((node.x + 1) * tileSize, node.y * tileSize);
        sf::Vector2f bottomRight((node.x + 1) * tileSize, (node.y + 1) * tileSize);
        sf::Vector2f bottomLeft(node.x * tileSize, (node.y + 1) * tileSize);

        pathVertices.append(sf::Vertex(topLeft, sf::Color::Blue));
        pathVertices.append(sf::Vertex(topRight, sf::Color::Blue));
        pathVertices.append(sf::Vertex(bottomRight, sf::Color::Blue));
        pathVertices.append(sf::Vertex(bottomLeft, sf::Color::Blue));
    }

    sf::Clock clock;
    int currentTargetIndex = 0;
    const float positionTolerance = 100.0f;

   // std::cout << "Path positions:" << std::endl;
    for (const auto& pos : pathWorldPositions) {
     //   std::cout << "(" << pos.x << ", " << pos.y << ")" << std::endl;
    }

    // Add goal position in world coordinates (9,9 in tile coordinates)
    sf::Vector2f goalPosition((goal.x + 0.5f) * tileSize, (goal.y + 0.5f) * tileSize);
    const float GOAL_RADIUS = 50.0f;  // How close the boid needs to be to count as reaching the goal

    while (window.isOpen()) {
        sf::Time deltaTime = clock.restart();
        float dt = deltaTime.asSeconds();
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(pathVertices);

        // Update and draw breadcrumbs
        for (auto& crumb : breadcrumbs) {
            crumb.draw(&window);
        }

        // Update and draw player boids
        for (auto& b : boids) {
            if (currentTargetIndex < pathWorldPositions.size()) {
                // Debug current target
              /*  std::cout << "Current target: " << currentTargetIndex 
                         << " Position: (" << pathWorldPositions[currentTargetIndex].x 
                         << ", " << pathWorldPositions[currentTargetIndex].y << ")" << std::endl;
*/
                // Set the current target position with all necessary parameters
                Kinematic targetKinematic;
                targetKinematic.position = pathWorldPositions[currentTargetIndex];
                targetKinematic.maxSpeed = 400.0f;          // Increased
                targetKinematic.maxAcceleration = 200.0f;   // Increased
                targetKinematic.maxAngularAcceleration = 5.0f;
                targetKinematic.maxRotation = 5.0f;
                targetKinematic.arrivalRadius = 3.0f;
                targetKinematic.slowRadius = 3.0f;
                targetKinematic.fleeRadius = 5.0f;

                // Update boid position and orientation
                b->updateWithEnemy(dt, targetKinematic, enemyBoid.getKinematic(), mapData, tileSize);
                
                // Debug boid position
                sf::Vector2f boidPosition = b->getPosition();
               // std::cout << "Boid position: (" << boidPosition.x << ", " << boidPosition.y << ")" << std::endl;

                // Check if reached target
                float distance = std::sqrt(
                    std::pow(boidPosition.x - targetKinematic.position.x, 2) +
                    std::pow(boidPosition.y - targetKinematic.position.y, 2)
                );
//                std::cout << "Distance to target: " << distance << std::endl;

                if (distance <= positionTolerance) {
                    currentTargetIndex++;
               //     std::cout << "Moving to next target: " << currentTargetIndex << std::endl;
                }
            }

            // Make sure to call move before draw
            b->move(dt);
            b->draw();
        }

        // Update and draw the enemy boid
        enemyBoid.update(dt, boids[0]->getKinematic(), mapData, tileSize, false);
        enemyBoid.move(dt, mapData);  // Updated to include mapData parameter
        enemyBoid.draw();

        // After updating boid positions but before drawing
        // Check if boid has reached the goal
        sf::Vector2f boidPosition = boids[0]->getPosition();
        float distanceToGoal = std::sqrt(
            std::pow(boidPosition.x - goalPosition.x, 2) +
            std::pow(boidPosition.y - goalPosition.y, 2)
        );

        if (distanceToGoal <= GOAL_RADIUS) {
            std::cout << "Goal reached! Closing window..." << std::endl;
            window.close();
            break;
        }

        window.display();
    }

    return 0;

}

bool flock = false;
int main()
{
    //testPath();
    std::cout << "Start" << std::endl;
    vsPath();
   /*if(flock == true)
     flocking();
    else
    AlignArriveAndWander();
   */
    return 0;
};