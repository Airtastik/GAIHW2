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
#include "run_data.hpp"
#include "learned_behavior.hpp"
#include <numeric>
#include "utils.hpp"  // Add this include at the top
#include "learning_enemy.hpp"

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

    for (size_t y = 0; y < mapData.size(); ++y) {
        for (size_t x = 0; x < mapData[y].size(); ++x) {
            sf::Vector2f topLeft(static_cast<float>(x * tileSize), static_cast<float>(y * tileSize));
            sf::Vector2f topRight(static_cast<float>((x + 1) * tileSize), static_cast<float>(y * tileSize));
            sf::Vector2f bottomRight(static_cast<float>((x + 1) * tileSize), static_cast<float>((y + 1) * tileSize));
            sf::Vector2f bottomLeft(static_cast<float>(x * tileSize), static_cast<float>((y + 1) * tileSize));

            // Check if the current tile is the start or goal
            if (static_cast<int>(x) == start.x && static_cast<int>(y) == start.y) {
                // If it's the start, make it red
                vertices.append(sf::Vertex(topLeft, sf::Color::Red));
                vertices.append(sf::Vertex(topRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomRight, sf::Color::Red));
                vertices.append(sf::Vertex(bottomLeft, sf::Color::Red));
            } else if (static_cast<int>(x) == goal.x && static_cast<int>(y) == goal.y) {
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
void testPath() {
    sf::RenderWindow window(sf::VideoMode(1000, 1000), "Pathfinding");
    sf::Texture texture;

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

    int numBoids = 1;

    // ...rest of existing code...
}
std::vector<sf::Vector2i> findPath(const std::vector<std::vector<int>>& mapData, 
    sf::Vector2i start, sf::Vector2i goal, 
    bool useAStar, HeuristicType heuristicType);

// Add length function at global scope
RunData vsPath() {
    RunData data;
    data.playerWon = false;  // Initialize to false
    data.runDuration = 0.0f;

    try {
        sf::Clock totalTime;
        sf::RenderWindow window(sf::VideoMode(1000, 1000), "Pathfinding");
        sf::Texture texture;

        if (!texture.loadFromFile("assets/boid.png")) {
            throw std::runtime_error("Failed to load boid.png");
        }

        // Initialize boid with safer values
        Kinematic boidKinematic;
        boidKinematic.position = sf::Vector2f(50, 50);  // Start away from corner
        boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        boidKinematic.maxSpeed = 150.0f;
        boidKinematic.maxAcceleration = 100.0f;
        boidKinematic.orientation = 0.0f;
        boidKinematic.maxRotation = 180.0f;

        // Initialize enemy with safer values
        Kinematic enemyKinematic;
        enemyKinematic.position = sf::Vector2f(500, 500);
        enemyKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        enemyKinematic.maxSpeed = 200.0f;
        enemyKinematic.maxAcceleration = 120.0f;

        // Create resources
        std::vector<crumb> breadcrumbs(100, crumb(0));
        std::vector<std::unique_ptr<boid>> boids;
        
        // Create boid with exception handling
        try {
            boids.push_back(std::make_unique<boid>(&window, texture, &breadcrumbs, boidKinematic));
        } catch (const std::exception& e) {
            std::cerr << "Failed to create boid: " << e.what() << std::endl;
            return data;
        }

        // Initialize enemy boid
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
        const float COLLISION_RADIUS = 30.0f;  // Add this near the other constants

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
            for (size_t i = 0; i < breadcrumbs.size(); ++i) {
                breadcrumbs[i].draw(&window);
            }

            // Update and draw player boids
            for (auto& b : boids) {
                if (static_cast<size_t>(currentTargetIndex) < pathWorldPositions.size()) {
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

            // After updating both boids but before display, check for collision
            sf::Vector2f boidPosition = boids[0]->getPosition();
            sf::Vector2f enemyPosition = enemyBoid.getKinematic().position;

            // Calculate distance between boid and enemy
            float distanceToEnemy = std::sqrt(
                std::pow(boidPosition.x - enemyPosition.x, 2) +
                std::pow(boidPosition.y - enemyPosition.y, 2)
            );

            // If enemy catches the boid, close window
            if (distanceToEnemy <= COLLISION_RADIUS) {
                std::cout << "Enemy caught the boid! Game Over." << std::endl;
                data.playerWon = false;
                data.runDuration = totalTime.getElapsedTime().asSeconds();
                window.close();
                return data;  // Return immediately to prevent overwriting
            }

            // After updating boid positions but before drawing
            // Check if boid has reached the goal
            float distanceToGoal = std::sqrt(
                std::pow(boidPosition.x - goalPosition.x, 2) +
                std::pow(boidPosition.y - goalPosition.y, 2)
            );

            if (distanceToGoal <= GOAL_RADIUS) {
                std::cout << "Goal reached! Player won!" << std::endl;
                data.playerWon = true;
                data.runDuration = totalTime.getElapsedTime().asSeconds();
                window.close();
                return data;  // Return immediately to prevent overwriting
            }

            // Record enemy behavior each frame
            BehaviorSample sample;
            sample.selfKinematic = enemyBoid.getKinematic();
            sample.playerKinematic = boids[0]->getKinematic();
            sample.mapData = mapData;
            sample.actionTaken = enemyBoid.getKinematic().velocity;
            sample.distanceToPlayer = std::sqrt(distanceToEnemy);
            sample.angleToPlayer = std::atan2(enemyPosition.y - boidPosition.y,
                enemyPosition.x - boidPosition.x);
            sample.relativeSpeed = length(enemyBoid.getKinematic().velocity);
            sample.isNearWall = false;  // Set based on wall proximity check

            data.samples.push_back(sample);

            // Check win/loss conditions
            if (distanceToEnemy <= COLLISION_RADIUS) {
                data.playerWon = false;
                window.close();
                break;
            }

            if (distanceToGoal <= GOAL_RADIUS) {
                data.playerWon = true;
                window.close();
                break;
            }

            window.display();
        }

        data.runDuration = totalTime.getElapsedTime().asSeconds();
    } catch (const std::exception& e) {
        std::cerr << "Error in vsPath: " << e.what() << std::endl;
    }
    return data;
}

RunData vsPathLearned(LearnedBehavior& learner) {
    RunData data;
    data.playerWon = false;
    data.runDuration = 0.0f;

    try {
        sf::Clock totalTime;
        sf::RenderWindow window(sf::VideoMode(1000, 1000), "Pathfinding (Learned Behavior)");
        sf::Texture texture;

        if (!texture.loadFromFile("assets/boid.png")) {
            throw std::runtime_error("Failed to load boid.png");
        }

        // Initialize boid with safer values
        Kinematic boidKinematic;
        boidKinematic.position = sf::Vector2f(50, 50);
        boidKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        boidKinematic.maxSpeed = 150.0f;
        boidKinematic.maxAcceleration = 100.0f;
        boidKinematic.orientation = 0.0f;
        boidKinematic.maxRotation = 180.0f;

        // Initialize enemy with safer values
        Kinematic enemyKinematic;
        enemyKinematic.position = sf::Vector2f(500, 500);
        enemyKinematic.velocity = sf::Vector2f(0.0f, 0.0f);
        enemyKinematic.maxSpeed = 200.0f;
        enemyKinematic.maxAcceleration = 120.0f;

        // Create resources
        std::vector<crumb> breadcrumbs(100, crumb(0));
        std::vector<std::unique_ptr<boid>> boids;

        // Setup pathfinding
        const int tileSize = 100;
        sf::Vector2i start(0, 0);
        sf::Vector2i goal(9, 9);
        
        // Define the map
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

        // Create visual elements
        sf::VertexArray pathVertices = createColoredMap(mapData, tileSize, start, goal);
        std::vector<sf::Vector2i> path = findPath(mapData, start, goal, true, HeuristicType::Euclidean);
        std::vector<sf::Vector2f> pathWorldPositions;
        for (const auto& node : path) {
            pathWorldPositions.emplace_back((node.x + 0.5f) * tileSize, (node.y + 0.5f) * tileSize);
        }

        // Create boid and enemy
        boids.push_back(std::make_unique<boid>(&window, texture, &breadcrumbs, boidKinematic));
        LearningEnemy enemyBoid(&window, &breadcrumbs, enemyKinematic, &learner);

        sf::Clock clock;
        int currentTargetIndex = 0;
        const float positionTolerance = 100.0f;
        const float COLLISION_RADIUS = 30.0f;
        const float GOAL_RADIUS = 50.0f;

        sf::Vector2f goalPosition((goal.x + 0.5f) * tileSize, (goal.y + 0.5f) * tileSize);

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

            // Update and draw boid
            if (currentTargetIndex < pathWorldPositions.size()) {
                Kinematic targetKinematic;
                targetKinematic.position = pathWorldPositions[currentTargetIndex];
                targetKinematic.maxSpeed = 400.0f;
                targetKinematic.maxAcceleration = 200.0f;

                boids[0]->updateWithEnemy(dt, targetKinematic, enemyBoid.getKinematic(), mapData, tileSize);
                boids[0]->move(dt);
                boids[0]->draw();

                // Check if reached current target
                float distance = length(boids[0]->getPosition() - targetKinematic.position);
                if (distance <= positionTolerance) {
                    currentTargetIndex++;
                }
            }

            // Update and draw enemy
            enemyBoid.update(dt, boids[0]->getKinematic(), mapData, tileSize, false);
            enemyBoid.draw();

            // Check win/lose conditions
            sf::Vector2f boidPosition = boids[0]->getPosition();
            sf::Vector2f enemyPosition = enemyBoid.getKinematic().position;
            
            float distanceToEnemy = length(boidPosition - enemyPosition);
            float distanceToGoal = length(boidPosition - goalPosition);

            if (distanceToEnemy <= COLLISION_RADIUS) {
                data.playerWon = false;
                data.runDuration = totalTime.getElapsedTime().asSeconds();
                window.close();
                return data;
            }

            if (distanceToGoal <= GOAL_RADIUS) {
                data.playerWon = true;
                data.runDuration = totalTime.getElapsedTime().asSeconds();
                window.close();
                return data;
            }

            window.display();
        }

        data.runDuration = totalTime.getElapsedTime().asSeconds();
    } catch (const std::exception& e) {
        std::cerr << "Error in vsPathLearned: " << e.what() << std::endl;
    }
    return data;
}

// ...existing code...

void multRun(LearnedBehavior& learner, int numRuns) {  // Add learner parameter
    std::vector<RunData> allRunsData;
    size_t playerWins = 0;
    
    for(int i = 0; i < numRuns; i++) {
        std::cout << "Starting run " << i + 1 << " of " << numRuns << std::endl;
        
        RunData runData = vsPath();
        if (runData.playerWon) {
            playerWins++;
            std::cout << "Run " << i + 1 << ": Player won!" << std::endl;
        } else {
            std::cout << "Run " << i + 1 << ": Enemy won!" << std::endl;
        }
        
        allRunsData.push_back(runData);
        
        // Add samples to learner
        for(const auto& sample : runData.samples) {
            learner.recordBehavior(sample.selfKinematic, sample.playerKinematic, 
                                 sample.mapData, sample.actionTaken);
        }
    }
    
    // Train the decision tree using collected data
    learner.trainDecisionTree();
    
    // Use the directly tracked wins instead of counting
    float avgDuration = std::accumulate(allRunsData.begin(), allRunsData.end(), 0.0f,
                               [](float sum, const RunData& data) { 
                                   return sum + data.runDuration; 
                               }) / static_cast<float>(numRuns);
    
    std::cout << "Training completed:\n"
              << "Player wins: " << playerWins << "/" << numRuns << "\n"
              << "Average run duration: " << avgDuration << "s" << std::endl;
}

void testLearnedBehavior(LearnedBehavior& learner, int numRuns) {
    std::vector<RunData> results;
    size_t playerWins = 0;
    
    for(int i = 0; i < numRuns; i++) {
        std::cout << "Starting learned behavior run " << i + 1 << " of " << numRuns << std::endl;
        
        RunData runData = vsPathLearned(learner);
        if (runData.playerWon) {
            playerWins++;
            std::cout << "Learned Run " << i + 1 << ": Player won!" << std::endl;
        } else {
            std::cout << "Learned Run " << i + 1 << ": Enemy won!" << std::endl;
        }
        
        results.push_back(runData);
    }
    
    float avgDuration = std::accumulate(results.begin(), results.end(), 0.0f,
                               [](float sum, const RunData& data) { 
                                   return sum + data.runDuration; 
                               }) / static_cast<float>(numRuns);
    
    std::cout << "\nLearned Behavior Results:\n"
              << "Player wins: " << playerWins << "/" << numRuns << "\n"
              << "Average run duration: " << avgDuration << "s" << std::endl;
}

bool flock = false;
int main()
{
    std::cout << "Start" << std::endl;
    
    // Create single learner instance
    const int NUM_RUNS = 5;
    LearnedBehavior learner;  // Single instance for both training and testing
    
    std::cout << "\nTraining Phase:" << std::endl;
    multRun(learner, NUM_RUNS);  // Pass learner reference
    
    std::cout << "\nTesting Learned Behavior:" << std::endl;
    testLearnedBehavior(learner, 1);  // Use same learner instance
    
    return 0;
}