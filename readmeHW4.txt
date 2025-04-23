Building and Running the Boid Simulation
======================================

Prerequisites:
-------------
1. Ubuntu 20.04 system
2. SFML library and development packages
3. G++ compiler
4. Make build system

Installation Steps:
-----------------
1. Install required packages:
   sudo apt-get update
   sudo apt-get install g++ make libsfml-dev

Building the Project:
-------------------
1. Open terminal in the project root directory
2. Run the following commands:
   make clean    # Clear any previous builds
   make         # Build the project
   make run # make and run project

Running the Simulation:
---------------------
1. After building, run the executable:
   ./build/boid_simulation

Troubleshooting:
--------------
If you encounter errors about missing SFML:
1. Verify SFML is installed: dpkg -l | grep sfml
2. Make sure library paths are correct: ldconfig -p | grep sfml

If you encounter permission issues:
1. Make the executable runnable: chmod +x ./build/boid_simulation

Note: Make sure you have read/write permissions in the project directory.
