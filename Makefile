# Compiler and Flags
CXX = g++
CXXFLAGS = -Wall -Wextra -Iinclude -std=c++17
LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

# Directories
SRC_DIR = src
BIN_DIR = bin
OBJ_DIR = obj

# Source and Object Files
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))

# Default Executable
TARGET = $(BIN_DIR)/project

# Default Rule: Build the executable
all: $(TARGET)

# Link the object files to create the executable
$(TARGET): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile each .cpp file into an object file
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure necessary directories exist
$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

# Clean build files
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Run the compiled program
run: all
	$(TARGET)