# compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++20

# executable name
TARGET = engine

# src directories
SRC_DIR = src
SRC = $(SRC_DIR)/main.cpp

# default build executable
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

# clean
clean:
	rm -f $(TARGET)
