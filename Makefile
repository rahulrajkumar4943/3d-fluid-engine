CXX = g++

RAYLIB_PATH = $(shell brew --prefix raylib)

CXXFLAGS = -std=c++17 -O2 -Iinclude -I$(RAYLIB_PATH)/include

LDFLAGS = -L$(RAYLIB_PATH)/lib \
          -lraylib \
          -framework OpenGL \
          -framework Cocoa \
          -framework IOKit \
          -framework CoreVideo

SRC = $(wildcard src/*.cpp)
TARGET = engine

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

run:
	./$(TARGET)

clean:
	rm -f $(TARGET)
