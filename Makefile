CXX = g++
CXXFLAGS = -std=c++20 -O2 -Iinclude -I/opt/homebrew/opt/raylib/include

RAYLIB_LINK = -L/opt/homebrew/opt/raylib/lib -lraylib \
              -framework OpenGL -framework Cocoa \
              -framework IOKit -framework CoreVideo

HEADERS = include/*.hpp

all: engine renderer

engine: src/engine_main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) src/engine_main.cpp -o engine $(RAYLIB_LINK)

renderer: src/renderer_main.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) src/renderer_main.cpp -o renderer $(RAYLIB_LINK)

clean:
	rm -f engine renderer
