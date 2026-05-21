#include <chrono>
#include <thread>

#include "simulation_state.hpp"
#include "simulation_init.hpp"
#include "renderer.hpp"
#include "constants.hpp"

// placeholder physics function... put in its own file later
void update_physics(SimulationState& state) {
    // LBM engine will go here later
}

using simulation_clock = std::chrono::high_resolution_clock;

int main() {

    // initialise simulation state
    SimulationState state(
        config::LBM_WORLD_LENGTH_X,
        config::LBM_WORLD_HEIGHT_Y,
        config::LBM_WORLD_WIDTH_Z
    );

    initWindTunnel(state);

    // tell the simulation state where object is and what the file is 
    // simulation state doesnt store the actual mesh. 
    // that is handled by the render and voxel functions
    spawnObjectIntoWorld(
        state,
        "static/Porsche_911_GT2.obj",
        config::OBJECT_POSITION, // position
        config::OBJECT_ROTATION, // rotation
        config::OBJECT_SCALE // scale of object
    );

    // renderer
    Renderer renderer;

    if (!renderer.init()) {
        return -1;
    }

    // hot path with fixed time
    auto nextEngineTick = simulation_clock::now();
    auto nextRenderTick = simulation_clock::now();

    const auto engineInterval = std::chrono::microseconds(config::ENGINE_INTERVAL_MICROS);

    const auto renderInterval = std::chrono::microseconds(config::RENDER_INTERVAL_MICROS);

    // main loop
    while (renderer.isOpen()) {

        auto now = simulation_clock::now();

        // -------- engine hotpath --------
        while (now >= nextEngineTick) {

            update_physics(state);

            nextEngineTick += engineInterval;
        }

        // -------- render hotpath --------
        if (now >= nextRenderTick) {

            renderer.handleEvents();
            // renderer.renderLBMVoxel(state);
            renderer.renderScene(state);

            nextRenderTick += renderInterval;
        }
    }

    renderer.close();

    return 0;
}
