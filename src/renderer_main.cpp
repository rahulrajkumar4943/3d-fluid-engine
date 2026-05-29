#include <iostream>
#include "simulation_state.hpp"
#include "simulation_init.hpp"
#include "renderer.hpp"
#include "constants.hpp"
#include "net_receiver.hpp"

int main() {
    NetReceiver receiver;
    if (!receiver.init()) {
        return -1;
    }

    // create a simulationstate just for rendering
    // render scene function in renderer needs the state
    // and the car model path
    SimulationState state(
        config::LBM_WORLD_LENGTH_X,
        config::LBM_WORLD_HEIGHT_Y,
        config::LBM_WORLD_WIDTH_Z
    );
    spawnObjectIntoWorld(state,
        "static/Porsche_911_GT2.obj",
        config::OBJECT_POSITION,
        config::OBJECT_ROTATION,
        config::OBJECT_SCALE
    );

    // initialize renderer and load object
    Renderer renderer;
    if (!renderer.init()) {
        return -1;
    }
    renderer.loadObject(state.object.stlPath); // no need for cstring here because that is done in loadobject function

    while (renderer.isOpen()) {
        // poll receiver this function writes to receiver data struction which can be read from
        receiver.poll();
        renderer.handleEvents();

        // copy received particles into state for render scene
        const auto& px = receiver.get_px();
        const auto& py = receiver.get_py();
        const auto& pz = receiver.get_pz();
        const auto& pspeed = receiver.get_pspeed();

        // count is the number of particles
        // size of px or tracer count whichever is smaller
        // size is type size t so must be casted to int
        int count = std::min((int)px.size(), state.tracerCount);
        // only draw active particles
        for (int i = 0; i < count; i++) {
            state.tracer_particles_x[i] = px[i];
            state.tracer_particles_y[i] = py[i];
            state.tracer_particles_z[i] = pz[i];
            state.tracer_speed[i] = pspeed[i];
        }
        // zero out the rest so render scene doesnt draw stale particles
        for (int i = count; i < state.tracerCount; i++) {
            state.tracer_particles_x[i] = 0.0f;
            state.tracer_speed[i] = 0.0f;
        }

        renderer.renderScene(state);
    }

    renderer.close();
    return 0;
}
