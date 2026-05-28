#include <iostream>
#include <chrono>
#include "raylib.h"
#include "simulation_state.hpp"
#include "simulation_init.hpp"
#include "physics.hpp"
#include "constants.hpp"
#include "net_sender.hpp"

using simulation_clock = std::chrono::high_resolution_clock;

int main() {
    // temporary hidden 1x1 window just for raylib model loading and voxelization
    // after this window closes and engine runs without window
    // raylib window is needed to load model and do voxelizations
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(1, 1, "engine initialization");
    SetWindowState(FLAG_WINDOW_HIDDEN);

    // init the state
    SimulationState state(
        config::LBM_WORLD_LENGTH_X,
        config::LBM_WORLD_HEIGHT_Y,
        config::LBM_WORLD_WIDTH_Z
    );

    // init the wind tunnel and spawn the object into world
    initWindTunnel(state);
    spawnObjectIntoWorld(state,
        "static/Porsche_911_GT2.obj",
        config::OBJECT_POSITION,
        config::OBJECT_ROTATION,
        config::OBJECT_SCALE
    );

    // load model for voxelization using raylib
    // need to call loadmodel instead of lload object because theres no more renderer class here
    Model model = LoadModel(state.object.stlPath.c_str()); // load model wants a c string not std string
    voxelizeObject(state, model);
    UnloadModel(model);

    // close window, window not needed for engine
    CloseWindow();

    // init lbm by setting velocity to all voxels
    initializeLBM(state);



    // init UDP sender
    NetSender sender;
    if (!sender.init()) {
        // if sender failed then return -1 on main function
        return -1;
    }

    const auto engine_interval = std::chrono::microseconds(config::ENGINE_INTERVAL_MICROS);
    const auto network_interval = std::chrono::microseconds(config::MICROS_IN_S / config::NETWORK_TICKRATE_HZ);

    auto next_engine_tick = simulation_clock::now();
    auto next_network_tick = simulation_clock::now();

    while (true) {
        auto now = simulation_clock::now();

        // physics at 200hz
        // while so that engine can catch up if it misses something
        // if engine tick takes longer to do than its supposed to consistently
        // then this while loop keeps running and the udp sender to the renderer slows down
        // to do
        // add a cap to the number of times this while loop can run (cap it to 5)
        while (now >= next_engine_tick) {
            update_physics(state);
            next_engine_tick += engine_interval;
        }

        // network broadcast at 60hz
        if (now >= next_network_tick) {
            sender.broadcast(state);
            next_network_tick += network_interval;
        }
    }

    return 0;
}
