#pragma once

#include "raylib.h"

namespace config {

    // physics engine and render frequency
    constexpr int ENGINE_HZ = 200;
    constexpr int RENDER_HZ = 60;

    constexpr int MICROS_IN_S = 1000000.0f;
    // interval micros are int because std chrono microseconds needs int
    constexpr int ENGINE_INTERVAL_MICROS = MICROS_IN_S / ENGINE_HZ; // microseconds between each engine frame
    constexpr int RENDER_INTERVAL_MICROS = MICROS_IN_S / RENDER_HZ; // microseconds between each render frame
    constexpr float ENGINE_INTERVAL_S = 1.0f / ENGINE_HZ; // timestep in seconds aka delta

    // seed for random number generator
    constexpr int RNG_SEED = 938475;
    // lbm parameters (only for math and physics)
    // world / domain size
    // in meters
    constexpr float WORLD_LENGTH_X = 14.0f; // flow direction
    constexpr float WORLD_HEIGHT_Y = 6.0f;
    constexpr float WORLD_WIDTH_Z  = 6.0f;
    // in voxels aka resolution
    constexpr int LBM_WORLD_LENGTH_X = 70; // flow direction
    constexpr int LBM_WORLD_HEIGHT_Y = 30;
    constexpr int LBM_WORLD_WIDTH_Z  = 30;
    // cell sizes
    constexpr float CELL_LENGTH_X = WORLD_LENGTH_X / LBM_WORLD_LENGTH_X; // flow direction
    constexpr float CELL_HEIGHT_Y = WORLD_HEIGHT_Y / LBM_WORLD_HEIGHT_Y;
    constexpr float CELL_WIDTH_Z  = WORLD_WIDTH_Z / LBM_WORLD_WIDTH_Z;
    // alias for lbm style code
    constexpr int NX = LBM_WORLD_LENGTH_X;
    constexpr int NY = LBM_WORLD_HEIGHT_Y;
    constexpr int NZ = LBM_WORLD_WIDTH_Z;
    // floor and wall thickness
    constexpr float FLOOR_THICKNESS = 0.1f;
    constexpr float WALL_THICKNESS = 0.1f;
    // object parameters
    inline const Vector3 OBJECT_POSITION = {WORLD_LENGTH_X/2, 0.62, WORLD_WIDTH_Z/2}; // for porsche 0 0 0 is middle of car. // porsche 911 g1 is roughly 1.3m tall so y offset is only 0.65
    inline const Vector3 OBJECT_ROTATION = {0.0f, 90.0f, 0.0f}; // x 90 makes car face up, y 90 makes car face inlet, z 90 makes car go on its side
    constexpr float OBJECT_SCALE = 1.0f; // the porsche model is already the correct size in meters
    // lbm parameters
    constexpr float TAU = 0.6f;   // relaxation time, usually about 0.6
    constexpr float RHO = 1.0f;   // density of fluid in a voxel
    // constexpr float DT  = 1.0f;   // lattice timestep this value is arbitrary and wrong
    constexpr float CS2 = 1.0f / 3.0f; // speed of sound squared
    // inlet and initial velocities for lbm sim
    constexpr float INLET_VELOCITY_X_MPS = 5.0f; // meter/s
    constexpr float INLET_VELOCITY_X = INLET_VELOCITY_X_MPS * (LBM_WORLD_LENGTH_X / WORLD_LENGTH_X); // lattice_unit/s
    constexpr float INLET_VELOCITY_X_LUPT = INLET_VELOCITY_X *  ENGINE_INTERVAL_S; // lattice units per timestep
    constexpr float INLET_VELOCITY_Y = 0.0f;
    constexpr float INLET_VELOCITY_Z = 0.0f;
    // adjustment to speed of whole wind tunnel every tick to prevent decay
    constexpr float VOXEL_X_ADJUSTMENT = 0.00025f;
    // tracer particles
    constexpr int MAX_PARTICLES = 150000; // 150k and 60k work fine. if you 10x that i think it does fit on cache anymore
    constexpr int INITIAL_PARTICLES = 60000;
    // number of particles to spawn per tick
    constexpr int INLET_SPAWN_FREQ = 100;


    // networking parameters
    constexpr int NETWORK_PORT = 9001;
    constexpr const char* NETWORK_ADDRESS = "127.0.0.1";
    constexpr int NETWORK_TICKRATE_HZ = 60;
    constexpr int NETWORK_DECIMATION = 4;
    constexpr float QUANTIZE_RESOLUTION = 65535.0f; // largest value in uint16
    // 1200 particles per packet = 7200 bytes + 16 bytes of header
    constexpr int NETWORK_PARTICLES_PER_PACKET = 1200;
    


    // render parameters
    constexpr int WINDOW_LENGTH = 1200;
    constexpr int WINDOW_HEIGHT = 800;
    constexpr int WINDOW_FPS = 60;
    // camera parameters
    constexpr float YAW = -90.0f;
    constexpr float PITCH = 0.0f;
    constexpr float MOVE_SPEED = 0.5f;
    constexpr float MOUSE_SENS = 0.1f;
    inline const Vector3 CAMERA_POSITION = {-1.0f, 4.0f, 10.0f}; // in meters
    inline const Vector3 CAMERA_TARGET = { 0.0f, 0.0f, 0.0f };
    inline const Vector3 CAMERA_UP = { 0.0f, 1.0f, 0.0f };
    constexpr float CAMERA_FOVY = 60.0f;


}
