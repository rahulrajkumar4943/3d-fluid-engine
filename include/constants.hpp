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

    // lbm parameters (only for math and physics)
    // world / domain size
    // in meters
    constexpr float WORLD_LENGTH_X = 14.0f; // flow direction
    constexpr float WORLD_HEIGHT_Y = 6.0f;
    constexpr float WORLD_WIDTH_Z  = 6.0f;
    // in voxels
    constexpr int LBM_WORLD_LENGTH_X = 140; // flow direction
    constexpr int LBM_WORLD_HEIGHT_Y = 60;
    constexpr int LBM_WORLD_WIDTH_Z  = 60;
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
    // lbm parameters
    constexpr float TAU = 0.6f;   // relaxation time
    constexpr float DT  = 1.0f;   // lattice timestep
    constexpr float CS2 = 1.0f / 3.0f; // speed of sound squared

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
