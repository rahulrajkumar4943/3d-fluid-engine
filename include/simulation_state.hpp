#pragma once

#include <string>
#include <vector>

#include "raylib.h"

// this object only stores the path to the stl and the position rotation etc
// so when the state is sent to the render scene function it can use this info to render the object
// or when the state is sent to init object voxel it uses this info to init the solid voxels
struct WorldObject {

    std::string stlPath;

    Vector3 position;
    Vector3 rotation; // rotation in degrees always
    float scale;
};

struct SimulationState {

    int nx;
    int ny;
    int nz;

    // this is a vector of every cell with a value of 0 or 1 representing solid or not solid
    std::vector<uint8_t> solid;

    WorldObject object; // this is the object that goes in the wind tunnel

    // constructor to initialize simulation state
    SimulationState(int nx_, int ny_, int nz_)
        : nx(nx_), ny(ny_), nz(nz_) {

        solid.resize(nx * ny * nz, 0);
    }



    // function to get index because its in a continguous 1d array
    inline int index(int x, int y, int z) const {
        return x + y * nx + z * nx * ny;
    }
};
