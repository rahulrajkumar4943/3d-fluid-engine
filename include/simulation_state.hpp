#pragma once

#include <string>
#include <vector>

#include "raylib.h"

#include "lattice_models.hpp"

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

    int nx; // world size params
    int ny;
    int nz;

    int totalCells;

    static constexpr int Q = lbm::D3Q19::Q; // 19

    // this is a vector of every cell with a value of 0 or 1 representing solid or not solid
    std::vector<uint8_t> solid;

    // flattened soa storage
    // [all dir 0][all dir 1]...[all dir 18]
    // current timestep
    std::vector<float> directions;
    // timestep being computed
    std::vector<float> nextDirections;


    WorldObject object; // this is the object that goes in the wind tunnel

    // constructor to initialize simulation state
    SimulationState(int nx_, int ny_, int nz_)
        : nx(nx_), ny(ny_), nz(nz_) {
        
        totalCells = nx * ny * nz;

        solid.resize(totalCells, 0);

        // q = 19 then * number of cells. flattened soa
        directions.resize(Q * totalCells, 0.0f);
        nextDirections.resize(Q * totalCells, 0.0f);
    }



    // function to get index because its in a continguous 1d array
    inline int index(int x, int y, int z) const {
        return x + y * nx + z * nx * ny;
    }

    // find index in flattened soa
    inline int directionIndex(int dir, int idx) const {
        return dir * totalCells + idx;
    }
    // same and prev function but with x y and z instead of already computed index
    inline int fIndex(int dir, int x, int y, int z) const {
        return dir * totalCells + index(x, y, z);
    }
};
