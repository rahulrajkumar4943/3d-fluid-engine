#pragma once
#include <vector>

struct SimulationState {

    int nx;
    int ny;
    int nz;

    // this is a vector of every cell with a value of 0 or 1 representing solid or not solid
    std::vector<uint8_t> solid;

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
