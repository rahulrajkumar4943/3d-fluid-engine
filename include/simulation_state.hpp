#pragma once

#include <string>
#include <vector>
#include <random>

#include "raylib.h"

#include "lattice_models.hpp"
#include "constants.hpp"

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
    std::vector<float> directions; // these are the weights for each direction for the voxels
    // timestep being computed
    std::vector<float> nextDirections;

    // tracer particles soa
    
    std::vector<float> tracer_particles_x;
    std::vector<float> tracer_particles_y;
    std::vector<float> tracer_particles_z;

    std::vector<float> tracer_velocity_x;
    std::vector<float> tracer_velocity_y;
    std::vector<float> tracer_velocity_z;

    // spawn positions for tracer particles
    std::vector<float> tracer_spawn_x;
    std::vector<float> tracer_spawn_y;
    std::vector<float> tracer_spawn_z;
    
    // active particle count
    int tracerCount;

    WorldObject object; // this is the object that goes in the wind tunnel

    // deterministic random number generator
    std::mt19937 randomNumberGenerator;

    // spawn voxel indices
    std::vector<int> spawn_voxel_indices;
    int num_spawn_voxels;

    // constructor to initialize simulation state
    SimulationState(int nx_, int ny_, int nz_)
        : nx(nx_), ny(ny_), nz(nz_) {
        
        totalCells = nx * ny * nz;

        solid.resize(totalCells, 0);

        // q = 19 then * number of cells. flattened soa
        // all direction 1, then all direction 2 (for every voxel)
        directions.resize(Q * totalCells, 0.0f);
        nextDirections.resize(Q * totalCells, 0.0f);

        // initialise the tracer particles
        // tracer particle storage
        tracer_particles_x.resize(config::MAX_PARTICLES, 0.0f);
        tracer_particles_y.resize(config::MAX_PARTICLES, 0.0f);
        tracer_particles_z.resize(config::MAX_PARTICLES, 0.0f);

        tracer_velocity_x.resize(config::MAX_PARTICLES, 0.0f);
        tracer_velocity_y.resize(config::MAX_PARTICLES, 0.0f);
        tracer_velocity_z.resize(config::MAX_PARTICLES, 0.0f);

        // tracer spawn location constructor. this only allocates space. 
        // there is a loop in spawn particles function in physics hpp that spawns the particles
        tracer_spawn_x.resize(config::MAX_PARTICLES, 0.0f);
        tracer_spawn_y.resize(config::MAX_PARTICLES, 0.0f);
        tracer_spawn_z.resize(config::MAX_PARTICLES, 0.0f);

        tracerCount = config::INITIAL_PARTICLES;

        // seed the generator
        randomNumberGenerator.seed(config::RNG_SEED);

        // built spawn voxel list
        const int spawn_x = 1;
        num_spawn_voxels = 0;

        for (int z = 0; z < nz; z++) {
            for (int y = 0; y < ny; y++) {
                if (((y + z) % 2) != 0) {
                    continue;
                }

                int spawn_idx = index(spawn_x, y, z);
                spawn_voxel_indices.push_back(spawn_idx);
                num_spawn_voxels += 1;

            }
        }
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
