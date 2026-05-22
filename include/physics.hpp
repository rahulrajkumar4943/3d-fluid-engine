#pragma once

#include <cmath>
#include <algorithm>

#include "simulation_state.hpp"
#include "lattice_models.hpp"
#include "constants.hpp"



// function to initialise lbm
// initial state has a +x airflow in every voxel (towards the outlet)
inline void initializeLBM(SimulationState& state) {
    const int Q = lbm::D3Q19::Q;
    const int N = state.totalCells;

    const float rho = config::RHO;

    const float initial_velocity_x = config::INLET_VELOCITY_X;
    const float initial_velocity_y = config::INLET_VELOCITY_Y;
    const float initial_velocity_z = config::INLET_VELOCITY_Z;

    // dot product
    const float initial_velocity_sq =
        initial_velocity_x * initial_velocity_x +
        initial_velocity_y * initial_velocity_y +
        initial_velocity_z * initial_velocity_z;

    // index is x + y * nx + z * nx * ny;
    // so x is the most contiguous so it is the fastest changing innermost loop for cache reasons
    for (int z = 0; z < state.nz; z++) {
        for (int y = 0; y < state.ny; y++) {
            for (int x = 0; x < state.nx; x++) {

                // get the index of each voxel
                int idx = state.index(x, y, z);

                // loop through each direction and assign values based on the simulation math
                for (int q = 0; q < Q; q++) {

                    // note that the directions vector in lbm namespace is different from the one in state
                    // this is a lookup table that has all the directions
                    // directionVector is an orthonormal vector of the direction q
                    const auto& direction_vector = lbm::D3Q19::directions[q];
                    // get the weight of that direction also from the lookup table
                    float direction_weight = lbm::D3Q19::weights[q];

                    // how aligned this direction is with the velocity
                    // its a dot product for how aligned the current direction is
                    // so if initial velocity points right, but current direction points up
                    // then the dot product is 0 so its not aligned at all
                    float direction_alignment =
                        direction_vector.x * initial_velocity_x +
                        direction_vector.y * initial_velocity_y +
                        direction_vector.z * initial_velocity_z;

                    // equilibrium distribution (standard D3Q19 formula)
                    // this is the flow in the current direction for the voxel being in equilibrium
                    float current_q_flow =
                        direction_weight * rho *
                        (1.0f
                        + 3.0f * direction_alignment
                        + 4.5f * direction_alignment * direction_alignment
                        - 1.5f * initial_velocity_sq);

                    // set the direction of the current voxel to the flow amount 
                    // in state directions flattened directions vector
                    state.directions[q * N + idx] = current_q_flow;
                }
            }
        }
    }

}





// spawn tracer particles
inline void spawnTracerParticles(SimulationState& state) {
    const int spawn_voxel_x = 1;
    const float spawn_point_x = (spawn_voxel_x + 0.5f) * config::CELL_LENGTH_X;
    
    // index of particle that is currently being spawned
    int particle_index = 0;

    // loop through grid
    for (int z = 0; z < state.nz; z++) {
        for (int y = 0; y < state.ny; y++) {
            
            // skip every other voxel
            if (((y + z) % 2) != 0) {
                continue;
            };

            // find next available empty particle slot
            //  where x <= 0.0f
            // be default at the end of this function we increment particle index but if that doesnt work then this is needed
            while (particle_index < state.tracerCount && state.tracer_particles_x[particle_index] > 0.0f) {
                particle_index++;
            }

            // if no more particle slots or if past limit then dont spawn
            if (particle_index >= state.tracerCount || particle_index >= config::MAX_PARTICLES) {
                return;
            }

            // spawn coords
            float spawn_y = (y + 0.5f) * config::CELL_HEIGHT_Y;
            float spawn_z = (z + 0.5f) * config::CELL_WIDTH_Z;

            state.tracer_particles_x[particle_index] = spawn_point_x;
            state.tracer_particles_y[particle_index] = spawn_y;
            state.tracer_particles_z[particle_index] = spawn_z;

            // initialise velocities to 0
            state.tracer_velocity_x[particle_index] = 0.0f;
            state.tracer_velocity_y[particle_index] = 0.0f;
            state.tracer_velocity_z[particle_index] = 0.0f;
            
            // Move to the next index for the next grid cell
            particle_index++;
        }
    }
}

// move tracer particles


// physics hot path runs every engine tick
inline void update_physics(SimulationState& state) {
    spawnTracerParticles(state);

}
