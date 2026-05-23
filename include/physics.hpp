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

    const float initial_velocity_x = config::INLET_VELOCITY_X_LUPT;
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
            while (particle_index < state.tracerCount && state.tracer_particles_x[particle_index] > 0.0f) { // if x location strictly > 0 then active
                particle_index++;
            }

            // if no more particle slots or if past limit then dont spawn
            if (particle_index >= state.tracerCount || particle_index >= config::MAX_PARTICLES) {
                return;
            }

            // debug line
            // if (particle_index == 8755) {
            //     std::cout << "Reset particle #" << 8755 << std::endl;
            // }
            
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

// right now i loop through each tracer and calculate its voxels velocity
// this is not the most efficient as i may be calculating the same voxel vel multiple times in one tick
// add some kindof cache storage to store already calculated vels for that tick to avoid unnecesary calcs

inline void updateTracerParticles(SimulationState& state)
{
    const int Q = lbm::D3Q19::Q;
    const int N = state.totalCells;

    const float dt = config::ENGINE_INTERVAL_S; // time delta between each frame in seconds

    // loop through all the tracers
    for (int i = 0; i < state.tracerCount; i++) {
        // skip inactive particles
        if (state.tracer_particles_x[i] <= 0.0f) // if the tracer is inactive then skip this iteration
            continue;

        // turn world position into voxel coordinates
        // to see which voxel the point is currently in
        float tracer_x_coord = state.tracer_particles_x[i];
        float tracer_y_coord = state.tracer_particles_y[i];
        float tracer_z_coord = state.tracer_particles_z[i];

        int tracer_x_voxel = (int)(tracer_x_coord / config::CELL_LENGTH_X);
        int tracer_y_voxel = (int)(tracer_y_coord / config::CELL_HEIGHT_Y);
        int tracer_z_voxel = (int)(tracer_z_coord / config::CELL_WIDTH_Z);

        // check bounds of voxel
        if (tracer_x_voxel < 0 || tracer_x_voxel >= state.nx ||
            tracer_y_voxel < 0 || tracer_y_voxel >= state.ny ||
            tracer_z_voxel < 0 || tracer_z_voxel >= state.nz)
        {
            state.tracer_particles_x[i] = 0.0f; // if tracer is out of bounds then make it invalid
            state.tracer_particles_y[i] = 0.0f;
            state.tracer_particles_z[i] = 0.0f;
            continue;
        }

        // index of current voxel
        int idx = state.index(tracer_x_voxel, tracer_y_voxel, tracer_z_voxel);

        // if the current voxel is solid then make the particle invalid
        if (state.solid[idx]) {
            state.tracer_particles_x[i] = 0.0f;
            continue;
        }

        // reconstruct voxel velocity from lbm
        float rho = 0.0f;
        float voxel_vel_x = 0.0f; 
        float voxel_vel_y = 0.0f; 
        float voxel_vel_z = 0.0f;

        for (int q = 0; q < Q; q++)
        {
            // value of current direction q
            float direction_flow_value = state.directions[q * N + idx];
            rho += direction_flow_value;

            // direction vector for current direction
            const auto& direction_vector = lbm::D3Q19::directions[q];

            voxel_vel_x += direction_flow_value * direction_vector.x;
            voxel_vel_y += direction_flow_value * direction_vector.y;
            voxel_vel_z += direction_flow_value * direction_vector.z;
        }

        if (rho > 1e-8f)
        {
            voxel_vel_x /= rho;
            voxel_vel_y /= rho;
            voxel_vel_z /= rho;
        }

        // voxel_vel_x exists properly


        // convert lattice velocity to world velocity
        // voxel_vel_x is in cells per tick so if voxel_vel_x = 1 that means you move 1 cell per tick
        // to change the cell into meters we multiply by the cell length
        // to change tick to second we divide by engine interval s
        float vx = voxel_vel_x * (config::CELL_LENGTH_X / config::ENGINE_INTERVAL_S); // now this is meters per second
        float vy = voxel_vel_y * (config::CELL_HEIGHT_Y / config::ENGINE_INTERVAL_S);
        float vz = voxel_vel_z * (config::CELL_WIDTH_Z / config::ENGINE_INTERVAL_S);

        // store velocities (meter/second)
        state.tracer_velocity_x[i] = vx;
        state.tracer_velocity_y[i] = vy;
        state.tracer_velocity_z[i] = vz;

        // debug statements
        // if (i == 8755) {
        //     std::cout << "vx: " << vx << std::endl;
        //     std::cout << "vy: " << vy << std::endl;
        //     std::cout << "vz: " << vz << std::endl;
        //     std::cout << "dt: " << dt << std::endl;
        // }


        // update particles positions based on velocity
        state.tracer_particles_x[i] += vx * dt;
        state.tracer_particles_y[i] += vy * dt;
        state.tracer_particles_z[i] += vz * dt;

        // if particle reaches outlet then make it inactive
        if (state.tracer_particles_x[i] >= config::WORLD_LENGTH_X)
        {
            state.tracer_particles_x[i] = 0.0f;
        }
    }
}


// physics hot path runs every engine tick
inline void update_physics(SimulationState& state) {
    spawnTracerParticles(state);
    updateTracerParticles(state);

    // debug statements
    // std::cout << "particle x vel" << state.tracer_velocity_x[8755] << std::endl;
    // std::cout << "particle x pos" << state.tracer_particles_x[8755] << std::endl;

}
