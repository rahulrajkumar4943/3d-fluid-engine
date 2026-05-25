#pragma once

#include <cmath>
#include <algorithm>
#include <chrono>

#include "simulation_state.hpp"
#include "lattice_models.hpp"
#include "constants.hpp"

using simulation_clock = std::chrono::high_resolution_clock;

// helper debug function to get x velocity
inline float getVoxelVelX(const SimulationState& state, int x, int y, int z) {
    const int Q = lbm::D3Q19::Q;
    const int N = state.totalCells;
    int idx = state.index(x, y, z);
    
    float rho = 0.0f;
    float ux = 0.0f;
    for (int q = 0; q < Q; q++) {
        float fq = state.directions[q * N + idx];
        rho += fq;
        ux += fq * lbm::D3Q19::directions[q].x;
    }
    if (rho > 1e-8f) ux /= rho;
    return ux;
}

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
    // z and y start from 1 because z, y = 0 is solid
    for (int z = 1; z < state.nz; z++) {
        for (int y = 1; y < state.ny; y++) {
            for (int x = 0; x < state.nx; x++) {

                // get the index of each voxel
                int idx = state.index(x, y, z);

                if (state.solid[idx] > 0) {
                    continue;
                };

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





//i need to have a spawn particle function that basically takes the count, 
// generates a random number and mods it to that count, then spawns a particle at that voxel. 
// or even better just directly generate a number between 0 and the count

// spawn tracer particles
inline void spawnTracerParticles(SimulationState& state) {

    // get next particle to be spawned

    int particles_spawned_this_tick = 0;

    // loop through all particles
    for (int particle_index = 0; particle_index < state.tracerCount; particle_index++) {
        // if the particle is active then skip
        if (state.tracer_particles_x[particle_index] > 0.0f) { // if the tracer is active then skip this iteration
            continue;
        }

        // otherwise spawn the particle
        // generate which voxel should spawn the particle
        // use uniform dist instead of mod to remove bias
        std::uniform_int_distribution<int> distribution(0, state.num_spawn_voxels - 1);
        int spawn_voxel_index = state.spawn_voxel_indices[distribution(state.randomNumberGenerator)];

        // spawn a particle at that voxel
        int spawn_voxel_x = 1; // we know voxel x is 1 because thats what all the spawn voxels were set at
        int spawn_voxel_y = (spawn_voxel_index / state.nx) % state.ny;
        int spawn_voxel_z = spawn_voxel_index / (state.nx * state.ny);

        // get the real world spawn point
        const float spawn_point_x = (spawn_voxel_x + 0.5f) * config::CELL_LENGTH_X;
        const float spawn_point_y = (spawn_voxel_y + 0.5f) * config::CELL_HEIGHT_Y;
        const float spawn_point_z = (spawn_voxel_z + 0.5f) * config::CELL_WIDTH_Z;

        // spawn the particle
        state.tracer_particles_x[particle_index] = spawn_point_x;
        state.tracer_particles_y[particle_index] = spawn_point_y;
        state.tracer_particles_z[particle_index] = spawn_point_z;

        // initialise velocities to 0
        // actually initialise x velocity to the inlet velocity
        state.tracer_velocity_x[particle_index] = config::INLET_VELOCITY_X_MPS;
        state.tracer_velocity_y[particle_index] = 0.0f;
        state.tracer_velocity_z[particle_index] = 0.0f;

        particles_spawned_this_tick += 1;

        if (particles_spawned_this_tick >= 25) {
            return;
        }

    }

}

// move tracer particles

// right now i loop through each tracer and calculate its voxels velocity
// this is not the most efficient as i may be calculating the same voxel vel multiple times in one tick
// add some kindof cache storage to store already calculated vels for that tick to avoid unnecesary calcs

inline void updateTracerParticles(SimulationState& state) {
    const int Q = lbm::D3Q19::Q;
    const int N = state.totalCells;

    const float dt = config::ENGINE_INTERVAL_S; // time delta between each frame in seconds

    // loop through all the tracers
    for (int i = 0; i < state.tracerCount; i++) {
        // skip inactive particles
        if (state.tracer_particles_x[i] <= 0.0f) { // if the tracer is inactive then skip this iteration
            continue;
        }

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
        if (state.solid[idx] > 0) {
            state.tracer_particles_x[i] = 0.0f;
            state.tracer_particles_y[i] = 0.0f;
            state.tracer_particles_z[i] = 0.0f;
            continue;
        }

        // reconstruct voxel velocity from lbm
        float rho = 0.0f;
        float voxel_vel_x = 0.0f; 
        float voxel_vel_y = 0.0f; 
        float voxel_vel_z = 0.0f;

        for (int q = 0; q < Q; q++) {
            // value of current direction q
            float direction_flow_value = state.directions[q * N + idx];
            rho += direction_flow_value;

            // direction vector for current direction
            const auto& direction_vector = lbm::D3Q19::directions[q];

            voxel_vel_x += direction_flow_value * direction_vector.x;
            voxel_vel_y += direction_flow_value * direction_vector.y;
            voxel_vel_z += direction_flow_value * direction_vector.z;
        }

        if (rho > 1e-8f) {
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
        // idx is for cur voxel so if thats 1 5 5 then print the voxel vels
        // inlet voxel
        // if (idx == state.index(1, 5, 5)) {
        //     std::cout << "voxel_vel_x: " << voxel_vel_x << std::endl;
        //     std::cout << "voxel_vel_y: " << voxel_vel_y << std::endl;
        //     std::cout << "voxel_vel_z: " << voxel_vel_z << std::endl;
        //     std::cout << "dt: " << dt << std::endl;
        // }
        // next voxel
        // if (idx == state.index(2, 5, 5)) {
        //     std::cout << "next voxel_vel_x: " << voxel_vel_x << std::endl;
        //     std::cout << "next voxel_vel_y: " << voxel_vel_y << std::endl;
        //     std::cout << "next voxel_vel_z: " << voxel_vel_z << std::endl;
        //     std::cout << "dt: " << dt << std::endl;
        // }


        // update particles positions based on velocity
        state.tracer_particles_x[i] += vx * dt;
        state.tracer_particles_y[i] += vy * dt;
        state.tracer_particles_z[i] += vz * dt;

        // if particle reaches outlet then make it inactive
        if (state.tracer_particles_x[i] >= config::WORLD_LENGTH_X) {
            state.tracer_particles_x[i] = 0.0f;
            state.tracer_particles_y[i] = 0.0f;
            state.tracer_particles_z[i] = 0.0f;
        }
    }
}


// update lbm function



inline void updateLBM(SimulationState& state) {
    const int Q   = lbm::D3Q19::Q;
    const int N   = state.totalCells;

    const float inlet_vel_x  = config::INLET_VELOCITY_X_LUPT;
    const float inlet_vel_y  = config::INLET_VELOCITY_Y;
    const float inlet_vel_z  = config::INLET_VELOCITY_Z;
    const float target_rho = config::RHO; // target density is 1.0 for each voxel

    // collision step, same way it is done in the 2d project but scaled to 3d
    // compute voxel velocities then do collision
    // loop through all voxels
    for (int z = 1; z < state.nz - 1; z++) {
        for (int y = 1; y < state.ny - 1; y++) {
            for (int x = 1; x < state.nx - 1; x++) {

                // get index of current voxel
                int idx = state.index(x, y, z);

                // if the voxel is solid then ignore it because theres no fluid action here
                if (state.solid[idx] > 0) {
                    continue;
                }

                // reconstruct the voxel velocities using lbm math

                // current fluid density of this voxel
                // sum up the flow in each direction to get this density, 
                // should add up to 1 or be near that
                float rho = 0.0f; 

                // voxel x y and z velocities
                float voxel_vel_x = 0.0f;
                float voxel_vel_y = 0.0f;
                float voxel_vel_z = 0.0f;

                // loop through all directions to rebuild velocities
                for (int q = 0; q < Q; q++) {

                    // fluid moving in direction q
                    float direction_q_flow = state.directions[state.directionIndex(q, idx)];
                    // add total flow to rho which is density of current voxel
                    rho += direction_q_flow;

                    // direction vector
                    const auto& direction_vector = lbm::D3Q19::directions[q];
                    // add to voxel velocities based on flow and current direction vector
                    voxel_vel_x += direction_q_flow * direction_vector.x;
                    voxel_vel_y += direction_q_flow * direction_vector.y;
                    voxel_vel_z += direction_q_flow * direction_vector.z;
                }

                // currently voxel vel x is momentum and has to be turned into velocity
                // momentum = rho * velocity
                if (rho <= 0.0f) {
                    rho = target_rho;
                } else { 
                    voxel_vel_x /= rho; 
                    voxel_vel_y /= rho; 
                    voxel_vel_z /= rho; 
                }

                // dot product to get velocity squared
                float velocity_sq = 
                    voxel_vel_x * voxel_vel_x 
                    + voxel_vel_y * voxel_vel_y 
                    + voxel_vel_z * voxel_vel_z;

                // collision step
                for (int q = 0; q < Q; q++) {
                    float direction_weight = lbm::D3Q19::weights[q];
                    const auto& direction_vector = lbm::D3Q19::directions[q];
                    
                    // dot product of direction and velocity of fluid
                    // so if this direction is up but velocity is right then its not algined at all
                    // dot product of orthogonal vectors is 0
                    float direction_vec_vel_alignment = 
                        direction_vector.x * voxel_vel_x 
                        + direction_vector.y * voxel_vel_y 
                        + direction_vector.z * voxel_vel_z;

                    // how much flow per direction ideally for equilibrium
                    // standard lbm function
                    // "Maxwell-Boltzmann equilibrium approximation"
                    float equilibrium_flow = 
                        direction_weight * rho 
                        * (
                            1.0f 
                            + 3.0f * direction_vec_vel_alignment 
                            + 4.5f * direction_vec_vel_alignment * direction_vec_vel_alignment 
                            - 1.5f * velocity_sq
                        );

                    // write collision result back into directions vector
                    // same formula used in 2d project
                    state.directions[state.directionIndex(q, idx)] += (1 / config::TAU) * (equilibrium_flow - state.directions[state.directionIndex(q, idx)]);
                }
            }
        }
    }

    // streaming step
    // write results into nextDirections vector 
    // handle inlet and outlet here (that is what i did in 2d project and it worked)
    // loop through all voxels and get the index
    // read from current directions and write into next directions
    for (int z = 1; z < state.nz - 1; z++) {
        for (int y = 1; y < state.ny - 1; y++) {
            for (int x = 1; x < state.nx - 1; x++) {

                int idx = state.index(x, y, z);
                if (state.solid[idx] > 0) {
                    continue;
                }

                // for that voxel loop through each direction
                for (int q = 0; q < Q; q++) {

                    const auto& direction_vector = lbm::D3Q19::directions[q];

                    // inlet is at x = 1 
                    // set inlet speed at equilibrium into next directions
                    // for x = 1
                    if (x == 1) {
                        float direction_vec_vel_alignment = 
                            direction_vector.x * inlet_vel_x 
                            + direction_vector.y * inlet_vel_y 
                            + direction_vector.z * inlet_vel_z;

                        float direction_velocity_sq = 
                            inlet_vel_x * inlet_vel_x 
                            + inlet_vel_y * inlet_vel_y 
                            + inlet_vel_z * inlet_vel_z;

                        state.nextDirections[state.directionIndex(q, idx)] =
                            lbm::D3Q19::weights[q] * target_rho * (
                                1.0f 
                                + 3.0f * direction_vec_vel_alignment 
                                + 4.5f * direction_vec_vel_alignment * direction_vec_vel_alignment 
                                - 1.5f * direction_velocity_sq
                            );
                        // dont do any other streaming steps for the inlet
                        continue;
                    }

                    // outlet is at x = nx-2 copy from voxel just before it
                    if (x == state.nx - 2) {
                        int src_idx = state.index(x-1, y, z);
                        state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(q, src_idx)];
                        continue;
                    }

                    // upstream neighbour based on direction vector
                    int source_x = x - direction_vector.x;
                    int source_y = y - direction_vector.y;
                    int source_z = z - direction_vector.z;

                    // check if current voxel is in bounds
                    bool inBounds =
                        source_x >= 0 && source_x < state.nx &&
                        source_y >= 0 && source_y < state.ny &&
                        source_z >= 0 && source_z < state.nz;

                    // if not inbounds then dont do anything otherwise do the normal streaming step
                    if (!inBounds) {
                        state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(q, idx)];
                        continue;
                    }

                    int src = state.index(source_x, source_y, source_z);



                    if (state.solid[src] == 1) {

                        // if its a car then src is 1 and no slip (half bounce)
                        int opp = lbm::D3Q19::opposite[q];
                        state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(opp, idx)];

                    } else if (state.solid[src] == 2) {

                        // if its a wall then free slip and only flip the wall normal component
                        // keep overall velocity

                        // the voxel that was going to be pulled from is a wall, flip the distribution
                     
                        // check which wall it was
                        // if source y is not current y then y changed and thats the wall that was solid
                        bool is_y_wall_solid = (source_y != y);
                        bool is_z_wall_solid = (source_z != z);

                        // reflect only the component orthogonal to the wall
                        int reflected_x = direction_vector.x; // not checking for x because walls are only side walls and roof
                        int reflected_y = direction_vector.y;
                        int reflected_z = direction_vector.z;

                        // if it was the y wall that was solid then flip all the y directions for the fluid
                        if (is_y_wall_solid) {
                            reflected_y = -1 * direction_vector.y;
                        }

                        if (is_z_wall_solid) {
                            reflected_z = -1 * direction_vector.z;
                        }


                        int reflected_q = -1; // -1 means not initilised yet
                        // becuase later we have to check reflected_q >= 0
                        
                        for (int possible_reflected_direction = 0; possible_reflected_direction < Q; possible_reflected_direction++) {

                            const auto& reflected_direction_vector = lbm::D3Q19::directions[possible_reflected_direction];
                            // if this direction matches what we have then set this to reflected q
                            if (reflected_direction_vector.x == reflected_x && reflected_direction_vector.y == reflected_y && reflected_direction_vector.z == reflected_z) {
                                reflected_q = possible_reflected_direction;
                                break;
                            }
                        }
                        // if reflection exists use it otherwise keep original value
                        if (reflected_q >= 0) {
                            state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(reflected_q, idx)];
                        } else {
                            state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(q, idx)];
                        }
                    } else {
                        // if its a fluid then normal stream
                        state.nextDirections[state.directionIndex(q, idx)] = state.directions[state.directionIndex(q, src)];
                    }


                }
            }
        }
    }

    // swap
    // set next directions to current direction after all calculations for current tick are done
    std::swap(state.directions, state.nextDirections);
}

// need to add something that sets the inlet velocity every tick because rn the model
// slows down too much

inline void enforceInletVelocity(SimulationState& state) {
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
    // set x = 1 because thats where all the inlet voxels are
    int x = 1;
    // z and y start from 1 because 0 is solid
    for (int z = 1; z < state.nz - 1; z++) {
        for (int y = 1; y < state.ny - 1; y++) {
            

                // get the index of each voxel
                int idx = state.index(x, y, z);

                if (state.solid[idx] > 0) {
                    continue;
                };

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
                    state.nextDirections[q * N + idx] = current_q_flow;
                }
            
        }
    }

}



// physics hot path runs every engine tick
inline void update_physics(SimulationState& state) {



    // time the update lbm function to see if cache vs ram
    // auto t0 = simulation_clock::now();
    updateLBM(state);
    // auto t1 = simulation_clock::now();
    // auto us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    // std::cout << "updateLBM: " << us << " us\n";




    // dont run this enforce inlet function because it is being enforced in update lbm now
    // enforceInletVelocity(state);
    spawnTracerParticles(state);
    updateTracerParticles(state);


    // debug statements
    // std::cout << "particle x vel" << state.tracer_velocity_x[8755] << std::endl;
    // std::cout << "particle x pos" << state.tracer_particles_x[8755] << std::endl;

}
