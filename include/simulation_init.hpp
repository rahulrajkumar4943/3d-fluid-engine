#pragma once

#include "simulation_state.hpp"


// initialise the wind tunnel by putting 1s (solid voxels) in solid vector

inline void initWindTunnel(SimulationState& state) {
    for (int z = 0; z < state.nz; z++) {
        for (int y = 0; y < state.ny; y++) {
            for (int x = 0; x < state.nx; x++) {

                int idx = state.index(x, y, z);

                bool floor = (y == 0);
                bool roof = (y == state.ny - 1);
                bool front = (x == 0); // inlet
                bool back  = (x == state.nx - 1); // where the air flows too
                bool right_side  = (z == 0); // right side of the car assuming car faces inlet
                bool left_side  = (z == state.nz - 1); // left side of the car assuming car faces inlet


                if (floor || back || left_side || right_side || roof || front) {
                    state.solid[idx] = 1;
                }
            }
        }
    }
}
