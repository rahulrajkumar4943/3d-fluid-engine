#pragma once

#include <string>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

#include "raymath.h"

#include "simulation_state.hpp"
#include "constants.hpp"


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


// init object function
// spawn object first then init the voxels
// spawn object just tells the simulation state where the object is, and which file the object is from
inline void spawnObjectIntoWorld(
    SimulationState& state,
    const std::string& stlPath,
    const Vector3& position,
    const Vector3& rotation,
    float scale
) {
    state.object.stlPath = stlPath;

    state.object.position = position;
    state.object.rotation = rotation;

    state.object.scale = scale;
}


// voxelize the object that was loaded
inline void voxelizeObject(SimulationState& state, const Model& model) {
    const int totalVoxels = state.nx * state.ny * state.nz;

    // loop through all meshes in the model
    for (int meshIndex = 0; meshIndex < model.meshCount; meshIndex++) {
        // get the current mesh (like body wheels etc)
        Mesh mesh = model.meshes[meshIndex];

        // make sure mesh contains vertices
        if (mesh.vertices == nullptr || mesh.vertexCount == 0) continue;

        // correct raylib data type for indices is unsigned short
        unsigned short* meshIndices = (unsigned short*)mesh.indices;

        // determine if the mesh is indexed or flat/sequential
        bool isIndexed = (meshIndices != nullptr && mesh.triangleCount > 0);
        
        // if no index buffer then loop based on vertex count
        int iterations;
        if (isIndexed) {
            iterations = mesh.triangleCount;
        } else {
            iterations = mesh.vertexCount / 3; // if not indexed then each triangle takes up 3 seperate consecutive points
        }

        // loop through each triangle
        for (int tri = 0; tri < iterations; tri++) {
            int i0, i1, i2; // these are the points of the traingle

            if (isIndexed) {
                // get triangle points
                i0 = meshIndices[tri * 3 + 0];
                i1 = meshIndices[tri * 3 + 1];
                i2 = meshIndices[tri * 3 + 2];
            } else {
                // get triangle points assuming each triangle has consecutive seperate points
                i0 = tri * 3 + 0;
                i1 = tri * 3 + 1;
                i2 = tri * 3 + 2;
            }

            // boundary check if a part of triangle is oob
            if (i2 * 3 + 2 >= mesh.vertexCount * 3) {
                continue; 
            }

            // get vertices from 1d array and turn into vector3
            Vector3 v0 = { mesh.vertices[i0 * 3 + 0], mesh.vertices[i0 * 3 + 1], mesh.vertices[i0 * 3 + 2] };
            Vector3 v1 = { mesh.vertices[i1 * 3 + 0], mesh.vertices[i1 * 3 + 1], mesh.vertices[i1 * 3 + 2] };
            Vector3 v2 = { mesh.vertices[i2 * 3 + 0], mesh.vertices[i2 * 3 + 1], mesh.vertices[i2 * 3 + 2] };

            // apply same transformations used for original object
            v0 = Vector3Scale(v0, state.object.scale);
            v1 = Vector3Scale(v1, state.object.scale);
            v2 = Vector3Scale(v2, state.object.scale);

            Matrix rotMat = MatrixRotateXYZ({
                state.object.rotation.x * DEG2RAD,
                state.object.rotation.y * DEG2RAD,
                state.object.rotation.z * DEG2RAD
            });

            v0 = Vector3Transform(v0, rotMat);
            v1 = Vector3Transform(v1, rotMat);
            v2 = Vector3Transform(v2, rotMat);

            v0 = Vector3Add(v0, state.object.position);
            v1 = Vector3Add(v1, state.object.position);
            v2 = Vector3Add(v2, state.object.position);

            // get triangles bounding box so that we only have to check for voxels in there instead of all voxels in the space
            float minX = std::min(v0.x, std::min(v1.x, v2.x));
            float minY = std::min(v0.y, std::min(v1.y, v2.y));
            float minZ = std::min(v0.z, std::min(v1.z, v2.z));

            float maxX = std::max(v0.x, std::max(v1.x, v2.x));
            float maxY = std::max(v0.y, std::max(v1.y, v2.y));
            float maxZ = std::max(v0.z, std::max(v1.z, v2.z));

            // coordinate boundaries
            int startX = std::max(0, std::min(state.nx - 1, (int)std::floor(minX / config::CELL_LENGTH_X)));
            int startY = std::max(0, std::min(state.ny - 1, (int)std::floor(minY / config::CELL_HEIGHT_Y)));
            int startZ = std::max(0, std::min(state.nz - 1, (int)std::floor(minZ / config::CELL_WIDTH_Z)));

            int endX   = std::max(0, std::min(state.nx - 1, (int)std::floor(maxX / config::CELL_LENGTH_X)));
            int endY   = std::max(0, std::min(state.ny - 1, (int)std::floor(maxY / config::CELL_HEIGHT_Y)));
            int endZ   = std::max(0, std::min(state.nz - 1, (int)std::floor(maxZ / config::CELL_WIDTH_Z)));

            // turn those voxels into solid
            // right now the entire bounding box is turned solid instead of just the voxels touching the triangle
            for (int z = startZ; z <= endZ; z++) {
                for (int y = startY; y <= endY; y++) {
                    for (int x = startX; x <= endX; x++) {
                        
                        int idx = state.index(x, y, z);
                        
                        if (idx >= 0 && idx < totalVoxels) {
                            state.solid[idx] = 1;
                        }
                    }
                }
            }
        }
    }
}
