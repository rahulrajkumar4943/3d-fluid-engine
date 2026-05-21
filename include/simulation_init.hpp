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


// forward declaration of triangle aabb intersect function
static bool TriangleAABBIntersect(
    const Vector3& v0,
    const Vector3& v1,
    const Vector3& v2,
    const Vector3& bmin,
    const Vector3& bmax
);

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
            // fixed the above statement
            // current triangle corners are v0, v1, v2
            for (int z = startZ; z <= endZ; z++) {
                for (int y = startY; y <= endY; y++) {
                    for (int x = startX; x <= endX; x++) {

                        int idx = state.index(x, y, z);

                        if (idx < 0 || idx >= totalVoxels)
                            continue;

                        Vector3 voxelMin = {
                            x * config::CELL_LENGTH_X,
                            y * config::CELL_HEIGHT_Y,
                            z * config::CELL_WIDTH_Z
                        };

                        Vector3 voxelMax = {
                            (x + 1) * config::CELL_LENGTH_X,
                            (y + 1) * config::CELL_HEIGHT_Y,
                            (z + 1) * config::CELL_WIDTH_Z
                        };

                        // if the triangle intersects the voxel then its solid
                        if (TriangleAABBIntersect(v0, v1, v2, voxelMin, voxelMax)) {
                            state.solid[idx] = 1;
                        }
                    }
                }
            }
        }
    }
}



// axis test helper function for triangle aabb intersect function
static bool axisTest(
    const Vector3& test_axis,
    float extent_x, float extent_y, float extent_z,
    const Vector3& shifted_v0,
    const Vector3& shifted_v1,
    const Vector3& shifted_v2
)
{
    // proj axis (triangle)
    float p0 = Vector3DotProduct(test_axis, shifted_v0);
    float p1 = Vector3DotProduct(test_axis, shifted_v1);
    float p2 = Vector3DotProduct(test_axis, shifted_v2);

    // compute proj of radius of aabb onto axis a
    float r = extent_x * fabsf(test_axis.x) + extent_y * fabsf(test_axis.y) + extent_z * fabsf(test_axis.z);

    // find min max prof of triangle for sat test condition
    float minP = fminf(p0, fminf(p1, p2));
    float maxP = fmaxf(p0, fmaxf(p1, p2));

    // sat test condition
    return !(minP > r || maxP < -r);
}

static bool TriangleAABBIntersect(
    const Vector3& v0,
    const Vector3& v1,
    const Vector3& v2,
    const Vector3& bmin, // box min coords
    const Vector3& bmax // box max coords
) {
    // get triangle vertices relative to box center
    Vector3 center = {
        (bmin.x + bmax.x) * 0.5f,
        (bmin.y + bmax.y) * 0.5f,
        (bmin.z + bmax.z) * 0.5f
    };

    // box half size or extents so box is c +- e
    Vector3 extent = {
        (bmax.x - bmin.x) * 0.5f,
        (bmax.y - bmin.y) * 0.5f,
        (bmax.z - bmin.z) * 0.5f
    };

    // transform triangle so box is centered at origin
    Vector3 shifted_v0 = Vector3Subtract(v0, center);
    Vector3 shifted_v1 = Vector3Subtract(v1, center);
    Vector3 shifted_v2 = Vector3Subtract(v2, center);

    // get triangle edges
    Vector3 edge0 = Vector3Subtract(shifted_v1, shifted_v0);
    Vector3 edge1 = Vector3Subtract(shifted_v2, shifted_v1);
    Vector3 edge2 = Vector3Subtract(shifted_v0, shifted_v2);



    // generate test axes
    Vector3 axes[9] = {
        Vector3CrossProduct({1,0,0}, edge0),
        Vector3CrossProduct({1,0,0}, edge1),
        Vector3CrossProduct({1,0,0}, edge2),

        Vector3CrossProduct({0,1,0}, edge0),
        Vector3CrossProduct({0,1,0}, edge1),
        Vector3CrossProduct({0,1,0}, edge2),

        Vector3CrossProduct({0,0,1}, edge0),
        Vector3CrossProduct({0,0,1}, edge1),
        Vector3CrossProduct({0,0,1}, edge2),
    };

    // test all 9 axes
    for (int i = 0; i < 9; i++) {
        Vector3 a = axes[i];

        if (fabsf(a.x) < 1e-6f && fabsf(a.y) < 1e-6f && fabsf(a.z) < 1e-6f)
            continue;

        if (!axisTest(a, extent.x, extent.y, extent.z, shifted_v0, shifted_v1, shifted_v2))
            return false;
    }

    // box normals
    if (!axisTest({1,0,0}, extent.x, extent.y, extent.z, shifted_v0, shifted_v1, shifted_v2)) return false;
    if (!axisTest({0,1,0}, extent.x, extent.y, extent.z, shifted_v0, shifted_v1, shifted_v2)) return false;
    if (!axisTest({0,0,1}, extent.x, extent.y, extent.z, shifted_v0, shifted_v1, shifted_v2)) return false;

    // triangle normal
    Vector3 n = Vector3CrossProduct(edge0, edge1);
    if (!axisTest(n, extent.x, extent.y, extent.z, shifted_v0, shifted_v1, shifted_v2)) return false;

    return true;
}
