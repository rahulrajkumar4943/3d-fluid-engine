#pragma once


#include <cmath>
#include <iostream>

#include "raylib.h"
#include "raymath.h"

#include "simulation_state.hpp"
#include "constants.hpp"

class Renderer {
    public:

        // initialise window and camera
        bool init() {

            InitWindow(config::WINDOW_LENGTH, config::WINDOW_HEIGHT, "LBM Wind Tunnel");
            SetTargetFPS(config::WINDOW_FPS);

            camera.position = config::CAMERA_POSITION;
            camera.target = config::CAMERA_TARGET;
            camera.up = config::CAMERA_UP;
            camera.fovy = config::CAMERA_FOVY;
            camera.projection = CAMERA_PERSPECTIVE;

            lastMouse = GetMousePosition();

            running = !WindowShouldClose();
            return true;
        }

        // handle window events
        void handleEvents() {
            running = !WindowShouldClose();
        }

        // load the object into the render class. puts it in the carModel variable owned by render class
        // this is called once before the hotpaths start running
        // this function loads all the vao meshes then loads meshes again for an unkown reason. warning says trying to reload already loaded mesh can be ignore
        void loadObject(const std::string& path) {
            std::cout << "LOAD OBJECT FUNCTION CALLED" << std::endl;

            if (carLoaded) {
                UnloadModel(carModel);
                carLoaded = false;
            }

            carModel = LoadModel(path.c_str());
            carLoaded = true;
        }

        const Model& getModel() const {
            return carModel;
        }
       

        // render hotpath (2 functions for the hotpath)
        // one for debugging and one for actual visuals

        // actual render function (looks nice)
        void renderScene(const SimulationState& state) {

           
            updateCamera();

            BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

            const float scale = 1.0f;

            DrawCube({0, 0, 10}, 1, 1, 1, BLUE);   // +z marker // to make sure orientation is correct
            DrawCubeWires({0, 0, 10}, 1, 1, 1, BLACK);
            DrawCube({0, 0, -10}, 1, 1, 1, RED);   // -z marker // blue should be at the right when looking at it from inlet
            DrawCubeWires({0, 0, -10}, 1, 1, 1, BLACK);
            // DrawCube({0, 0, 0}, 1, 1, 1, BLACK);   // 0 marker // black is in the middle origin point 


            // draw domain, just the floor and the wall to the right of the car

            // sx sy sz are state dimensions
            float sx = config::WORLD_LENGTH_X;
            float sy = config::WORLD_HEIGHT_Y;
            float sz = config::WORLD_WIDTH_Z;

           
            Color wallColor = DARKGRAY;
            Color lineColor = BLACK;

            // floor y = 0
            {
                Vector3 a = {0, 0, 0};
                Vector3 b = {sx, 0, 0};
                Vector3 c = {sx, 0, sz};
                Vector3 d = {0, 0, sz};

                DrawCube(
                    {sx * 0.5f, -0.5 * config::FLOOR_THICKNESS, sz * 0.5f}, // offset by half of everything to account for thickness
                    sx, config::FLOOR_THICKNESS, sz, // thickness of 10cm (0.1m)
                    wallColor
                );
                DrawCubeWires(
                    {sx * 0.5f, -0.5 * config::FLOOR_THICKNESS, sz * 0.5f},
                    sx, config::FLOOR_THICKNESS, sz,
                    lineColor
                );

                


            }
            
            // wall to the right of car z = 0
            {
                Vector3 a = {0, 0, 0};
                Vector3 b = {sx, 0, 0};
                Vector3 c = {sx, sy, 0};
                Vector3 d = {0, sy, 0};

                DrawCube(
                    {sx * 0.5f, sy * 0.5f, -0.5 * config::WALL_THICKNESS},
                    sx, sy, config::WALL_THICKNESS,
                    wallColor
                );
                DrawCubeWires(
                    {sx * 0.5f, sy * 0.5f, -0.5 * config::WALL_THICKNESS},
                    sx, sy, config::WALL_THICKNESS,
                    lineColor
                );

            }

            // draw the car
            {
                // load the car only once
                // if (!carLoaded) {
                //     carModel = LoadModel(state.object.stlPath.c_str());
                //     carLoaded = true;
                // }
                // car is loaded seperately once before the hotpath. not here

                // build transform from SimulationState
                // scale
                Matrix scaleMat = MatrixScale(
                    state.object.scale,
                    state.object.scale,
                    state.object.scale
                );

                // rotation
                Matrix rotMat = MatrixRotateXYZ({
                    state.object.rotation.x * DEG2RAD,
                    state.object.rotation.y * DEG2RAD,
                    state.object.rotation.z * DEG2RAD
                });

                // position
                Matrix transMat = MatrixTranslate(
                    state.object.position.x,
                    state.object.position.y,
                    state.object.position.z
                );

                // multiply all to get transform matrix
                Matrix transform = MatrixMultiply(
                    MatrixMultiply(scaleMat, rotMat),
                    transMat
                );

                carModel.transform = transform;

                // draw model, keep scale as 1.0 because scale is already handled above
                DrawModel(carModel, Vector3Zero(), 1.0f, Color{150, 150, 150, 200});
            }

            EndMode3D();

            DrawText("Scene Renderer", 20, 20, 20, DARKGRAY);
            DrawFPS(20, 50);

            EndDrawing();

        }

        // shows solid voxels for lbm
        // only for development and debugging
        void renderLBMVoxel(const SimulationState& state) {

            updateCamera();

            BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);

            const float x_scale = config::CELL_LENGTH_X;
            const float y_scale = config::CELL_HEIGHT_Y;
            const float z_scale = config::CELL_WIDTH_Z;

            // these numbers are in meters now
            DrawCube({0, 0, 10}, 1, 1, 1, BLUE);   // +z marker // to make sure orientation is correct
            DrawCube({0, 0, -10}, 1, 1, 1, RED);   // -z marker // blue should be at the right when looking at it from inlet

            // loop through all voxels to render, inefficient, only for debugging (O(n^3))
            for (int z = 0; z < state.nz; z++) {
                for (int y = 0; y < state.ny; y++) {
                    for (int x = 0; x < state.nx; x++) {

                        // get voxel index and if its solid then draw the cube
                        int idx = state.index(x, y, z);

                        if (state.solid[idx] == 1) {

                            Vector3 pos = {
                                (x + 0.5f) * x_scale, // shifted by 0.5 so its center aligned
                                (y + 0.5f) * y_scale,
                                (z + 0.5f) * z_scale
                            };

                            DrawCube(pos, x_scale, y_scale, z_scale, DARKGRAY);
                            DrawCubeWires(pos, x_scale, y_scale, z_scale, BLACK); // lines make it go from 28 to 6 fps
                        }
                    }
                }
            }

            EndMode3D();

            DrawText("LBM Wind Tunnel - Solid Voxels", 20, 20, 20, DARKGRAY);
            DrawFPS(20, 50);

            EndDrawing();
        }

        // window state
        bool isOpen() const {
            return running;
        }

        void close() {
            running = false;
            CloseWindow();
        }

    private:

        // camera fps style controller
        void updateCamera() {

            // mouse look around when holding left button
            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {

                Vector2 mouse = GetMousePosition();
                Vector2 delta = {
                    mouse.x - lastMouse.x,
                    mouse.y - lastMouse.y
                };

                yaw   += delta.x * mouseSensitivity;
                pitch -= delta.y * mouseSensitivity;

                if (pitch > 89.0f) pitch = 89.0f;
                if (pitch < -89.0f) pitch = -89.0f;
            }

            lastMouse = GetMousePosition();

            // direction vector
            Vector3 forward;
            forward.x = cosf(DEG2RAD * yaw) * cosf(DEG2RAD * pitch);
            forward.y = sinf(DEG2RAD * pitch);
            forward.z = sinf(DEG2RAD * yaw) * cosf(DEG2RAD * pitch);

            forward = Vector3Normalize(forward);

            Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, {0, 1, 0}));

            // move with wasd
            if (IsKeyDown(KEY_W)) camera.position = Vector3Add(camera.position, Vector3Scale(forward, moveSpeed));
            if (IsKeyDown(KEY_S)) camera.position = Vector3Subtract(camera.position, Vector3Scale(forward, moveSpeed));
            if (IsKeyDown(KEY_D)) camera.position = Vector3Add(camera.position, Vector3Scale(right, moveSpeed));
            if (IsKeyDown(KEY_A)) camera.position = Vector3Subtract(camera.position, Vector3Scale(right, moveSpeed));

            // vertical movement with space and shift
            if (IsKeyDown(KEY_SPACE)) camera.position.y += moveSpeed;
            if (IsKeyDown(KEY_LEFT_SHIFT)) camera.position.y -= moveSpeed;

            // update target. if you dont do this the camera lock onto the same spot while it moves
            camera.target = Vector3Add(camera.position, forward);
        }

    private:

        bool running = true;
        Camera3D camera{};

        Vector2 lastMouse = {0, 0};

        float yaw = config::YAW;
        float pitch = config::PITCH;

        float moveSpeed = config::MOVE_SPEED;
        float mouseSensitivity = config::MOUSE_SENS;

        Model carModel;
        bool carLoaded = false;
};
