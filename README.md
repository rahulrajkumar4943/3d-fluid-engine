# 3d-fluid-engine
Deterministic 3d fluid engine (wind tunnel CFD, lattice-Boltzmann) using distributed systems. High efficiency, low latency, written in C++


run brew install raylib
add /opt/homebrew/include to c/c++ edit configurations ui in vscode
here also change vscode c++ to 17 or 20 to avoid ide errors


fixes to be made
- [x] directions is not currently contiguous per q, it should be
- [ ] simd
- [ ] multithreading
- [ ] cache blocking



Refactored lattice and voxel direction storage from direction-major to voxel-major layout (now the directions for the same voxel are contiguous), improved cache locality for per voxel collision and reconstruction passes, improved engine iteration time by ~2x (4300us -> 2200us per engine tick)

