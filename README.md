# 3d-fluid-engine

## Lattice Boltzmann Wind Tunnel

A real time, deterministic 3D Lattice Boltzmann Method (LBM) fluid simulation engine with distributed 
compute/render architecture. Built in C++ using D3Q19 LBM for airflow simulation 
around a Porsche 911 GT2, with a decoupled UDP based visualization pipeline.

The system is designed to explore real-time CFD visualization and distributed simulation patterns commonly found in high-performance and low-latency systems.

## Architecture

The system is split into two independent executables communicating over UDP, the engine process for the physics and the renderer process

```
┌─────────────────────────┐         UDP (60hz)         ┌─────────────────────────┐
│      ENGINE PROCESS     │  ──────────────────────>   │    RENDERER PROCESS     │
│                         │                            │                         │
│  LBM physics at 200hz   │   quantized, decimated     │  receives non-blocking  │
│  BGK collision          │   sequence number data     │  gap detection          │
│  bounce-back / freeslip │   multi-packet batching    │  dequantize + draw      │
└─────────────────────────┘                            └─────────────────────────┘
```

## Technical Details

### Systems Concepts Demonstrated
- Producer-consumer decoupling (engine vs renderer)
- Backpressure avoidance via UDP (no blocking)
- Sequence-numbered streaming protocol
- Loss-tolerant real-time visualization
- Memory layout optimization for cache locality
- Fixed timestep simulation loop (200Hz deterministic kernel)

### Fluid Simulation

- **D3Q19 Lattice Boltzmann** with BGK collision operator
- Half-way bounce-back no-slip boundary condition on solid car surface
- Free-slip boundary condition on tunnel walls
- Zero-gradient outlet condition
- Triangle-AABB intersection voxelization of OBJ mesh into LBM solid field
- Configurable relaxation time TAU controlling fluid viscosity
- 70×30×30 voxel grid at 200hz physics tick rate



### Network Protocol
Designed around the same constraints as a market data feed handler:

- **UDP** - latency over reliability, dropping frames is preferred over blocking to maintain real-time latency bounds
- **Non-blocking socket drain** - renderer execution thread never stalls due to missed packets
- **Sequence numbers + gap detection** - receiver logs missed frames
- **Multi-packet reassembly** - full particle frame split across multiple UDP 
  packets with batch_index / batch_total header fields
- **uint16 quantization** - particle positions compressed from 12 bytes to 6 bytes
  per particle (float to uint16 mapped over world bounds), 0.21mm precision
- **Decimation** - every 4th particle transmitted, reducing bandwidth 4x with 
  minimal perceptible visual loss
- **Decoupling** - physics at 200hz, network broadcast at 60hz, renderer 
  draws at 60hz independently. Crashing or freezing with one executable does not affect the other


### Performance
- Cache-conscious flattened Structure of Arrays and Array of Structs layout for particle and fluid distribution data (improved engine performance by ~2x, 4300us -> 2200us per engine tick)
- Collision and streaming loops structured for sequential memory access
- Two buffer ping-pong scheme for LBM distributions, collision step reads `directions[]`,
  writes to `nextDirections[]`. Streaming step reads `nextDirections[]`, writes to `directions[]`
- 60k tracer particles at stable 60fps render, 200hz physics

### Tracer Particles
- Up to 150k tracer particles advected through the LBM velocity field
- Spawn deterministically, pseudo-randomly with `std::mt19937` at inlet voxels, deactivated on solid contact or domain exit
- Velocity reconstructed from LBM distributions each tick
- World-space velocity conversion from lattice units per timestep to metres/second


## Build

Requires raylib installed via homebrew.

```bash
brew install raylib
make
```

This builds two executables:

```bash
./engine    # headless LBM solver + UDP broadcaster
./renderer  # visualization client
```

Run engine and renderer executables:

```bash
./engine
./renderer
```

## Project Structure

```
.
├── include
│   ├── constants.hpp    # simulation and render parameters
│   ├── lattice_models.hpp    # D3Q19 direction vectors, weights, opposites
│   ├── net_protocol.hpp    # shared packet definition, quantization
│   ├── net_receiver.hpp    # UDP receiver, reassembly, gap detections
│   ├── net_sender.hpp    # UDP sender, decimation, batching
│   ├── physics.hpp    # LBM collision, streaming, tracer advection
│   ├── renderer.hpp    # raylib 3D renderer, camera controller
│   ├── simulation_init.hpp    # wind tunnel init, object voxelization
│   └── simulation_state.hpp    # SoA & AoS simulation state, particle storage
├── Makefile
├── src
│   ├── engine_main.cpp    # physics process
│   └── renderer_main.cpp    # render process
├── static
│   └── Porsche_911_GT2.obj    # car mesh for voxelization and rendering
├── engine    # physics engine process executable
├── renderer    # render process executable
```


## Key Design Decisions

**Why UDP over TCP:** A dropped particle frame means the renderer shows the same frame for an extra 16ms which isn't a perceivable visual difference. TCP's retransmit and head-of-line blocking would stall the renderer waiting for a missed packet which is much worse for this usecase.

**When SoA and when AoS:** Cell data is stored in flattened SoA form for cache-ordered spatial traversal. LBM distributions use a voxel-major `(cell × Q)` layout aligned loop access order in the physics engine, forming an implicit AoSoA pattern. The indexing is abstracted through `directionIndex()`, enabling a smooth future direction-major switch for SIMD without changes to simulation logic.

**Why two buffer LBM:** In place collision + streaming would cause read-after-write 
corruption. When streaming cell x=5, data from x=4 would be read which has already been 
updated this tick rather than the pre-collision value. Two buffers keep timesteps 
cleanly separated.

**Why uint16 quantization:** 0.21mm position precision is way below the visual threshold for a 14m domain. Halving bytes per particle halves network bandwidth and fits more particles per UDP packet, reducing packet count and syscall overhead.

## Performance Roadmap

Current optimization targets for further scaling:


- SIMD vectorization of voxel operations (D3Q19 collision and streaming)
- Multithreaded domain decomposition (spatial partitioning across CPU cores)
- Cache blocking / tiling for improved L1/L2 reuse in large grids

## Dependencies

- [raylib](https://www.raylib.com/) — Window, visualization, rendering, OBJ loading
- C++20
- POSIX sockets (macOS / Linux)


