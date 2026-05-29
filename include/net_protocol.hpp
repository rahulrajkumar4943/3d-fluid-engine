#pragma once

#include <cstdint>

// network parameters added in constants hpp

// quantization
// uint16 is 0 to 65535
// particle world positions are in meters
// quantize mapped over the world bounds and cast to int to save memory
// 6 instead of 12 bytes per particle

// if location is 7.5 on x axis
// 7.5/14 * 65535 gives same location but on the uint16 scale

inline uint16_t quantize(float val, float max_val) {
    // enforce boundaries
    if (val < 0.0f) {
        val = 0.0f;
    }

    if (val > max_val) {
        val = max_val;
    }

    // quantize
    return static_cast<uint16_t>((val / max_val) * config::QUANTIZE_RESOLUTION);
}

// turn quantized value into actual world value
inline float dequantize(uint16_t val, float max_val) {
    return (static_cast<float>(val) / config::QUANTIZE_RESOLUTION) * max_val;
}

// packet structure is number of particles in packet
// defined in config::NETWORK_PARTICLES_PER_PACKET in config namespace


// 4 byte alignment for same memory layout on sender and receiber
struct alignas(config::NETWORK_MEMORY_ALIGNMENT) SimPacket {

    


    uint32_t sequence;       // increase counter every network tick
    uint16_t batch_index;    // which batch within this tick or frame
    uint16_t batch_total;    // total batches in this frame so renderer konws when its done
    uint32_t count;          // particles in this packet
    uint32_t total_active;   // total number of active particles this tick
    // particle data structure soa
    uint16_t x[config::NETWORK_PARTICLES_PER_PACKET];
    uint16_t y[config::NETWORK_PARTICLES_PER_PACKET];
    uint16_t z[config::NETWORK_PARTICLES_PER_PACKET];
    uint16_t speed[config::NETWORK_PARTICLES_PER_PACKET];
};

static_assert(sizeof(SimPacket) <= 8000, "SimPacket size > 8000 bytes (safe UDP payload), reduce NETWORK_PARTICLES_PER_PACKET");
