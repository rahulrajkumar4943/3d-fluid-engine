#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <vector>

#include "net_protocol.hpp"
#include "simulation_state.hpp"

class NetSender {
    public:
        bool init() {
            // create the udp socket, AF_INET is ipv4
            // sock dgram is datagram which is udp
            sock = socket(AF_INET, SOCK_DGRAM, 0);
            // if socket returns -1 its a failure
            if (sock < 0) {
                std::cerr << "net_sender.hpp, socket failed" << std::endl;
                return false;
            }

            renderer_addr = {};
            // htons converts from x86 little endian to network big endian
            renderer_addr.sin_family = AF_INET;
            renderer_addr.sin_port   = htons(config::NETWORK_PORT);
            // inet pton converts ip to binary
            inet_pton(AF_INET, config::NETWORK_ADDRESS, &renderer_addr.sin_addr);

            std::cout << "net_sender.hpp, sending to: " << config::NETWORK_ADDRESS << ":" << config::NETWORK_PORT << std::endl;
            return true;
        }

        // this is called in engine main in network hot loop 60hz
        void broadcast(const SimulationState& state) {

          
            // add decimated active particles to buffer
            particle_buffer_x.clear();
            particle_buffer_y.clear();
            particle_buffer_z.clear();
            collect_buf_speed.clear();

            // config::NETWORK_DECIMATION is 4 so add every 4th particle
            for (int i = 0; i < state.tracerCount; i += config::NETWORK_DECIMATION) {
                // if particle is not active then ignore itt
                if (state.tracer_particles_x[i] <= 0.0f) {
                    continue;
                }
                // add the particle to soa buffer
                particle_buffer_x.push_back(state.tracer_particles_x[i]);
                particle_buffer_y.push_back(state.tracer_particles_y[i]);
                particle_buffer_z.push_back(state.tracer_particles_z[i]);

                // get particle speed and add to buffer
                float vx = state.tracer_velocity_x[i];
                float vy = state.tracer_velocity_y[i];
                float vz = state.tracer_velocity_z[i];
                collect_buf_speed.push_back(sqrtf(vx*vx + vy*vy + vz*vz));
            }

            // total is the number of particles to send
            int num_particles_to_send = static_cast<int>(particle_buffer_x.size());

            // find number of batches needed to send all particles
            // config::NETWORK_PARTICLES_PER_PACKET - 1 makes it integer ceiling division
            // so we dont truncate down as lose the last partial batch
            int batches = (num_particles_to_send + config::NETWORK_PARTICLES_PER_PACKET - 1) / config::NETWORK_PARTICLES_PER_PACKET;
            if (batches == 0) {
                batches = 1; // always send at least one packet
            }

            // send each batch
            // each batch is one UDP packet

            // initialize packet struct (all 0 so theres no garbage from the last batch)
            SimPacket packet{};
            // headers
            packet.sequence = sequence;
            packet.batch_total = static_cast<uint16_t>(batches);
            packet.total_active = static_cast<uint32_t>(num_particles_to_send);


            // loop through all the batches
            int sent = 0; // number of particles that have already been sent
            for (int batch_num = 0; batch_num < batches; batch_num++) {
                // set the batch index
                packet.batch_index = static_cast<uint16_t>(batch_num);

                // how many particles in this batch
                int remaining = num_particles_to_send - sent;
                int count;

                if (remaining < config::NETWORK_PARTICLES_PER_PACKET) {
                    count = remaining;
                } else {
                    count = config::NETWORK_PARTICLES_PER_PACKET;
                }

                // set count for the packet
                packet.count = static_cast<uint32_t>(count);

                // quantize positions
                for (int i = 0; i < count; i++) {
                    // index of particle
                    int idx = sent + i;
                    packet.x[i] = quantize(particle_buffer_x[idx], config::WORLD_LENGTH_X);
                    packet.y[i] = quantize(particle_buffer_y[idx], config::WORLD_HEIGHT_Y);
                    packet.z[i] = quantize(particle_buffer_z[idx], config::WORLD_WIDTH_Z);
                    // and quantize speed
                    packet.speed[i] = quantize(collect_buf_speed[idx], config::QUANT_SPEED_MAX);

                }

                // send the packet
                // use sendto instead of send because udp doesnt call connect before this
                sendto(sock,
                    &packet,
                    sizeof(packet),
                    0, // send packet normally, other values are msg oob or msg dontroute etc
                    reinterpret_cast<sockaddr*>(&renderer_addr),
                    sizeof(renderer_addr));

                sent += count;
            }

            sequence++;
        }

        void close_socket() {
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
        }

        // destructor
        // need to add rule of 5 to this class
        ~NetSender() { 
            close_socket(); 
        }

    private:
        int sock = -1;
        sockaddr_in renderer_addr{};
        // increase counter every network tick
        uint32_t sequence = 0;

        // reused each broadcast to avoid per-tick allocation
        std::vector<float> particle_buffer_x;
        std::vector<float> particle_buffer_y;
        std::vector<float> particle_buffer_z;
        std::vector<float> collect_buf_speed;
};
