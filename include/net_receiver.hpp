#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include "net_protocol.hpp"

class NetReceiver {
    public:
        bool init() {
            // set the socket
            sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                std::cerr << "net_receiver.hpp: socket failed" << std::endl;
                return false;
            }

            // set the address
            sockaddr_in my_addr{};
            my_addr.sin_family      = AF_INET;
            my_addr.sin_port        = htons(config::NETWORK_PORT);
            my_addr.sin_addr.s_addr = INADDR_ANY;

            // bind the socket with port address and make sure it didnt fail
            if (bind(sock, reinterpret_cast<sockaddr*>(&my_addr), sizeof(my_addr)) < 0) {
                std::cerr << "net_receiver.hpp: bind failed" << std::endl;
                return false;
            }


            // use nonblocking flag so that it nothing comes dont freeze, just redraw last known state
            int flags = fcntl(sock, F_GETFL, 0);
            fcntl(sock, F_SETFL, flags | O_NONBLOCK);

            std::cout << "net_receiver.hpp: listening on port " << config::NETWORK_PORT << std::endl;
            return true;
        }

        // call this every render frame
        // takes all remaining udp packets and updates particle buffer
        void poll() {
            SimPacket incoming{};

            // keep receiving packets until its empty
            // recvfrom returns negative if none or error
            while (recvfrom(sock, &incoming, sizeof(incoming), 0, nullptr, nullptr) > 0) {

                // check packet loss
                // incoming sequence shoulld be last sequence seen + 1
                if (last_sequence_seen > 0 && incoming.sequence > last_sequence_seen + 1) {
                    uint32_t num_packets_dropped = incoming.sequence - last_sequence_seen - 1;
                    total_packets_dropped += num_packets_dropped;
                }

                // reassemble udp packets for each simulation frame
                // if its a new sequence then clear the particle buffer
                // keep adding batches until received all batches
                if (incoming.sequence != current_sequence) {
                    // if statement checks if its a new sequence then clear buffer
                    current_sequence = incoming.sequence;
                    last_sequence_seen = incoming.sequence;
                    // assembling true means we are currently receiving and assembling batches
                    // false means we are done receiving batches in that sequence or frame
                    assembling = true;

                    // only clear on batch 0, dont clear in middle of sequence
                    if (incoming.batch_index == 0) {
                        px.clear();
                        py.clear();
                        pz.clear();
                        batches_received = 0;
                        expected_batches = incoming.batch_total;
                    }
                }

                // dequantise
                for (uint32_t i = 0; i < incoming.count; i++) {
                    px.push_back(dequantize(incoming.x[i], config::WORLD_LENGTH_X));
                    py.push_back(dequantize(incoming.y[i], config::WORLD_HEIGHT_Y));
                    pz.push_back(dequantize(incoming.z[i], config::WORLD_WIDTH_Z));
                }

                batches_received++;

                // mark complete when all expected batches arrived
                // no longer assembling new batches
                if (batches_received >= expected_batches) {
                    assembling = false;
                }
            }
        }

        // particle positions for renderer to draw
        const std::vector<float>& get_px() const { 
            return px; 
        }
        const std::vector<float>& get_py() const { 
            return py; 
        }
        const std::vector<float>& get_pz() const { 
            return pz; 
        }

        // more helper functions for renderer to know the state of the receiver
        uint32_t get_packets_dropped() const { 
            return total_packets_dropped; 
        }

        bool is_assembling() const { 
            return assembling;
        }

        void close_socket() {
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
        }

        // add rule of 5
        ~NetReceiver() { 
            close_socket(); 
        }

    private:
        int sock = -1;

        // reassembly state
        uint32_t current_sequence = 0;
        uint32_t last_sequence_seen = 0;
        uint16_t expected_batches = 0;
        uint16_t batches_received = 0;
        bool assembling = false;

        // gap detection stats
        uint32_t total_packets_dropped = 0;

        // dequantized particle buffer
        std::vector<float> px;
        std::vector<float> py;
        std::vector<float> pz;
};
