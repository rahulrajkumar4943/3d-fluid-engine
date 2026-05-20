#pragma once

#include <array>

namespace lbm {

    // lattice direction struct (like a vector)
    struct Int3 {
        int x;
        int y;
        int z;
    };


    // -------- D2Q9 --------
    struct D2Q9 {

        static constexpr int Q = 9; // number of directions to compute
        static constexpr int Dimensions = 2;


        // lattice directions
        // 0  = rest
        // 1  = east
        // 2  = north
        // 3  = west
        // 4  = south
        // 5  = northeast
        // 6  = northwest
        // 7  = southwest
        // 8  = southeast


        static inline constexpr std::array<Int3, Q> directions = {{
            {0, 0, 0}, // 0
            {1, 0, 0}, // 1
            {0, 1, 0}, // 2
            {-1, 0, 0}, // 3
            {0, -1, 0}, // 4
            {1, 1, 0}, // 5
            {-1, 1, 0}, // 6
            {-1, -1, 0}, // 7
            {1, -1, 0}  // 8
        }};

        // lattice weights
        static inline constexpr std::array<float, Q> weights = {{
            4.0f / 9.0f, // center

            1.0f / 9.0f, // orthogonal
            1.0f / 9.0f,
            1.0f / 9.0f,
            1.0f / 9.0f,

            1.0f / 36.0f, // diagonal
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f
        }};


        // opposite direction index
        static inline constexpr std::array<int, Q> opposite = {{
            0,

            3,
            4,
            1,
            2,

            7,
            8,
            5,
            6
        }};
    };

    // -------- D3Q19 --------
    struct D3Q19 {

        static constexpr int Q = 19;
        static constexpr int Dimensions = 3;

        // lattice directions
        // 0  = rest
        // axis directions 1 to 6
        // edge diagonals 7 to 18


        static inline constexpr std::array<Int3, Q> directions = {{
            {0, 0, 0}, // 0

            {1, 0, 0}, // 1
            {-1, 0, 0}, // 2

            {0, 1, 0}, // 3
            {0, -1, 0}, // 4

            {0, 0, 1}, // 5
            {0, 0, -1}, // 6

            {1, 1, 0}, // 7
            {-1, -1, 0}, // 8

            {1, -1, 0}, // 9
            {-1, 1, 0}, // 10

            {1, 0, 1}, // 11
            {-1, 0, -1}, // 12

            {1, 0, -1}, // 13
            {-1, 0, 1}, // 14

            {0, 1, 1}, // 15
            {0, -1, -1}, // 16

            {0, 1, -1}, // 17
            {0, -1, 1}  // 18
        }};


        // lattice weights
        // rest      = 1/3
        // axis      = 1/18
        // diagonals = 1/36
   

        static inline constexpr std::array<float, Q> weights = {{
            1.0f / 3.0f,

            1.0f / 18.0f,
            1.0f / 18.0f,
            1.0f / 18.0f,
            1.0f / 18.0f,
            1.0f / 18.0f,
            1.0f / 18.0f,

            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f,
            1.0f / 36.0f
        }};


        // opposite direction lookup
        static inline constexpr std::array<int, Q> opposite = {{
            0,

            2,  
            1,
            4,  
            3,
            6,  
            5,

            8,  
            7,
            10,  
            9,
            12, 
            11,
            14, 
            13,
            16, 
            15,
            18, 
            17
        }};
    };

}
