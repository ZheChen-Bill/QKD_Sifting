`define PULSE_WIDTH         64
`define QUBIT_WIDTH         32

`define SIFTEDKEY_WIDTH     64

// z-basis
`define QUBIT_0             2'b10
`define QUBIT_1             2'b01
`define QUBIT_NO_DETECT     2'b00
`define QUBIT_DECOY         2'b11



// x-basis
// Alice
`define NO_PULSE            2'b00
`define PULSE_0             2'b10
`define PULSE_1             2'b01
// Bob
`define DETECT_PULSE        1'b1
`define NO_DETECT_PULSE     1'b0
// compare result
`define COMPARE_1           2'b01
`define COMPARE_0           2'b10
`define COMPARE_NO          2'b00



// visibility
`define A_CHECKKEY_1_WIDTH  22
`define A_CHECKKEY_0_WIDTH  22
`define COMPARE_1_WIDTH     22
`define COMPARE_0_WIDTH     22
`define NVIS_WIDTH          22



