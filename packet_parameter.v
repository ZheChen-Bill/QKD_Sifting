





// packet message


//packet width
`define PACKET_WIDTH    32


//packet type = packet_message [31:28]
`define PACKET_TYPE_WIDTH       4
`define PACKET_TYPE_LOW_INDEX   28

// sifting
`define B2A_Z_BASIS_DETECTED        `PACKET_TYPE_WIDTH'd1
`define B2A_X_BASIS_DETECTED        `PACKET_TYPE_WIDTH'd2
`define A2B_Z_BASIS_DECOY           `PACKET_TYPE_WIDTH'd3
// reconciliation
`define B2A_ASK_PARITY              `PACKET_TYPE_WIDTH'd4
`define B2A_VERIFICATION_HASHTAG    `PACKET_TYPE_WIDTH'd5
`define A2B_CORRECT_PARITY          `PACKET_TYPE_WIDTH'd6
`define A2B_TARGET_HASHTAG          `PACKET_TYPE_WIDTH'd7 //hash tag & qber
// privacy amplification
`define A2B_SECRETKEY_LENGTH        `PACKET_TYPE_WIDTH'd8
// random bit
`define A2B_EV_RANDOMBIT            `PACKET_TYPE_WIDTH'd9
`define A2B_PA_RANDOMBIT            `PACKET_TYPE_WIDTH'd10


//packet length = packet_message [27:24]
`define PACKET_LENGTH_WIDTH     4
`define PACKET_LENGTH_LOW_INDEX 24

//bit_number = packet_length*32
`define PACKET_LENGTH_257   `PACKET_LENGTH_WIDTH'b0001
`define PACKET_LENGTH_514   `PACKET_LENGTH_WIDTH'b0010
`define PACKET_LENGTH_771   `PACKET_LENGTH_WIDTH'b0100
`define PACKET_LENGTH_1028  `PACKET_LENGTH_WIDTH'b1000





//packet_info = packet_message [23:0] 
`define PACKET_INFO_WIDTH       24
`define PACKET_INFO_LOW_INDEX   0





 
