




`timescale 1ns/1ps


`include "../sifting_parameter.v"
`include "packet_parameter.v"


module top_A_sifting (
    input clk,                              //clk
    input rst_n,                            //reset


    input start_A_sifting,                  //start to sifting
    input start_A_TX, //start A TX
    
    output wait_A_TX, //indicate A is in wait state
    output A_sifting_finish,                        //sifting is done

    output reset_sift_parameter,

    input Zbasis_Xbasis_fifo_full,


    // visibility parameter
    output [`NVIS_WIDTH-1:0] nvis,                  //nvis
    output [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1,  //A_checkkey_1
    output [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0,  //A_checkkey_0
    output [`COMPARE_1_WIDTH-1:0] A_compare_1,      //A_compare_1
    output [`COMPARE_0_WIDTH-1:0] A_compare_0,      //A_compare_0
    output A_visibility_valid,                      //visibility parameter is valid
    
    
    // // visibility parameter fifo
    // input visibility_rd_clk,
    // input visibility_rd_en,
    // output [119 : 0] visibility_rd_dout,
    // output visibility_rd_empty,
    // output visibility_rd_valid,


    // Alice qubit BRAM (input)
    // width = 64 , depth = 32768
    // port B
    input wire [63:0] Qubit_doutb,     //Z-basis detected qubit from AXI manager
    output wire [14:0] Qubit_addrb,    //0~32767
    output wire Qubit_clkb,
    output wire Qubit_enb,                    //1'b1
    output wire Qubit_rstb,                   //1'b0
    output wire [7:0] Qubit_web,              //8 bit write enable , 8'b0



    // A_RX_Xbasis_detected FIFO (input)
    // width = 64 , depth = 32768
    output wire A_RX_Xbasis_detected_rd_clk,
    output wire A_RX_Xbasis_detected_rd_en,
    input wire [63:0] A_RX_Xbasis_detected_rd_dout,
    input wire A_RX_Xbasis_detected_empty,
    input wire A_RX_Xbasis_detected_rd_valid,
    input wire A_RX_Xbasis_detected_full,


    // A_RX_Zbasis_detected FIFO (input)
    // width = 32 , depth = 32768
    output wire A_RX_Zbasis_detected_rd_clk,
    output wire A_RX_Zbasis_detected_rd_en,
    input wire [31:0] A_RX_Zbasis_detected_rd_dout,
    input wire A_RX_Zbasis_detected_empty,
    input wire A_RX_Zbasis_detected_rd_valid,
    input wire A_RX_Zbasis_detected_full,


    // A_TX_decoy FIFO (output)
    // width = 32 , depth = 2048
    output wire A_TX_decoy_wr_clk,
    output reg [31:0] A_TX_decoy_wr_din,
    output reg A_TX_decoy_wr_en,
    input wire A_TX_decoy_full,
    input wire A_TX_decoy_wr_ack,
    input wire A_TX_decoy_empty,


    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output reg [63:0] Asiftedkey_dina,     //Alice sifted key 
    output reg [14:0] Asiftedkey_addra,    //0~32767
    output wire Asiftedkey_clka,
    output wire Asiftedkey_ena,                    //1'b1
    output reg Asiftedkey_wea,              //

    // output sifting state
    output A_sift_state
);


//****************************** BRAM setup ******************************
    // Alice qubit BRAM (input)
    // width = 64 , depth = 32768
    // port B
    assign Qubit_clkb = clk;
    assign Qubit_enb = 1'b1;
    assign Qubit_web = 8'b0;

    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    assign Asiftedkey_clka = clk;
    assign Asiftedkey_ena = 1'b1;
//****************************** BRAM setup ******************************
//****************************** FIFO setup ******************************
    // A_RX_Xbasis_detected FIFO (input)
    // width = 64 , depth = 32768
    assign A_RX_Xbasis_detected_rd_clk = clk;

    // A_RX_Zbasis_detected FIFO (input)
    // width = 32 , depth = 32768
    assign A_RX_Zbasis_detected_rd_clk = clk;

    // A_TX_decoy FIFO (output)
    // width = 32 , depth = 2048
    assign A_TX_decoy_wr_clk = clk;
//****************************** FIFO setup ******************************
//****************************** DFF for bram output ******************************
    reg [63:0] Qubit_doutb_ff;

    always @(posedge clk ) begin
        if (~rst_n) begin
            Qubit_doutb_ff <= 64'b0;
        end
        else begin
            Qubit_doutb_ff <= Qubit_doutb;
        end
    end
//****************************** DFF for bram output ******************************








// //****************************** visibility_fifo ******************************
//     wire visibility_wr_clk;
//     wire [119 : 0] visibility_wr_din;
//     wire visibility_wr_en;
//     wire visibility_wr_full;
//     wire visibility_wr_ack;

//     // wire visibility_rd_clk;
//     // wire visibility_rd_en;
//     // wire [119 : 0] visibility_rd_dout;
//     // wire visibility_rd_empty;
//     // wire visibility_rd_valid;

//     wire visibility_wr_rst_busy;
//     wire visibility_rd_rst_busy;


//     assign visibility_wr_clk = clk;
//     assign visibility_wr_din = {2'b0 , nvis,
//                                 2'b0 , A_checkkey_1,
//                                 2'b0 , A_checkkey_0,
//                                 2'b0 , A_compare_1,
//                                 2'b0 , A_compare_0};
//     assign visibility_wr_en = A_visibility_valid;


//     visibility_fifo visibility_parameter_fifo (
//         .srst(rst_n),                // input wire srst

//         .wr_clk(visibility_wr_clk),            // input wire wr_clk
//         .din(visibility_wr_din),                  // input wire [119 : 0] din
//         .wr_en(visibility_wr_en),              // input wire wr_en
//         .full(visibility_wr_full),                // output wire full
//         .wr_ack(visibility_wr_ack),            // output wire wr_ack

//         .rd_clk(visibility_rd_clk),            // input wire rd_clk
//         .rd_en(visibility_rd_en),              // input wire rd_en
//         .dout(visibility_rd_dout),                // output wire [119 : 0] dout
//         .empty(visibility_rd_empty),              // output wire empty
//         .valid(visibility_rd_valid),              // output wire valid
        
//         .wr_rst_busy(visibility_wr_rst_busy),  // output wire wr_rst_busy
//         .rd_rst_busy(visibility_rd_rst_busy)  // output wire rd_rst_busy
//     );
// //****************************** visibility_fifo ******************************








//****************************** A sift fsm ******************************
    //fsm input
    wire sift_decoy_nodetected_finish;

    //fsm output
    wire sift_decoy_nodetected_en;
    //wire reset_sift_parameter;

    wire [3:0] A_sift_state;

    A_sifting_fsm Asift_fsm(
        .clk(clk),
        .rst_n(rst_n),

        .start_A_sifting(start_A_sifting),
        .start_A_TX(start_A_TX),
        .A_RX_Xbasis_detected_full(Zbasis_Xbasis_fifo_full),
        .A_RX_Zbasis_detected_full(Zbasis_Xbasis_fifo_full),
        .sift_decoy_nodetected_finish(sift_decoy_nodetected_finish),

        .wait_A_TX(wait_A_TX),
        .sift_decoy_nodetected_en(sift_decoy_nodetected_en),
        // .A_visibility_valid(A_visibility_valid),
        .reset_sift_parameter(reset_sift_parameter),
        .A_sifting_finish(A_sifting_finish),
        .A_sift_state(A_sift_state)
    );

//****************************** A sift fsm ******************************









//****************************** send decoy pos ******************************

    //fsm input 
    wire decoy_round_count_finish;
    wire decoy_last_round;



    //fsm output
    wire decoy_round_count_en;
    wire reset_decoy_cnt;
    wire send_decoy_finish;
    wire reset_decoy_round_cnt;

    wire [19:0] decoy_round_addr_offset;
    wire [7:0] decoy_round;
    wire [3:0] decoy_state;

    send_decoy_fsm decoy_fsm(
        .clk(clk),
        .rst_n(rst_n),

        .sift_decoy_nodetected_en(sift_decoy_nodetected_en),
        .A_TX_decoy_empty(A_TX_decoy_empty),
        .decoy_round_count_finish(decoy_round_count_finish),
        .decoy_last_round(decoy_last_round),

        .decoy_round_count_en(decoy_round_count_en),
        .reset_decoy_cnt(reset_decoy_cnt),
        .send_decoy_finish(sift_decoy_nodetected_finish),

        .reset_decoy_round_cnt(reset_decoy_round_cnt),

        .decoy_round_addr_offset(decoy_round_addr_offset),
        .decoy_round(decoy_round),
        .decoy_state(decoy_state)
    );

    //output wire [14:0] Qubit_addrb,    //0~32767
    reg [10:0] decoy_addr_cnt;
    always @(posedge clk ) begin
        if (~rst_n) begin
            decoy_addr_cnt <= 11'b0;
        end
        else if (reset_decoy_round_cnt) begin
            decoy_addr_cnt <= 11'b0;
        end
        else if (decoy_round_count_en&&(decoy_addr_cnt<1023)) begin
            decoy_addr_cnt <= decoy_addr_cnt + 1;
        end
        else begin
            decoy_addr_cnt <= decoy_addr_cnt;
        end
    end

    assign Qubit_addrb = (decoy_addr_cnt + decoy_round_addr_offset);

    assign decoy_round_count_finish = (decoy_addr_cnt==1023);


    
    //reg [63:0] Qubit_doutb_ff;
    reg [31:0] decoy_32pos;
    integer idx;
    always @(*) begin
        /*
        decoy_32pos[31] = ~(Qubit_doutb_ff[63] & Qubit_doutb_ff[62]);
        decoy_32pos[30] = ~(Qubit_doutb_ff[61] & Qubit_doutb_ff[60]);
        decoy_32pos[29] = ~(Qubit_doutb_ff[59] & Qubit_doutb_ff[58]);
        decoy_32pos[28] = ~(Qubit_doutb_ff[57] & Qubit_doutb_ff[56]);


        decoy_32pos[1] = ~(Qubit_doutb_ff[3] & Qubit_doutb_ff[2]);
        decoy_32pos[0] = ~(Qubit_doutb_ff[1] & Qubit_doutb_ff[0]);

        */
        for (idx = 31; idx >= 0 ; idx = idx - 1) begin
            decoy_32pos[idx] = ~(Qubit_doutb_ff[(idx<<1)+1] & Qubit_doutb_ff[(idx<<1)]);
        end
    end


    wire [31:0] decoy_A2B_header;
    assign decoy_A2B_header =   {`A2B_Z_BASIS_DECOY,
                                `PACKET_LENGTH_1028,
                                16'b0,
                                decoy_round};
    wire decoy_A2B_header_write_en;
    assign decoy_A2B_header_write_en = (decoy_addr_cnt==1)? 1'b1:1'b0;


    reg decoy_round_count_en_delay;
    reg decoy_wr_en;
    always @(posedge clk ) begin
        if (~rst_n) begin
            decoy_round_count_en_delay <= 1'b0;
            decoy_wr_en <= 1'b0;
        end
        else begin
            decoy_round_count_en_delay <= decoy_round_count_en;
            decoy_wr_en <= decoy_round_count_en_delay;
        end
    end

    wire [31:0] decoy_wr_din;
    assign decoy_wr_din = (decoy_wr_en)? decoy_32pos:32'b0;

    // last round
    assign decoy_last_round = (decoy_round==8'd31)? 1'b1:1'b0;
    
//****************************** send decoy pos ******************************













//****************************** write A2B FIFO ******************************

    always @(posedge clk ) begin
        if (~rst_n) begin
            A_TX_decoy_wr_din <= 32'b0;
            A_TX_decoy_wr_en <= 1'b0;
        end
        else if (sift_decoy_nodetected_en && decoy_A2B_header_write_en) begin
            A_TX_decoy_wr_din <= decoy_A2B_header;
            A_TX_decoy_wr_en <= 1'b1;
        end
        else if (sift_decoy_nodetected_en) begin
            A_TX_decoy_wr_din <= decoy_wr_din;
            A_TX_decoy_wr_en <= decoy_wr_en;
        end

        else begin
            A_TX_decoy_wr_din <= 32'b0;
            A_TX_decoy_wr_en <= 1'b0;
        end
    end

//****************************** write A2B FIFO ******************************






//****************************** qubit_deletion ******************************
    //input 
    wire deletion_en;
    wire [`PULSE_WIDTH-1:0] pulse_64;
    wire [`QUBIT_WIDTH-1:0] B_detected_pos_32;
    wire [`QUBIT_WIDTH-1:0] A_notdecoy_pos_32;

    // output
    wire sifted_key_64_valid;
    wire [`SIFTEDKEY_WIDTH-1:0] sifted_key_64;



    assign deletion_en = decoy_wr_en;
    assign pulse_64 = Qubit_doutb_ff;
    assign B_detected_pos_32 = A_RX_Zbasis_detected_rd_dout;
    assign A_RX_Zbasis_detected_rd_en = decoy_wr_en;
    assign A_notdecoy_pos_32 = decoy_32pos;


    A_qubit_deletion A_deletion(
        .clk(clk),
        .rst_n(rst_n),

        .deletion_en(deletion_en),
        .pulse_64(pulse_64),
        .B_detected_pos_32(B_detected_pos_32),
        .A_notdecoy_pos_32(A_notdecoy_pos_32),

        
        .sifted_key_64_valid(sifted_key_64_valid),
        .sifted_key_64(sifted_key_64)

    );
//****************************** qubit_deletion ******************************


//****************************** visibility ******************************

    // input 
    wire visibility_en;
    wire [`PULSE_WIDTH-1:0] A_pulse_64;
    wire [`PULSE_WIDTH-1:0] B_xbasis_64;

    wire reset_visibility_parameter;
    //wire reset_sift_parameter;


    assign visibility_en = decoy_wr_en;
    assign A_pulse_64 = Qubit_doutb_ff;
    assign B_xbasis_64 = A_RX_Xbasis_detected_rd_dout;
    assign A_RX_Xbasis_detected_rd_en = decoy_wr_en;
    assign reset_visibility_parameter = (Asiftedkey_wea & (&Asiftedkey_addra[6:0]));


    assign A_visibility_valid = (Asiftedkey_wea & (&Asiftedkey_addra[6:0]));

    A_visibility visibility(
        .clk(clk),
        .rst_n(rst_n),

        .visibility_en(visibility_en),

        .A_pulse_64(A_pulse_64),
        .B_xbasis_64(B_xbasis_64),

        .reset_sift_parameter(reset_sift_parameter),
        .reset_visibility_parameter(reset_visibility_parameter),


        .nvis(nvis),                  //nvis
        .A_checkkey_1(A_checkkey_1),  //A_checkkey_1
        .A_checkkey_0(A_checkkey_0),  //A_checkkey_0
        .A_compare_1(A_compare_1),      //A_compare_1
        .A_compare_0(A_compare_0)      //A_compare_0
    );


//****************************** visibility ******************************




//****************************** sifted key output ******************************

    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    /*
    output reg [63:0] Asiftedkey_dina,     //Alice sifted key 
    output reg [14:0] Asiftedkey_addra,    //0~32767
    output wire Asiftedkey_clka,
    output wire Asiftedkey_ena,                    //1'b1
    output reg Asiftedkey_wea              //
    */

    always @(posedge clk ) begin
        if (~rst_n) begin
            Asiftedkey_addra <= 15'b1111_1111_1111_111;
            Asiftedkey_dina <= 64'b0;
            Asiftedkey_wea <= 1'b0;
        end
        else if (sifted_key_64_valid) begin
            Asiftedkey_addra <= Asiftedkey_addra + 1;
            Asiftedkey_dina <= sifted_key_64;
            Asiftedkey_wea <= 1'b1;
        end
        else begin
            Asiftedkey_addra <= Asiftedkey_addra;
            Asiftedkey_dina <= 64'b0;
            Asiftedkey_wea <= 1'b0;
        end
    end

//****************************** sifted key output ******************************
















endmodule
















module A_visibility (
    input clk,
    input rst_n,

    input visibility_en,
    input [`PULSE_WIDTH-1:0] A_pulse_64,
    input [`PULSE_WIDTH-1:0] B_xbasis_64,

    input reset_sift_parameter,
    input reset_visibility_parameter,


    output reg [`NVIS_WIDTH-1:0] nvis,                  //nvis
    output reg [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1,  //A_checkkey_1
    output reg [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0,  //A_checkkey_0
    output reg [`COMPARE_1_WIDTH-1:0] A_compare_1,      //A_compare_1
    output reg [`COMPARE_0_WIDTH-1:0] A_compare_0      //A_compare_0

);

    // delay 2 cycle
    reg visibility_en_delay_1 , visibility_en_delay_2;
    reg [`PULSE_WIDTH-1:0] A_pulse_64_delay_1 , A_pulse_64_delay_2;
    reg [`PULSE_WIDTH-1:0] B_xbasis_64_delay_1 , B_xbasis_64_delay_2;
    always @(posedge clk ) begin
        visibility_en_delay_1 <= visibility_en;
        visibility_en_delay_2 <= visibility_en_delay_1;
        A_pulse_64_delay_1 <= A_pulse_64;
        A_pulse_64_delay_2 <= A_pulse_64_delay_1;
        B_xbasis_64_delay_1 <= B_xbasis_64;
        B_xbasis_64_delay_2 <= B_xbasis_64_delay_1;
    end



    integer idx;
    wire [`PULSE_WIDTH-1:0] delay_pulse_64;
    reg [1:0] A_check_key [0:`PULSE_WIDTH-1];
    reg [1:0] A_compare_key [0:`PULSE_WIDTH-1];


    reg last_round_last_pulse;

    // last round last pulse
    always @(posedge clk ) begin
        if (~rst_n) begin
            last_round_last_pulse <= 1'b0;
        end
        else if (reset_sift_parameter) begin
            last_round_last_pulse <= 1'b0;
        end
        else if (visibility_en_delay_2) begin
            last_round_last_pulse <= A_pulse_64_delay_2[0];
        end
        else begin
            last_round_last_pulse <= last_round_last_pulse;
        end
    end



    // delay_pulse_64
    assign delay_pulse_64 = {last_round_last_pulse, A_pulse_64_delay_2[63:1]};

    // A_check_key
    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            case ({A_pulse_64_delay_2[idx] , delay_pulse_64[idx]})
                2'b00: A_check_key[idx][1:0] = `NO_PULSE;
                2'b01: A_check_key[idx][1:0] = `PULSE_1;
                2'b10: A_check_key[idx][1:0] = `PULSE_1;
                2'b11: A_check_key[idx][1:0] = `PULSE_0;

                default: A_check_key[idx][1:0] = `NO_PULSE;
            endcase
        end
    end

    // A_compare_key
    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            case ({B_xbasis_64_delay_2[idx] , A_check_key[idx][1:0]})
                3'b000: A_compare_key[idx][1:0] = `COMPARE_NO;
                3'b001: A_compare_key[idx][1:0] = `COMPARE_NO;
                3'b010: A_compare_key[idx][1:0] = `COMPARE_NO;


                3'b100: A_compare_key[idx][1:0] = `COMPARE_NO;
                3'b101: A_compare_key[idx][1:0] = `COMPARE_1;
                3'b110: A_compare_key[idx][1:0] = `COMPARE_0;


                default: A_compare_key[idx][1:0] = `COMPARE_NO;
            endcase
        end
    end

    
    // nvis
    integer one_idx;
    reg [5:0] nvis_ones;
    always @(*) begin
        if (visibility_en_delay_2) begin
            nvis_ones = 0;
            for(one_idx=0; one_idx<`PULSE_WIDTH; one_idx=one_idx+1)   //for all the bits.
                nvis_ones = nvis_ones + B_xbasis_64_delay_2[one_idx]; //Add the bit to the count.
        end
        else begin
            nvis_ones = 0;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            nvis <= 0;
        end
        else if (reset_visibility_parameter) begin
            nvis <= nvis_ones;
        end
        else if (visibility_en_delay_2) begin
            nvis <= nvis + nvis_ones;
        end
        else begin
            nvis <= nvis;
        end
    end



    // A_checkkey_1
    reg [5:0] checkkey_ones;
    reg [63:0] A_check_key_pulse_1;

    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            A_check_key_pulse_1[idx] = A_check_key[idx][0];
        end
    end

    always @(*) begin
        if (visibility_en_delay_2) begin
            checkkey_ones = 0;
            for(one_idx=0; one_idx<`PULSE_WIDTH; one_idx=one_idx+1)   //for all the bits.
                checkkey_ones = checkkey_ones + A_check_key_pulse_1[one_idx]; //Add the bit to the count.
        end
        else begin
            checkkey_ones = 0;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            A_checkkey_1 <= 0;
        end
        else if (reset_visibility_parameter) begin
            A_checkkey_1 <= checkkey_ones;
        end
        else if (visibility_en_delay_2) begin
            A_checkkey_1 <= A_checkkey_1 + checkkey_ones;
        end
        else begin
            A_checkkey_1 <= A_checkkey_1;
        end
    end



    // A_checkkey_0
    reg [5:0] checkkey_zeros;
    reg [63:0] A_check_key_pulse_0;

    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            A_check_key_pulse_0[idx] = A_check_key[idx][1];
        end
    end

    always @(*) begin
        if (visibility_en_delay_2) begin
            checkkey_zeros = 0;
            for(one_idx=0; one_idx<`PULSE_WIDTH; one_idx=one_idx+1)   //for all the bits.
                checkkey_zeros = checkkey_zeros + A_check_key_pulse_0[one_idx]; //Add the bit to the count.
        end
        else begin
            checkkey_zeros = 0;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            A_checkkey_0 <= 0;
        end
        else if (reset_visibility_parameter) begin
            A_checkkey_0 <= checkkey_zeros;
        end
        else if (visibility_en_delay_2) begin
            A_checkkey_0 <= A_checkkey_0 + checkkey_zeros;
        end
        else begin
            A_checkkey_0 <= A_checkkey_0;
        end
    end

    // A_compare_1
    reg [5:0] comparekey_ones;
    reg [63:0] compare_key_1;

    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            compare_key_1[idx] = A_compare_key[idx][0];
        end
    end

    always @(*) begin
        if (visibility_en_delay_2) begin
            comparekey_ones = 0;
            for(one_idx=0; one_idx<`PULSE_WIDTH; one_idx=one_idx+1)   //for all the bits.
                comparekey_ones = comparekey_ones + compare_key_1[one_idx]; //Add the bit to the count.
        end
        else begin
            comparekey_ones = 0;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            A_compare_1 <= 0;
        end
        else if (reset_visibility_parameter) begin
            A_compare_1 <= comparekey_ones;
        end
        else if (visibility_en_delay_2) begin
            A_compare_1 <= A_compare_1 + comparekey_ones;
        end
        else begin
            A_compare_1 <= A_compare_1;
        end
    end

    // A_compare_0
    reg [5:0] comparekey_zeros;
    reg [63:0] compare_key_0;

    always @(*) begin
        for (idx=63; idx>=0; idx=idx-1) begin
            compare_key_0[idx] = A_compare_key[idx][1];
        end
    end

    always @(*) begin
        if (visibility_en_delay_2) begin
            comparekey_zeros = 0;
            for(one_idx=0; one_idx<`PULSE_WIDTH; one_idx=one_idx+1)   //for all the bits.
                comparekey_zeros = comparekey_zeros + compare_key_0[one_idx]; //Add the bit to the count.
        end
        else begin
            comparekey_zeros = 0;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            A_compare_0 <= 0;
        end
        else if (reset_visibility_parameter) begin
            A_compare_0 <= comparekey_zeros;
        end
        else if (visibility_en_delay_2) begin
            A_compare_0 <= A_compare_0 + comparekey_zeros;
        end
        else begin
            A_compare_0 <= A_compare_0;
        end
    end

endmodule














module A_qubit_deletion(
    input clk,
    input rst_n,

    input deletion_en,
    input [`PULSE_WIDTH-1:0] pulse_64,
    input [`QUBIT_WIDTH-1:0] B_detected_pos_32,
    input [`QUBIT_WIDTH-1:0] A_notdecoy_pos_32,

    
    output reg sifted_key_64_valid,
    output reg [`SIFTEDKEY_WIDTH-1:0] sifted_key_64

);

    integer i,j;
    
    reg [6:0] sifted_key_offset_128;
    reg [127:0] sifted_key_128;
    reg [1:0] sifted_key_128_valid;

    wire [`QUBIT_WIDTH-1:0] position_keep_32;
    reg [4:0] sifted_key_index_32[0:31];
    reg [4:0] sifted_key_number_32;


    // position keep
    assign position_keep_32 = (deletion_en)? (A_notdecoy_pos_32 & B_detected_pos_32):32'b0;


    // sifted_key_number_32
    integer one_idx;
    always @(*) begin
        if (deletion_en) begin
            sifted_key_number_32 = 0;
            for(one_idx=0; one_idx<`QUBIT_WIDTH; one_idx=one_idx+1)   //for all the bits.
                sifted_key_number_32 = sifted_key_number_32 + position_keep_32[one_idx]; //Add the bit to the count.
        end
        else begin
            sifted_key_number_32 = 0;
        end
    end


    // sifted_key_index_32
    always @(*) begin
        if (deletion_en) begin
            sifted_key_index_32[31] = 5'b0;

            for (i=30; i>=0; i=i-1) begin
                sifted_key_index_32[i] = 0;
                for (j=31; j>i; j=j-1) begin
                    sifted_key_index_32[i] = sifted_key_index_32[i] + position_keep_32[j];
                end
            end
        end
        else begin
            for (i=0; i<`QUBIT_WIDTH; i=i+1) begin
                sifted_key_index_32[i] = 5'b0;
            end
        end
    end

    // sifted_key_offset_128
    always@(posedge clk ) begin
        if (~rst_n) begin
            sifted_key_offset_128 <= 7'b0;
        end
        else begin
            if (deletion_en) begin
                sifted_key_offset_128 <= (sifted_key_offset_128 + sifted_key_number_32);
            end
            else begin
                sifted_key_offset_128 <= sifted_key_offset_128;
            end
        end
    end

    // A sifted key 128 valid
    always @(posedge clk ) begin
        if (~rst_n) begin
            sifted_key_128_valid <= 2'b00;
        end
        else begin
            if ((((sifted_key_offset_128 + sifted_key_number_32)>=64) && (sifted_key_offset_128<64))) begin
                sifted_key_128_valid <= 2'b10;
            end
            else if ((sifted_key_offset_128 + sifted_key_number_32)>=128) begin
                sifted_key_128_valid <= 2'b01;
            end
            else begin
                sifted_key_128_valid <= 2'b00;
            end
        end
    end


    // A sifted key 128
    always @(posedge clk ) begin
        if (~rst_n) begin
            sifted_key_128 <= 128'b0;
        end
        else begin

            for (i=31; i>=0; i=i-1) begin
                case ({(position_keep_32[i]), (pulse_64[ (2*i+1)-: 2])})
                    3'b110:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= 1'b0;
                    end
                    3'b101:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= 1'b1;
                    end
                    default:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128];
                    end
                endcase
            end
        end
    end


    // A sifted key 64 output
    //output reg sifted_key_64_valid,
    //output reg [`SIFTEDKEY_WIDTH-1:0] sifted_key_64
    always @(posedge clk ) begin
        if (~rst_n) begin
            sifted_key_64_valid <= 1'b0;
            sifted_key_64 <= `SIFTEDKEY_WIDTH'b0;
        end
        else begin
            case({sifted_key_128_valid})
                2'b10:begin
                    sifted_key_64 <= sifted_key_128[127:64];
                    sifted_key_64_valid <= 1'b1;
                end
                2'b01:begin
                    sifted_key_64 <= sifted_key_128[63:0];
                    sifted_key_64_valid <= 1'b1;
                end
                default: begin
                    sifted_key_64 <= 64'b0;
                    sifted_key_64_valid <= 1'b0;
                end
            endcase
        end
    end

endmodule







module send_decoy_fsm (
    input clk,
    input rst_n,

    input sift_decoy_nodetected_en,
    input A_TX_decoy_empty,
    input decoy_round_count_finish,
    input decoy_last_round,

    output reg decoy_round_count_en,
    output reg reset_decoy_cnt,
    output reg send_decoy_finish,

    output wire reset_decoy_round_cnt,

    output wire [19:0] decoy_round_addr_offset,
    output reg [7:0] decoy_round,
    output reg [3:0] decoy_state
);

    localparam DECOY_IDLE              = 4'd0;
    localparam DECOY_START             = 4'd1;
    localparam ROUND_IDLE               = 4'd2;
    localparam ROUND_COUNT              = 4'd3;
    localparam ROUND_COUNT_END          = 4'd4;
    localparam RESET_DECOY_CNT         = 4'd5;
    localparam DECOY_END               = 4'd6;


    assign reset_decoy_round_cnt = ((decoy_state==ROUND_IDLE)&&(next_decoy_state==ROUND_COUNT))?
                                1'b1:1'b0;

    always @(posedge clk ) begin
        if (~rst_n) begin
            decoy_round <= 8'b0;
        end
        else if (reset_decoy_cnt) begin
            decoy_round <= 8'b0;
        end
        else if (decoy_state==ROUND_COUNT_END) begin
            decoy_round <= decoy_round + 1;
        end
        else begin
            decoy_round <= decoy_round;
        end
    end


    assign decoy_round_addr_offset = (decoy_round<<10);


    reg [3:0] next_decoy_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            decoy_state <= DECOY_IDLE;
        end
        else begin
            decoy_state <= next_decoy_state;
        end
    end


    always @(*) begin
        case (decoy_state)
            DECOY_IDLE: begin
                if (sift_decoy_nodetected_en) begin
                    next_decoy_state = DECOY_START;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
                else begin
                    next_decoy_state = DECOY_IDLE;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
            end


            DECOY_START: begin
                next_decoy_state = ROUND_IDLE;
                decoy_round_count_en = 1'b0;
                reset_decoy_cnt = 1'b0;
                send_decoy_finish = 1'b0;
            end

            ROUND_IDLE: begin
                if (A_TX_decoy_empty) begin
                    next_decoy_state = ROUND_COUNT;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
                else begin
                    next_decoy_state = ROUND_IDLE;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
            end

            ROUND_COUNT: begin
                if (decoy_round_count_finish) begin
                    next_decoy_state = ROUND_COUNT_END;
                    decoy_round_count_en = 1'b1;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
                else begin
                    next_decoy_state = ROUND_COUNT;
                    decoy_round_count_en = 1'b1;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
            end


            ROUND_COUNT_END: begin
                if (decoy_last_round) begin
                    next_decoy_state = RESET_DECOY_CNT;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
                else begin
                    next_decoy_state = ROUND_IDLE;
                    decoy_round_count_en = 1'b0;
                    reset_decoy_cnt = 1'b0;
                    send_decoy_finish = 1'b0;
                end
            end


            RESET_DECOY_CNT: begin
                next_decoy_state = DECOY_END;
                decoy_round_count_en = 1'b0;
                reset_decoy_cnt = 1'b1;
                send_decoy_finish = 1'b0;
            end

            DECOY_END: begin
                next_decoy_state = DECOY_IDLE;
                decoy_round_count_en = 1'b0;
                reset_decoy_cnt = 1'b0;
                send_decoy_finish = 1'b1;
            end

            default: begin
                next_decoy_state = DECOY_IDLE;
                decoy_round_count_en = 1'b0;
                reset_decoy_cnt = 1'b0;
                send_decoy_finish = 1'b0;
            end
        endcase
    end

endmodule







module A_sifting_fsm (
    input clk,
    input rst_n,

    input start_A_sifting,
    input start_A_TX,
    input A_RX_Xbasis_detected_full,
    input A_RX_Zbasis_detected_full,
    input sift_decoy_nodetected_finish,
    
    output reg wait_A_TX,
    output reg sift_decoy_nodetected_en,
    // output reg A_visibility_valid,
    output reg reset_sift_parameter,
    output reg A_sifting_finish,
    output reg [3:0] A_sift_state
);

    localparam SIFT_IDLE                    = 4'd0;
    localparam SIFT_START                   = 4'd1;
    localparam WAIT_ZBASIS_XBASIS           = 4'd2;
    localparam ZBASIS_XBASIS_FULL           = 4'd3;
    localparam SIFT_DEOCY_NODETECTED        = 4'd4;
    localparam SIFT_DEOCY_NODETECTED_END    = 4'd5;
    localparam VISIBILITY_PARAMETER_VALID   = 4'd6;
    localparam RESET_SIFT_PARAMETER         = 4'd7;
    localparam SIFT_END                     = 4'd8;


    reg visibility_en;

    reg [3:0] next_A_sift_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            A_sift_state <= SIFT_IDLE;
        end
        else begin
            A_sift_state <= next_A_sift_state;
        end
    end

    always @(*) begin
        case (A_sift_state)
            SIFT_IDLE: begin
                if (start_A_sifting) begin
                    next_A_sift_state = SIFT_START;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
                else begin
                    next_A_sift_state = SIFT_IDLE;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
            end


            SIFT_START: begin
                next_A_sift_state = WAIT_ZBASIS_XBASIS;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b0;
                reset_sift_parameter = 1'b0;
                A_sifting_finish = 1'b0;
            end


            WAIT_ZBASIS_XBASIS: begin
                if (A_RX_Xbasis_detected_full & A_RX_Zbasis_detected_full) begin
                    next_A_sift_state = ZBASIS_XBASIS_FULL;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
                else begin
                    next_A_sift_state = WAIT_ZBASIS_XBASIS;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
            end

            ZBASIS_XBASIS_FULL: begin
                if (start_A_TX) begin
                    next_A_sift_state = SIFT_DEOCY_NODETECTED;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                    wait_A_TX = 1'b0;
                end else begin
                    next_A_sift_state = ZBASIS_XBASIS_FULL;
                    sift_decoy_nodetected_en = 1'b0;
                    visibility_en = 1'b0;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                    wait_A_TX = 1'b1;
                end
            end


            SIFT_DEOCY_NODETECTED: begin
                if (sift_decoy_nodetected_finish) begin
                    next_A_sift_state = SIFT_DEOCY_NODETECTED_END;
                    sift_decoy_nodetected_en = 1'b1;
                    visibility_en = 1'b1;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
                else begin
                    next_A_sift_state = SIFT_DEOCY_NODETECTED;
                    sift_decoy_nodetected_en = 1'b1;
                    visibility_en = 1'b1;
                    //A_visibility_valid = 1'b0;
                    reset_sift_parameter = 1'b0;
                    A_sifting_finish = 1'b0;
                end
            end

            SIFT_DEOCY_NODETECTED_END: begin
                next_A_sift_state = VISIBILITY_PARAMETER_VALID;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b0;
                reset_sift_parameter = 1'b0;
                A_sifting_finish = 1'b0;
            end


            VISIBILITY_PARAMETER_VALID: begin
                next_A_sift_state = RESET_SIFT_PARAMETER;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b1;
                reset_sift_parameter = 1'b0;
                A_sifting_finish = 1'b0;
            end


            RESET_SIFT_PARAMETER: begin
                next_A_sift_state = SIFT_END;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b0;
                reset_sift_parameter = 1'b1;
                A_sifting_finish = 1'b0;
            end


            SIFT_END: begin
                next_A_sift_state = SIFT_IDLE;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b0;
                reset_sift_parameter = 1'b0;
                A_sifting_finish = 1'b1;
            end

            default: begin
                next_A_sift_state = SIFT_IDLE;
                sift_decoy_nodetected_en = 1'b0;
                visibility_en = 1'b0;
                //A_visibility_valid = 1'b0;
                reset_sift_parameter = 1'b0;
                A_sifting_finish = 1'b0;
                wait_A_TX = 1'b0;
            end
        endcase
    end
    

endmodule