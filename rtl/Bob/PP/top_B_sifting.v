


`timescale 1ns/1ps


`include "../sifting_parameter.v"
`include "packet_parameter.v"




module top_B_sifting (
    input clk,                              //clk
    input rst_n,                          //reset

    input start_B_sifting, //start to sifting
    input start_B_TX, 
//    input start_B_RX, 

    output wait_B_TX,
//    output wait_B_RX,

    output B_sifting_finish, //sifting is done


    // Bob X-basis detected position BRAM (input)
    // width = 64 , depth = 32768
    // port B
    input wire [63:0] Xbasis_detected_pos_doutb,     //X-basis detected pos from AXI manager
    output wire [14:0] Xbasis_detected_pos_addrb,    //0~32767
    output wire Xbasis_detected_pos_clkb,
    output wire Xbasis_detected_pos_enb,                    //1'b1
    output wire Xbasis_detected_pos_rstb,                   //1'b0
    output wire [7:0] Xbasis_detected_pos_web,              //8 bit write enable , 8'b0



    // Bob Z-basis detected qubit BRAM (input)
    // width = 64 , depth = 32768
    // port B
    input wire [63:0] Zbasis_detected_pos_doutb,     //Z-basis detected qubit from AXI manager
    output wire [14:0] Zbasis_detected_pos_addrb,    //0~32767
    output wire Zbasis_detected_pos_clkb,
    output wire Zbasis_detected_pos_enb,                    //1'b1
    output wire Zbasis_detected_pos_rstb,                   //1'b0
    output wire [7:0] Zbasis_detected_pos_web,              //8 bit write enable , 8'b0



    // B_RX_Zbasis_decoy FIFO (input)
    // width = 32 , depth = 2048
    output wire B_RX_Zbasis_decoy_rd_clk,
    output wire B_RX_Zbasis_decoy_rd_en,
    input wire [31:0] B_RX_Zbasis_decoy_rd_dout,
    input wire B_RX_Zbasis_decoy_empty,
    input wire B_RX_Zbasis_decoy_rd_valid,



    // B_TX_detected FIFO (output)
    // width = 32 , depth = 2048
    output wire B_TX_detected_wr_clk,
    output reg [31:0] B_TX_detected_wr_din,
    output reg B_TX_detected_wr_en,
    input wire B_TX_detected_full,
    input wire B_TX_detected_wr_ack,
    input wire B_TX_detected_empty,



    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output reg [63:0] Bsiftedkey_dina,     //Alice sifted key 
    output reg [14:0] Bsiftedkey_addra,    //0~32767
    output wire Bsiftedkey_clka,
    output wire Bsiftedkey_ena,                    //1'b1
    output reg Bsiftedkey_wea,              //
    
    //output sifting state
    output B_sift_state
);
    

//****************************** BRAM setup ******************************
    // Bob X-basis detected position BRAM (input)
    // width = 64 , depth = 32768
    // port B
    assign Xbasis_detected_pos_clkb = clk;
    assign Xbasis_detected_pos_enb = 1'b1;
    assign Xbasis_detected_pos_web = 8'b0;

    // Bob Z-basis detected qubit BRAM (input)
    // width = 64 , depth = 32768
    // port B
    assign Zbasis_detected_pos_clkb = clk;
    assign Zbasis_detected_pos_enb = 1'b1;
    assign Zbasis_detected_pos_web = 8'b0;

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    assign Bsiftedkey_clka = clk;
    assign Bsiftedkey_ena = 1'b1;
//****************************** BRAM setup ******************************
//****************************** FIFO setup ******************************
    // B_RX_Zbasis_decoy FIFO (input)
    // width = 32 , depth = 2048
    assign B_RX_Zbasis_decoy_rd_clk = clk;

    // B_TX_detected FIFO (output)
    // width = 32 , depth = 2048
    assign B_TX_detected_wr_clk = clk;
//****************************** FIFO setup ******************************
//****************************** DFF for bram output ******************************
    reg [63:0] Xbasis_detected_pos_doutb_ff;
    reg [63:0] Zbasis_detected_pos_doutb_ff;

    always @(posedge clk ) begin
        if (~rst_n) begin
            Xbasis_detected_pos_doutb_ff <= 64'b0;
            Zbasis_detected_pos_doutb_ff <= 64'b0;
        end
        else begin
            Xbasis_detected_pos_doutb_ff <= Xbasis_detected_pos_doutb;
            Zbasis_detected_pos_doutb_ff <= Zbasis_detected_pos_doutb;
        end
    end
//****************************** DFF for bram output ******************************



//****************************** B sift fsm ******************************
    //fsm input
    wire send_xbasis_finish;
    wire send_zbasis_finish;
    wire sift_decoy_nodetected_finish;




    //fsm output
    wire send_xbasis_en;
    wire send_zbasis_en;
    wire sift_decoy_nodetected_en;
    wire reset_sift_parameter;

    wire [3:0] B_sift_state;
    wire wait_B_TX;
    B_sifting_fsm Bsift_fsm(
        .clk(clk),
        .rst_n(rst_n),

        .start_B_sifting(start_B_sifting),
        .start_B_TX(start_B_TX),
//        .start_B_RX(start_B_RX),
        
        .wait_B_TX(wait_B_TX),
//        .wait_B_RX(wait_B_RX),
        .send_xbasis_finish(send_xbasis_finish),
        .send_zbasis_finish(send_zbasis_finish),
        .sift_decoy_nodetected_finish(sift_decoy_nodetected_finish),

        .send_xbasis_en(send_xbasis_en),
        .send_zbasis_en(send_zbasis_en),
        .sift_decoy_nodetected_en(sift_decoy_nodetected_en),
        .reset_sift_parameter(reset_sift_parameter),
        .B_sifting_finish(B_sifting_finish),
        .B_sift_state(B_sift_state)
    );
//****************************** B sift fsm ******************************



//****************************** send X-basis pos ******************************
    //wire send_xbasis_en;
    //wire send_xbasis_finish;


    //fsm input 
    wire X_round_count_finish;
    wire last_round;

    //fsm output
    wire round_count_en;
    wire reset_xbasis_cnt;
    wire reset_round_cnt;

    wire [19:0] xbasis_round_addr_offset;
    wire [7:0] xbasis_round;
    wire [3:0] xbasis_state;

    send_xbasis_fsm xbasis_fsm(
        .clk(clk),
        .rst_n(rst_n),

        .send_xbasis_en(send_xbasis_en),
        .B_TX_detected_empty(B_TX_detected_empty),
        .round_count_finish(X_round_count_finish),
        .last_round(last_round),


        .round_count_en(round_count_en),
        .reset_xbasis_cnt(reset_xbasis_cnt),
        .send_xbasis_finish(send_xbasis_finish),
        .reset_round_cnt(reset_round_cnt),

        .xbasis_round_addr_offset(xbasis_round_addr_offset),
        .xbasis_round(xbasis_round),
        .xbasis_state(xbasis_state)
    );

    //output wire [14:0] Xbasis_detected_pos_addrb,    //0~32767
    reg [10:0] xbasis_addr_cnt;
    always @(posedge clk ) begin
        if (~rst_n) begin
            xbasis_addr_cnt <= 11'b0;
        end
        else if (reset_round_cnt) begin
            xbasis_addr_cnt <= 11'b0;
        end
        else if (round_count_en&&(xbasis_addr_cnt<1023)) begin
            xbasis_addr_cnt <= xbasis_addr_cnt + 1;
        end
        else begin
            xbasis_addr_cnt <= xbasis_addr_cnt;
        end
    end

    assign Xbasis_detected_pos_addrb = (xbasis_addr_cnt[10:1] + xbasis_round_addr_offset);

    assign X_round_count_finish = (xbasis_addr_cnt==1023);



    reg [10:0] xbasis_addr_cnt_delay_1, xbasis_addr_cnt_delay_2;
    always @(posedge clk ) begin
        if (~rst_n) begin
            xbasis_addr_cnt_delay_1 <= 1'b0;
            xbasis_addr_cnt_delay_2 <= 1'b0;
        end
        else begin
            xbasis_addr_cnt_delay_1 <= xbasis_addr_cnt;
            xbasis_addr_cnt_delay_2 <= xbasis_addr_cnt_delay_1;
        end
    end

    reg round_count_en_delay;
    reg xbasis_wr_en;
    always @(posedge clk ) begin
        if (~rst_n) begin
            round_count_en_delay <= 1'b0;
            xbasis_wr_en <= 1'b0;
        end
        else begin
            round_count_en_delay <= round_count_en;
            xbasis_wr_en <= round_count_en_delay;
        end
    end

    wire [31:0] xbasis_wr_din;
    assign xbasis_wr_din = ((~xbasis_addr_cnt_delay_2[0]) && xbasis_wr_en)? 
                            Xbasis_detected_pos_doutb_ff[63:32]:Xbasis_detected_pos_doutb_ff[31:0];

    wire [31:0] xbasis_B2A_header;
    assign xbasis_B2A_header = {`B2A_X_BASIS_DETECTED ,
                                `PACKET_LENGTH_1028,
                                16'b0,
                                xbasis_round};
    wire xbasis_B2A_header_write_en;
    assign xbasis_B2A_header_write_en = (xbasis_addr_cnt==1)? 1'b1:1'b0;

    // last round
    assign last_round = (xbasis_round==8'd63)? 1'b1:1'b0;
//****************************** send X-basis pos ******************************






//****************************** send Z-basis pos ******************************
    //wire send_zbasis_en;
    //wire send_zbasis_finish;

    //fsm input
    wire z_round_count_finish;
    wire z_last_round;


    //fsm output
    wire z_round_count_en;
    wire reset_zbasis_cnt;
    wire reset_z_round_cnt;

    wire [19:0] zbasis_round_addr_offset;
    wire [7:0] zbasis_round;
    wire [3:0] zbasis_state;


    send_zbasis_fsm zbasis_fsm(
        .clk(clk),
        .rst_n(rst_n),

        .send_zbasis_en(send_zbasis_en),
        .B_TX_detected_empty(B_TX_detected_empty),
        .z_round_count_finish(z_round_count_finish),
        .z_last_round(z_last_round),


        .z_round_count_en(z_round_count_en),
        .reset_zbasis_cnt(reset_zbasis_cnt),
        .send_zbasis_finish(send_zbasis_finish),
        .reset_z_round_cnt(reset_z_round_cnt),

        .zbasis_round_addr_offset(zbasis_round_addr_offset),
        .zbasis_round(zbasis_round),
        .zbasis_state(zbasis_state)
    );


    //output wire [14:0] Zbasis_detected_pos_addrb,    //0~32767
    reg [10:0] zbasis_addr_cnt;
    always @(posedge clk ) begin
        if (~rst_n) begin
            zbasis_addr_cnt <= 11'b0;
        end
        else if (reset_z_round_cnt) begin
            zbasis_addr_cnt <= 11'b0;
        end
        else if (z_round_count_en&&(zbasis_addr_cnt<1023)) begin
            zbasis_addr_cnt <= zbasis_addr_cnt + 1;
        end
        else begin
            zbasis_addr_cnt <= zbasis_addr_cnt;
        end
    end

    wire [14:0] send_zbasis_pos32_addr;
    assign send_zbasis_pos32_addr = (zbasis_addr_cnt + zbasis_round_addr_offset);

    assign z_round_count_finish = (zbasis_addr_cnt==1023);





    //reg [63:0] Zbasis_detected_pos_doutb_ff;
    reg [31:0] Zbasis_detected_32pos;
    integer idx;

    always @(*) begin
        /*
        Zbasis_detected_32pos[31] = Zbasis_detected_pos_doutb_ff[63] | Zbasis_detected_pos_doutb_ff[62];
        Zbasis_detected_32pos[30] = Zbasis_detected_pos_doutb_ff[61] | Zbasis_detected_pos_doutb_ff[60];
        Zbasis_detected_32pos[29] = Zbasis_detected_pos_doutb_ff[59] | Zbasis_detected_pos_doutb_ff[58];
        Zbasis_detected_32pos[28] = Zbasis_detected_pos_doutb_ff[57] | Zbasis_detected_pos_doutb_ff[56];

        Zbasis_detected_32pos[1] = Zbasis_detected_pos_doutb_ff[3] | Zbasis_detected_pos_doutb_ff[2];
        Zbasis_detected_32pos[0] = Zbasis_detected_pos_doutb_ff[1] | Zbasis_detected_pos_doutb_ff[0];

        */
        for (idx = 31; idx >= 0 ; idx = idx - 1) begin
            Zbasis_detected_32pos[idx] = Zbasis_detected_pos_doutb_ff[(idx<<1)+1] | Zbasis_detected_pos_doutb_ff[(idx<<1)];
        end
    end
    

    wire [31:0] zbasis_B2A_header;
    assign zbasis_B2A_header = {`B2A_Z_BASIS_DETECTED ,
                                `PACKET_LENGTH_1028,
                                16'b0,
                                zbasis_round};
    wire zbasis_B2A_header_write_en;
    assign zbasis_B2A_header_write_en = (zbasis_addr_cnt==1)? 1'b1:1'b0;


    

    reg z_round_count_en_delay;
    reg zbasis_wr_en;
    always @(posedge clk ) begin
        if (~rst_n) begin
            z_round_count_en_delay <= 1'b0;
            zbasis_wr_en <= 1'b0;
        end
        else begin
            z_round_count_en_delay <= z_round_count_en;
            zbasis_wr_en <= z_round_count_en_delay;
        end
    end

    wire [31:0] zbasis_wr_din;
    assign zbasis_wr_din = (zbasis_wr_en)? Zbasis_detected_32pos:32'b0;

    // last round
    assign z_last_round = (zbasis_round==8'd31)? 1'b1:1'b0;

//****************************** send Z-basis pos ******************************




//****************************** write B2A FIFO ******************************



    always @(posedge clk ) begin
        if (~rst_n) begin
            B_TX_detected_wr_din <= 32'b0;
            B_TX_detected_wr_en <= 1'b0;
        end
        else if (send_xbasis_en && xbasis_B2A_header_write_en) begin
            B_TX_detected_wr_din <= xbasis_B2A_header;
            B_TX_detected_wr_en <= 1'b1;
        end
        else if (send_xbasis_en) begin
            B_TX_detected_wr_din <= xbasis_wr_din;
            B_TX_detected_wr_en <= xbasis_wr_en;
        end


        else if (send_zbasis_en && zbasis_B2A_header_write_en) begin
            B_TX_detected_wr_din <= zbasis_B2A_header;
            B_TX_detected_wr_en <= 1'b1;
        end
        else if (send_zbasis_en) begin
            B_TX_detected_wr_din <= zbasis_wr_din;
            B_TX_detected_wr_en <= zbasis_wr_en;
        end
        else begin
            B_TX_detected_wr_din <= 32'b0;
            B_TX_detected_wr_en <= 1'b0;
        end
    end


//****************************** write B2A FIFO ******************************







//****************************** read z-basis pos ******************************
    assign Zbasis_detected_pos_addrb = (sift_decoy_nodetected_en)?
                                        qubit_deletion_addr:send_zbasis_pos32_addr;




    reg [14:0] qubit_deletion_addr;
    always @(posedge clk ) begin
        if (~rst_n) begin
            qubit_deletion_addr <= 15'b0;
        end
        else if (B_RX_Zbasis_decoy_rd_valid) begin
            qubit_deletion_addr <= qubit_deletion_addr + 1;
        end
        else begin
            qubit_deletion_addr <= qubit_deletion_addr;
        end
    end


//****************************** read z-basis pos ******************************




//****************************** qubit_deletion ******************************
    //input 
    reg deletion_en;
    wire [`PULSE_WIDTH-1:0] pulse_64;
    wire [`QUBIT_WIDTH-1:0] B_detected_pos_32;
    wire [`QUBIT_WIDTH-1:0] A_notdecoy_pos_32;

    // output
    wire sifted_key_64_valid;
    wire [`SIFTEDKEY_WIDTH-1:0] sifted_key_64;




    assign pulse_64 = Zbasis_detected_pos_doutb_ff;
    assign B_detected_pos_32 = Zbasis_detected_32pos;
    assign A_notdecoy_pos_32 = B_RX_Zbasis_decoy_rd_dout_delay_2;


    reg B_RX_Zbasis_decoy_rd_valid_delay;
    always @(posedge clk ) begin
        if (~rst_n) begin
            deletion_en <= 1'b0;
            B_RX_Zbasis_decoy_rd_valid_delay <= B_RX_Zbasis_decoy_rd_valid;
        end
        else begin
            deletion_en <= B_RX_Zbasis_decoy_rd_valid_delay;
            B_RX_Zbasis_decoy_rd_valid_delay <= B_RX_Zbasis_decoy_rd_valid;
        end
    end

    reg [31:0] deletion_counter;
    always @(posedge clk ) begin
        if (~rst_n) begin
            deletion_counter <= 32'b0;
        end
        else if (reset_sift_parameter) begin
            deletion_counter <= 32'b0;
        end
        else if (deletion_en) begin
            deletion_counter <= deletion_counter + 1;
        end
        else begin
            deletion_counter <= deletion_counter;
        end
    end


    wire deletion_done;
    assign deletion_done = (deletion_counter==32768);

    reg deletion_done_delay_1;
    always @(posedge clk ) begin
        if (~rst_n) begin
            deletion_done_delay_1 <= 1'b0;
        end
        else begin
            deletion_done_delay_1 <= deletion_done;
        end
    end


    assign sift_decoy_nodetected_finish = deletion_done_delay_1;

    reg [31:0] B_RX_Zbasis_decoy_rd_dout_delay_1, B_RX_Zbasis_decoy_rd_dout_delay_2;
    always @(posedge clk ) begin
        if (~rst_n) begin
            B_RX_Zbasis_decoy_rd_dout_delay_1 <= 32'b0;
            B_RX_Zbasis_decoy_rd_dout_delay_2 <= 32'b0;
        end
        else begin
            B_RX_Zbasis_decoy_rd_dout_delay_1 <= B_RX_Zbasis_decoy_rd_dout;
            B_RX_Zbasis_decoy_rd_dout_delay_2 <= B_RX_Zbasis_decoy_rd_dout_delay_1;
        end
    end

    assign B_RX_Zbasis_decoy_rd_en = B_RX_Zbasis_decoy_rd_valid;

    B_qubit_deletion B_deletion(
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



//****************************** sifted key output ******************************

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    /*
    output reg [63:0] Bsiftedkey_dina,     //Alice sifted key 
    output reg [14:0] Bsiftedkey_addra,    //0~32767
    output wire Bsiftedkey_clka,
    output wire Bsiftedkey_ena,                    //1'b1
    output reg Bsiftedkey_wea              //
    */


    always @(posedge clk ) begin
        if (~rst_n) begin
            Bsiftedkey_addra <= 15'b1111_1111_1111_111;
            Bsiftedkey_dina <= 64'b0;
            Bsiftedkey_wea <= 1'b0;
        end
        else if (sifted_key_64_valid) begin
            Bsiftedkey_addra <= Bsiftedkey_addra + 1;
            Bsiftedkey_dina <= sifted_key_64;
            Bsiftedkey_wea <= 1'b1;
        end
        else begin
            Bsiftedkey_addra <= Bsiftedkey_addra;
            Bsiftedkey_dina <= 64'b0;
            Bsiftedkey_wea <= 1'b0;
        end
    end

//****************************** sifted key output ******************************



endmodule























module B_qubit_deletion(
    input clk,
    input rst_n,

    input deletion_en,
    input [`PULSE_WIDTH-1:0] pulse_64,
    input [`QUBIT_WIDTH-1:0] B_detected_pos_32,
    input [`QUBIT_WIDTH-1:0] A_notdecoy_pos_32,

    
    output reg sifted_key_64_valid,
    output reg [`SIFTEDKEY_WIDTH-1:0] sifted_key_64

);

    //reg [31:0] i;
    //reg [31:0] j;
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
                case ({(position_keep_32[i]),  (pulse_64[ (2*i+1)-: 2]) })
                    3'b110:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= 1'b0;
                    end


                    3'b101:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= 1'b1;
                    end


                    3'b111:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= i[0];
                    end

/*
                    3'b111:begin
                        sifted_key_128[7'b1111111 - sifted_key_index_32[i] - sifted_key_offset_128] <= 1'b1;
                    end
                    */

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















module send_zbasis_fsm (
    input clk,
    input rst_n,

    input send_zbasis_en,
    input B_TX_detected_empty,
    input z_round_count_finish,
    input z_last_round,


    output reg z_round_count_en,
    output reg reset_zbasis_cnt,
    output reg send_zbasis_finish,

    output wire reset_z_round_cnt,

    output wire [19:0] zbasis_round_addr_offset,
    output reg [7:0] zbasis_round,
    output reg [3:0] zbasis_state
);


    localparam ZBASIS_IDLE              = 4'd0;
    localparam ZBASIS_START             = 4'd1;
    localparam ROUND_IDLE               = 4'd2;
    localparam ROUND_COUNT              = 4'd3;
    localparam ROUND_COUNT_END          = 4'd4;
    localparam RESET_ZBASIS_CNT         = 4'd5;
    localparam ZBASIS_END               = 4'd6;

    assign reset_z_round_cnt = ((zbasis_state==ROUND_IDLE)&&(next_zbasis_state==ROUND_COUNT))?
                                1'b1:1'b0;

    always @(posedge clk ) begin
        if (~rst_n) begin
            zbasis_round <= 8'b0;
        end
        else if (reset_zbasis_cnt) begin
            zbasis_round <= 8'b0;
        end
        else if (zbasis_state==ROUND_COUNT_END) begin
            zbasis_round <= zbasis_round + 1;
        end
        else begin
            zbasis_round <= zbasis_round;
        end
    end

    assign zbasis_round_addr_offset = (zbasis_round<<10);


    reg [3:0] next_zbasis_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            zbasis_state <= ZBASIS_IDLE;
        end
        else begin
            zbasis_state <= next_zbasis_state;
        end
    end

    always @(*) begin
        case (zbasis_state)
            ZBASIS_IDLE: begin
                if (send_zbasis_en) begin
                    next_zbasis_state = ZBASIS_START;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
                else begin
                    next_zbasis_state = ZBASIS_IDLE;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
            end


            ZBASIS_START: begin
                next_zbasis_state = ROUND_IDLE;
                z_round_count_en = 1'b0;
                reset_zbasis_cnt = 1'b0;
                send_zbasis_finish = 1'b0;
            end

            ROUND_IDLE: begin
                if (B_TX_detected_empty) begin
                    next_zbasis_state = ROUND_COUNT;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
                else begin
                    next_zbasis_state = ROUND_IDLE;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
            end

            ROUND_COUNT: begin
                if (z_round_count_finish) begin
                    next_zbasis_state = ROUND_COUNT_END;
                    z_round_count_en = 1'b1;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
                else begin
                    next_zbasis_state = ROUND_COUNT;
                    z_round_count_en = 1'b1;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
            end


            ROUND_COUNT_END: begin
                if (z_last_round) begin
                    next_zbasis_state = RESET_ZBASIS_CNT;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
                else begin
                    next_zbasis_state = ROUND_IDLE;
                    z_round_count_en = 1'b0;
                    reset_zbasis_cnt = 1'b0;
                    send_zbasis_finish = 1'b0;
                end
            end


            RESET_ZBASIS_CNT: begin
                next_zbasis_state = ZBASIS_END;
                z_round_count_en = 1'b0;
                reset_zbasis_cnt = 1'b1;
                send_zbasis_finish = 1'b0;
            end

            ZBASIS_END: begin
                next_zbasis_state = ZBASIS_IDLE;
                z_round_count_en = 1'b0;
                reset_zbasis_cnt = 1'b0;
                send_zbasis_finish = 1'b1;
            end

            default: begin
                next_zbasis_state = ZBASIS_IDLE;
                z_round_count_en = 1'b0;
                reset_zbasis_cnt = 1'b0;
                send_zbasis_finish = 1'b0;
            end
        endcase
    end



endmodule
























module send_xbasis_fsm (
    input clk,
    input rst_n,

    input send_xbasis_en,
    input B_TX_detected_empty,
    input round_count_finish,
    input last_round,


    output reg round_count_en,
    output reg reset_xbasis_cnt,
    output reg send_xbasis_finish,

    output wire reset_round_cnt,

    output wire [19:0] xbasis_round_addr_offset,
    output reg [7:0] xbasis_round,
    output reg [3:0] xbasis_state
);


    localparam XBASIS_IDLE              = 4'd0;
    localparam XBASIS_START             = 4'd1;
    localparam ROUND_IDLE               = 4'd2;
    localparam ROUND_COUNT              = 4'd3;
    localparam ROUND_COUNT_END          = 4'd4;
    localparam RESET_XBASIS_CNT         = 4'd5;
    localparam XBASIS_END               = 4'd6;

    assign reset_round_cnt = ((xbasis_state==ROUND_IDLE)&&(next_xbasis_state==ROUND_COUNT))?
                                1'b1:1'b0;

    always @(posedge clk ) begin
        if (~rst_n) begin
            xbasis_round <= 8'b0;
        end
        else if (reset_xbasis_cnt) begin
            xbasis_round <= 8'b0;
        end
        else if (xbasis_state==ROUND_COUNT_END) begin
            xbasis_round <= xbasis_round + 1;
        end
        else begin
            xbasis_round <= xbasis_round;
        end
    end

    assign xbasis_round_addr_offset = (xbasis_round<<9);


    reg [3:0] next_xbasis_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            xbasis_state <= XBASIS_IDLE;
        end
        else begin
            xbasis_state <= next_xbasis_state;
        end
    end

    always @(*) begin
        case (xbasis_state)
            XBASIS_IDLE: begin
                if (send_xbasis_en) begin
                    next_xbasis_state = XBASIS_START;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
                else begin
                    next_xbasis_state = XBASIS_IDLE;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
            end


            XBASIS_START: begin
                next_xbasis_state = ROUND_IDLE;
                round_count_en = 1'b0;
                reset_xbasis_cnt = 1'b0;
                send_xbasis_finish = 1'b0;
            end

            ROUND_IDLE: begin
                if (B_TX_detected_empty) begin
                    next_xbasis_state = ROUND_COUNT;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
                else begin
                    next_xbasis_state = ROUND_IDLE;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
            end

            ROUND_COUNT: begin
                if (round_count_finish) begin
                    next_xbasis_state = ROUND_COUNT_END;
                    round_count_en = 1'b1;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
                else begin
                    next_xbasis_state = ROUND_COUNT;
                    round_count_en = 1'b1;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
            end


            ROUND_COUNT_END: begin
                if (last_round) begin
                    next_xbasis_state = RESET_XBASIS_CNT;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
                else begin
                    next_xbasis_state = ROUND_IDLE;
                    round_count_en = 1'b0;
                    reset_xbasis_cnt = 1'b0;
                    send_xbasis_finish = 1'b0;
                end
            end


            RESET_XBASIS_CNT: begin
                next_xbasis_state = XBASIS_END;
                round_count_en = 1'b0;
                reset_xbasis_cnt = 1'b1;
                send_xbasis_finish = 1'b0;
            end

            XBASIS_END: begin
                next_xbasis_state = XBASIS_IDLE;
                round_count_en = 1'b0;
                reset_xbasis_cnt = 1'b0;
                send_xbasis_finish = 1'b1;
            end

            default: begin
                next_xbasis_state = XBASIS_IDLE;
                round_count_en = 1'b0;
                reset_xbasis_cnt = 1'b0;
                send_xbasis_finish = 1'b0;
            end
        endcase
    end



endmodule
















module B_sifting_fsm (
    input clk,
    input rst_n,

    input start_B_sifting,
    input start_B_TX,
    
    input send_xbasis_finish,
    input send_zbasis_finish,
    input sift_decoy_nodetected_finish,

    output reg wait_B_TX,
    output reg send_xbasis_en,
    output reg send_zbasis_en,
    output reg sift_decoy_nodetected_en,
    output reg reset_sift_parameter,
    output reg B_sifting_finish,
    output reg [3:0] B_sift_state
);


    localparam SIFT_IDLE                    = 4'd0;
    localparam SIFT_START                   = 4'd1;
    localparam SEND_XBASIS                  = 4'd2;
    localparam XBASIS_END                   = 4'd3;
    localparam SEND_ZBASIS                  = 4'd4;
    localparam ZBASIS_END                   = 4'd5;
    localparam SIFT_DEOCY_NODETECTED        = 4'd6;
    localparam SIFT_DEOCY_NODETECTED_END    = 4'd7;
    localparam RESET_SIFT_PARAMETER         = 4'd8;
    localparam SIFT_END                     = 4'd9;


    reg [3:0] next_B_sift_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            B_sift_state <= SIFT_IDLE;
        end
        else begin
            B_sift_state <= next_B_sift_state;
        end
    end
    



    always @(*) begin
        case (B_sift_state)
            SIFT_IDLE: begin
                if (start_B_sifting) begin
                    next_B_sift_state = SIFT_START;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
                else begin
                    next_B_sift_state = SIFT_IDLE;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
            end


            SIFT_START: begin
                if (start_B_TX) begin
                    next_B_sift_state = SEND_XBASIS;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                    wait_B_TX = 1'b0;
                end else begin
                    next_B_sift_state = SIFT_START;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                    wait_B_TX = 1'b1;
                end
            end

            SEND_XBASIS: begin
                if (send_xbasis_finish) begin
                    next_B_sift_state = XBASIS_END;
                    send_xbasis_en = 1'b1;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
                else begin
                    next_B_sift_state = SEND_XBASIS;
                    send_xbasis_en = 1'b1;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
            end

            XBASIS_END: begin
                next_B_sift_state = SEND_ZBASIS;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b0;
                B_sifting_finish = 1'b0;
            end

            SEND_ZBASIS: begin
                if (send_zbasis_finish) begin
                    next_B_sift_state = ZBASIS_END;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b1;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
                else begin
                    next_B_sift_state = SEND_ZBASIS;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b1;
                    sift_decoy_nodetected_en = 1'b0;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
            end

            ZBASIS_END: begin
//            if (start_B_RX) begin
//                next_B_sift_state = SIFT_DEOCY_NODETECTED;
//                send_xbasis_en = 1'b0;
//                send_zbasis_en = 1'b0;
//                sift_decoy_nodetected_en = 1'b0;
//                reset_sift_parameter = 1'b0;
//                B_sifting_finish = 1'b0;
//                wait_B_RX = 1'b0;
//            end else begin
//                next_B_sift_state = SIFT_DEOCY_NODETECTED;
//                send_xbasis_en = 1'b0;
//                send_zbasis_en = 1'b0;
//                sift_decoy_nodetected_en = 1'b0;
//                reset_sift_parameter = 1'b0;
//                B_sifting_finish = 1'b0;
//                wait_B_RX = 1'b1;
//            end
                next_B_sift_state = SIFT_DEOCY_NODETECTED;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b0;
                B_sifting_finish = 1'b0;
            end

            SIFT_DEOCY_NODETECTED: begin
                if (sift_decoy_nodetected_finish) begin
                    next_B_sift_state = SIFT_DEOCY_NODETECTED_END;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b1;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
                else begin
                    next_B_sift_state = SIFT_DEOCY_NODETECTED;
                    send_xbasis_en = 1'b0;
                    send_zbasis_en = 1'b0;
                    sift_decoy_nodetected_en = 1'b1;
                    reset_sift_parameter = 1'b0;
                    B_sifting_finish = 1'b0;
                end
            end

            SIFT_DEOCY_NODETECTED_END: begin
                next_B_sift_state = RESET_SIFT_PARAMETER;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b0;
                B_sifting_finish = 1'b0;
            end


            RESET_SIFT_PARAMETER: begin
                next_B_sift_state = SIFT_END;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b1;
                B_sifting_finish = 1'b0;
            end

            SIFT_END: begin
                next_B_sift_state = SIFT_IDLE;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b0;
                B_sifting_finish = 1'b1;
            end


            default: begin
                next_B_sift_state = SIFT_IDLE;
                send_xbasis_en = 1'b0;
                send_zbasis_en = 1'b0;
                sift_decoy_nodetected_en = 1'b0;
                reset_sift_parameter = 1'b0;
                B_sifting_finish = 1'b0;
                wait_B_TX = 1'b0;
            end
        endcase
    end


endmodule






