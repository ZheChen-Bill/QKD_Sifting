`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/04/18 16:19:54
// Design Name: 
// Module Name: top_Alice
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top_Bob(

    // global reset 
    input reset,

    // system clock -- 300M Hz
    input clk_300M_n,
    input clk_300M_p,

    // GT reference clock for PHY module
    input gtrefclk_p,
    input gtrefclk_n,
    // TX and RX port for connecting SFP
    output txp,
    output txn,
    input rxp,
    input rxn,
    // TX enable signal connecting to SFP module 
    output tx_disable,

    input start_switch,

    input start_TX,

    //    input start_RX,

    output reg output_clk,
    input independent_clk_en,
    input io_refclk_en,
    input gmii_rx_clk_en,
    output [7:0] tcp_status

);

    always @* begin
        output_clk = 1'b0;
        if (independent_clk_en) begin
            output_clk = independent_clk;
        end else if (io_refclk_en) begin
            output_clk = io_refclk;
        end else if (gmii_rx_clk_en) begin
            output_clk = gmii_rx_clk;
        end
    end
    wire independent_clk;
    wire io_refclk;

    wire txn, txp, rxn, rxp;
    wire gmii_rx_clk, gmii_tx_clk;
    wire clock_100M;
    //    wire clock_80M;
    wire clock_125M;

    (*mark_debug = "TRUE"*) wire [7:0]gmii_txd; // Transmit data from client MAC.
    (*mark_debug = "TRUE"*) wire gmii_tx_en; // Transmit control signal from client MAC.
    (*mark_debug = "TRUE"*) wire gmii_tx_er; // Transmit control signal from client MAC.
    (*mark_debug = "TRUE"*) wire [7:0]gmii_rxd; // Received Data to client MAC.
    (*mark_debug = "TRUE"*) wire gmii_rx_dv; // Received control signal to client MAC.
    (*mark_debug = "TRUE"*) wire gmii_rx_er; // Received control signal to client MAC.

//    wire [7:0]gmii_txd; // Transmit data from client MAC.
//    wire gmii_tx_en; // Transmit control signal from client MAC.
//    wire gmii_tx_er; // Transmit control signal from client MAC.
//    wire [7:0]gmii_rxd; // Received Data to client MAC.
//    wire gmii_rx_dv; // Received control signal to client MAC.
//    wire gmii_rx_er; // Received control signal to client MAC.

    wire [15:0] status_vector;

    reg B_sifting_finish_reg;

    //    assign link_status = status_vector[0] & status_vector[1];
    assign link_status = status_vector[0];
    //    assign link_status = 1'b1;
    wire [3:0] network_fsm_TCP_TX;
    assign achieve = (network_fsm_TCP_TX == 4'd3)  ? 1'b1 : 1'b0; // network_fsm_TCP has reached TRANSFER_TCP state
    assign disconnect = (network_fsm_TCP_TX == 4'd0)  ? 1'b1 : 1'b0;
    assign handshake0 = (network_fsm_TCP_TX == 4'd2)  ? 1'b1 : 1'b0;
    assign handshake1 = (network_fsm_TCP_TX == 4'd4)  ? 1'b1 : 1'b0;
    assign handshake = (network_fsm_TCP_TX == 4'd1)  ? 1'b1 : 1'b0;
    assign ack_t = (network_fsm_TCP_TX == 4'd6)  ? 1'b1 : 1'b0;
    assign ack_r = (network_fsm_TCP_TX == 4'd5)  ? 1'b1 : 1'b0;

    //                                          LED7      LED6               LED5          LED4                LED3    LED2        LED1                                 LED0
    //    assign tcp_status = {link_status, handshake0, handshake, handshake1, achieve, ack_t, B2A_busy_Net2PP_RX, B2A_busy_Net2PP_TX};

    //                                          LED7      LED6               LED5                             LED4                       LED3             LED2        LED1                LED0
    assign tcp_status = {link_status, achieve, B2A_busy_Net2PP_RX, B2A_busy_Net2PP_TX, start_switch, start_TX , wait_B_TX,  B_sifting_finish_reg};

    //                                          LED7      LED6               LED5                             LED4                             LED3                LED2              LED1          LED0
    //    assign tcp_status = {link_status, achieve, B2A_busy_Net2PP_RX, B2A_busy_Net2PP_TX, start_switch, sift_state_4, sift_state_5, sift_state_6};

    wire [3:0] B_sift_state;
    //    assign sift_state_4 = (B_sift_state == 4'd4)  ? 1'b1 : 1'b0;
    //    assign sift_state_5 = (B_sift_state == 4'd5)  ? 1'b1 : 1'b0;
    //    assign sift_state_6 = (B_sift_state == 4'd6)  ? 1'b1 : 1'b0;

    assign tx_disable = 1'b1;

    clock_generator Uclk_gen
    (.clk_in1_n(clk_300M_n), // input
        .clk_in1_p(clk_300M_p), // input
        .clk_out1_62_5M(independent_clk), // output
        .clk_out2_100M(clock_100M), // output
        .clk_out3_300M(io_refclk), // output
        .clk_out4_375M(clk_fast), // output
        //        .clk_out5_125M(clock_125M), // output

        .reset(reset) // input
    );

    top_phy Utop_phy //for Bob is TX, for Alice is RX
    (

        .independent_clock(independent_clk),
        .io_refclk(io_refclk),

        // Tranceiver Interface
        //---------------------
        .gtrefclk_p(gtrefclk_p), // Differential +ve of reference clock for MGT: very high quality.
        .gtrefclk_n(gtrefclk_n), // Differential -ve of reference clock for MGT: very high quality.
        .txp(txp), // Differential +ve of serial transmission from PMA to PMD.
        .txn(txn), // Differential -ve of serial transmission from PMA to PMD.
        .rxp(rxp), // Differential +ve for serial reception from PMD to PMA.
        .rxn(rxn), // Differential -ve for serial reception from PMD to PMA.

        // GMII Interface (client MAC <=> PCS)
        //------------------------------------
        .gmii_tx_clk(gmii_tx_clk), // Transmit clock from client MAC.
        .gmii_rx_clk(gmii_rx_clk), // Receive clock to client MAC.
        .gmii_txd(gmii_txd), // Transmit data from client MAC.
        .gmii_tx_en(gmii_tx_en), // Transmit control signal from client MAC.
        .gmii_tx_er(gmii_tx_er), // Transmit control signal from client MAC.
        .gmii_rxd(gmii_rxd), // Received Data to client MAC.
        .gmii_rx_dv(gmii_rx_dv), // Received control signal to client MAC.
        .gmii_rx_er(gmii_rx_er), // Received control signal to client MAC.
        // Management: Alternative to MDIO Interface
        //------------------------------------------

        //    input [4:0]      configuration_vector,  // Alternative to MDIO interface.

        //    .an_interrupt(an_interrupt),          // Interrupt to processor to signal that Auto-Negotiation has completed
        //    input [15:0]     an_adv_config_vector,  // Alternate interface to program REG4 (AN ADV)
        //    input            an_restart_config,     // Alternate signal to modify AN restart bit in REG0


        // General IO's
        //-------------
        .status_vector(status_vector), // Core status.
        .reset(reset) // Asynchronous reset for entire core.
        //    input            signal_detect          // Input from PMD to indicate presence of optical input.
    );

    wire [63:0] Bsiftedkey_dina; //Alice sifted key 
    wire [14:0] Bsiftedkey_addra; //0~32767
    wire Bsiftedkey_clka;
    wire Bsiftedkey_ena; //1'b1
    wire Bsiftedkey_wea; //


    Bob u_Bob(
        .clk(clock_100M),
        //        .clk_100M(clock_100M), // for jtag
        .rst_n(~reset),

        .gmii_tx_clk(gmii_tx_clk),
        .gmii_rx_clk(gmii_rx_clk),

        .clk_PP(clk_fast), // 375MHz
        //        .clk_PP(clock_125M), // 125MHz
        .link_status(link_status),

        .B2A_busy_Net2PP_TX(B2A_busy_Net2PP_TX),
        .B2A_busy_Net2PP_RX(B2A_busy_Net2PP_RX),


        .gmii_txd(gmii_txd), // Transmit data from client MAC.
        .gmii_tx_en(gmii_tx_en), // Transmit control signal from client MAC.
        .gmii_tx_er(gmii_tx_er), // Transmit control signal from client MAC.
        .gmii_rxd(gmii_rxd), // Received Data to client MAC.
        .gmii_rx_dv(gmii_rx_dv), // Received control signal to client MAC.
        .gmii_rx_er(gmii_rx_er), // Received control signal to client MAC.


        //        .output_next_pb_TX(output_next_pb_TX),
        //        .output_next_pb_RX(output_next_pb_RX),

        .clkTX_msg(clkTX_msg),
        .clkRX_msg(clkRX_msg),


        .start_switch(start_switch),
        .start_B_TX(start_TX),
        //        .start_B_RX(start_RX),

        .wait_B_TX(wait_B_TX),
        //        .wait_B_RX(wait_B_RX),

        // Bob sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        .Bsiftedkey_dina(Bsiftedkey_dina), //Alice sifted key 
        .Bsiftedkey_addra(Bsiftedkey_addra), //0~32767
        .Bsiftedkey_clka(Bsiftedkey_clka),
        .Bsiftedkey_ena(Bsiftedkey_ena), //1'b1
        .Bsiftedkey_wea(Bsiftedkey_wea), //


        // Alice sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        //        .Asiftedkey_dina(Asiftedkey_dina),     //Alice sifted key 
        //        .Asiftedkey_addra(Asiftedkey_addra),    //0~32767
        //        .Asiftedkey_clka(Asiftedkey_clka),
        //        .Asiftedkey_ena(Asiftedkey_ena),                    //1'b1
        //        .Asiftedkey_wea(Asiftedkey_wea),              //


        //        .nvis(nvis),
        //        .A_checkkey_1(A_checkkey_1),
        //        .A_checkkey_0(A_checkkey_0),
        //        .A_compare_1(A_compare_1),
        //        .A_compare_0(A_compare_0),
        //        .A_visibility_valid(A_visibility_valid),
        //        .A_sifting_finish(A_sifting_finish),
        .B_sifting_finish(B_sifting_finish),

        .B_sift_state(B_sift_state),
        .network_fsm_TCP_B_TX(network_fsm_TCP_TX)
        //        .transfer_fsm(transfer_fsm),
        //        .network_fsm_TX(network_fsm_TX),
        //        .network_fsm_RX(network_fsm_RX),
        //        .start_handle_FrameSniffer(start_handle_FrameSniffer),
        //        .received_valid(received_valid),
        //        .need_ack(need_ack),
        //        .is_handshake(is_handshake),
        //        .transfer_finish(transfer_finish),
        //        .transfer_en(transfer_en),
        //        .busy_TX2CentCtrl(busy_TX2CentCtrl),
        //        .index_frame_FrameGenerator(index_frame_FrameGenerator),
        //        .frame_data_FrameGenerator(frame_data_FrameGenerator),
        //        .total_len_TCP_FrameGenerator(total_len_TCP_FrameGenerator),
        //        .douta_FrameGenerator(douta_FrameGenerator),
        //        .keep_crc32_FrameGenerator(keep_crc32_FrameGenerator),
        //        .crc32_valid_FrameGenerator(crc32_valid_FrameGenerator),
        //        .ack_received_cdc_after_FrameGenerator(ack_received_cdc_after_FrameGenerator),
        //        .ack_received(ack_received),
        //        .sizeTX_msg_buf_FrameGenerator(sizeTX_msg_buf_FrameGenerator),
        //        .base_addr_tmp_FrameGenerator(base_addr_tmp_FrameGenerator),
        //        .addr_gmii_FrameSniffer(addr_gmii_FrameSniffer),
        //        .tcp_segment_len_FrameSniffer(tcp_segment_len_FrameSniffer),
        //        .packet_in_crc32_FrameSniffer(packet_in_crc32_FrameSniffer),
        //        .keep_crc32_FrameSniffer(keep_crc32_FrameSniffer),
        //        .crc32_out_FrameSniffer(crc32_out_FrameSniffer),
        //        .seq_RX(seq_RX),
        //        .ack_RX(ack_RX),
        //        .lost(lost),
        //        .FCS_received_FrameSniffer(FCS_received_FrameSniffer),
        //        .packet_valid_FrameSniffer(packet_valid_FrameSniffer),
        //        .tcp_chksum_FrameSniffer(tcp_chksum_FrameSniffer),
        //        .network_chksum_FrameSniffer(network_chksum_FrameSniffer),
        //        .msg_accessed_en_FrameSniffer(msg_accessed_en_FrameSniffer),
        //        .lost_cnt_en(lost_cnt_en)
    );

    always@(posedge clock_100M or posedge reset) begin
        if(reset) begin
            B_sifting_finish_reg <= 1'b0;
        end else if (B_sifting_finish) begin
            B_sifting_finish_reg <= 1'b1;
        end else begin
            B_sifting_finish_reg <= B_sifting_finish_reg;
        end
    end
    //    //************************ Jtag for B siftkey bram***********************

    //    JTAG_wrapper U_JTAG(
    //        .bram_addrb_1({14'b0, Bsiftedkey_addra, 3'b0}),
    //        .bram_clkb_1(Bsiftedkey_clka),
    //        .bram_dinb_1(Bsiftedkey_dina),
    //        .bram_enb_1(Bsiftedkey_ena),
    //        .bram_rstb_1(~rst_n),
    //        .bram_web_1({8{Bsiftedkey_wea}}),

    //        .clk_in_100M(clk_100M),
    //        .reset(~rst_n)
    //    );

    //    //************************ Jtag for B siftkey bram***********************
endmodule
