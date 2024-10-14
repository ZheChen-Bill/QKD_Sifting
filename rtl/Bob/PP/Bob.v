
`timescale 1ns/1ps


`include "./sifting_parameter.v"


module Bob(
    input clk,
    //----------------------------new add signal------------------------------
    input clk_100M,
    //    input clk_GMII,
    input gmii_tx_clk,
    input gmii_rx_clk,

    input clk_PP,
    input link_status,
    //    input output_next_pb_TX,
    //    input output_next_pb_RX,

    output B2A_busy_Net2PP_TX,
    output B2A_busy_PP2Net_TX,

    output B2A_busy_Net2PP_RX,
    output B2A_busy_PP2Net_RX,
    //    output A2B_busy_Net2PP_RX,
    //    output A2B_busy_PP2Net_RX,

    output [7:0] gmii_txd, // Transmit data from client MAC.
    output gmii_tx_en, // Transmit control signal from client MAC.
    output gmii_tx_er, // Transmit control signal from client MAC.
    input   [7:0] gmii_rxd, // Received Data to client MAC.
    input   gmii_rx_dv, // Received control signal to client MAC.
    input   gmii_rx_er, // Received control signal to client MAC.


    output clkTX_msg,
    output clkRX_msg,
    //----------------------------new add signal------------------------------
    input rst_n,

    input start_switch,

    input start_B_TX, //start A transport
    output wait_B_TX, //indicate A is in wait state
    
    output B_sifting_finish, //sifting is done
    //    output [`NVIS_WIDTH-1:0] nvis,                  //nvis
    //    output [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1,  //A_checkkey_1
    //    output [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0,  //A_checkkey_0
    //    output [`COMPARE_1_WIDTH-1:0] A_compare_1,      //A_compare_1
    //    output [`COMPARE_0_WIDTH-1:0] A_compare_0,      //A_compare_0
    //    output A_visibility_valid,                      //visibility parameter is valid
    //    output A_sifting_finish,                        //sifting is done

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output wire [63:0] Bsiftedkey_dina, //Alice sifted key 
    output wire [14:0] Bsiftedkey_addra, //0~32767
    output wire Bsiftedkey_clka,
    output wire Bsiftedkey_ena, //1'b1
    output wire Bsiftedkey_wea //




    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    //    output wire [63:0] Asiftedkey_dina,     //Alice sifted key 
    //    output wire [14:0] Asiftedkey_addra,    //0~32767
    //    output wire Asiftedkey_clka,
    //    output wire Asiftedkey_ena,                    //1'b1
    //    output wire Asiftedkey_wea                  //

    ,
    output wire B_sift_state
    ,
//    output reg [63:0] Bsiftedkey_dina_FF

    // test signal 
    output [3:0] network_fsm_TCP_B_TX
    //    output [3:0] network_fsm_TCP_B_RX
    //    output [2:0] transfer_fsm,
    //    output [2:0] network_fsm_TX,
    //    output [1:0] network_fsm_RX,
    //    output start_handle_FrameSniffer,
    //    output received_valid,
    //    output need_ack,
    //    output is_handshake,
    //    output transfer_finish,
    //    output transfer_en,
    //    output busy_TX2CentCtrl,
    //    output [10:0] index_frame_FrameGenerator,
    //    output [7:0] frame_data_FrameGenerator,
    //    output [15:0] total_len_TCP_FrameGenerator,
    //    output [63:0] douta_FrameGenerator,
    //    output [7:0] keep_crc32_FrameGenerator,
    //    output crc32_valid_FrameGenerator,
    //    output ack_received,

    //    output [15:0] sizeTX_msg_buf_FrameGenerator,
    //    output [15:0] base_addr_tmp_FrameGenerator,
    //    output ack_received_cdc_after_FrameGenerator, 
    //    output [10:0] addr_gmii_FrameSniffer,
    //    output [15:0] tcp_segment_len_FrameSniffer,
    //    output [63:0] packet_in_crc32_FrameSniffer,
    //    output [7:0] keep_crc32_FrameSniffer,
    //    output [31:0] crc32_out_FrameSniffer,
    //    output [31:0] seq_RX,
    //    output [31:0] ack_RX,
    //    output [28-1:0] lost,
    //    output [31:0] FCS_received_FrameSniffer,
    //    output packet_valid_FrameSniffer,
    //    output [19:0] tcp_chksum_FrameSniffer,
    //    output [19:0] network_chksum_FrameSniffer,
    //    output msg_accessed_en_FrameSniffer
);
    //****************************** start compute ******************************
    wire start_sifting;
    reg [15:0] start_switch_cnt;

    always @(posedge clk ) begin
        if (~rst_n) begin
            start_switch_cnt <= 16'b0;
        end
        else if (start_switch_cnt==16'b0000_0000_1111_1111) begin
            start_switch_cnt <= start_switch_cnt;
        end
        else if (start_switch) begin
            start_switch_cnt <= start_switch_cnt + 1'b1;
        end
        else begin
            start_switch_cnt <= start_switch_cnt;
        end
    end

    assign start_sifting = (start_switch_cnt ==16'b0000_0000_1000_0000)? 1'b1:1'b0;
    //****************************** start compute ******************************











    //****************************** init B Xbasis detected instantiation ******************************
    wire Xbasis_detected_pos_clkb;
    wire Xbasis_detected_pos_enb;
    wire [7:0] Xbasis_detected_pos_web;
    wire [14:0] Xbasis_detected_pos_addrb;
    wire [63:0] Xbasis_detected_pos_doutb;
    wire Xbasis_detected_pos_rstb;


    B_Xbasis_qubit_bram Xbasis_detected_bram (
        .clka(), // input wire clka
        .ena(1'b0), // input wire ena
        .wea(1'b0), // input wire [0 : 0] wea
        .addra(), // input wire [14 : 0] addra
        .dina(), // input wire [63 : 0] dina
        .douta(), // output wire [63 : 0] douta

        .clkb(Xbasis_detected_pos_clkb), // input wire clkb
        .enb(Xbasis_detected_pos_enb), // input wire enb
        .web((|Xbasis_detected_pos_web)), // input wire [0 : 0] web
        .addrb(Xbasis_detected_pos_addrb), // input wire [14 : 0] addrb
        .dinb(), // input wire [63 : 0] dinb
        .doutb(Xbasis_detected_pos_doutb) // output wire [63 : 0] doutb
    );
    //****************************** init B Xbasis detected instantiation ******************************
    //****************************** init B Zbasis detected instantiation ******************************
    wire Zbasis_detected_pos_clkb;
    wire Zbasis_detected_pos_enb;
    wire [7:0] Zbasis_detected_pos_web;
    wire [14:0] Zbasis_detected_pos_addrb;
    wire [63:0] Zbasis_detected_pos_doutb;
    wire Zbasis_detected_pos_rstb;

    B_Zbasis_qubit_bram Zbasis_qubit_bram (
        .clka(), // input wire clka
        .ena(1'b0), // input wire ena
        .wea(1'b0), // input wire [0 : 0] wea
        .addra(), // input wire [14 : 0] addra
        .dina(), // input wire [63 : 0] dina
        .douta(), // output wire [63 : 0] douta

        .clkb(Zbasis_detected_pos_clkb), // input wire clkb
        .enb(Zbasis_detected_pos_enb), // input wire enb
        .web((|Zbasis_detected_pos_web)), // input wire [0 : 0] web
        .addrb(Zbasis_detected_pos_addrb), // input wire [14 : 0] addrb
        .dinb(), // input wire [63 : 0] dinb
        .doutb(Zbasis_detected_pos_doutb) // output wire [63 : 0] doutb
    );
    //****************************** init B Zbasis detected instantiation ******************************
    //****************************** Bob RX decoy FIFO instantiation ******************************

    wire B_RX_Zbasis_decoy_wr_clk;
    wire [31:0] B_RX_Zbasis_decoy_wr_din;
    wire B_RX_Zbasis_decoy_wr_en;
    wire B_RX_Zbasis_decoy_full;
    wire B_RX_Zbasis_decoy_wr_ack;

    wire B_RX_Zbasis_decoy_rd_clk;
    wire B_RX_Zbasis_decoy_rd_en;
    wire [31:0] B_RX_Zbasis_decoy_rd_dout;
    wire B_RX_Zbasis_decoy_empty;
    wire B_RX_Zbasis_decoy_rd_valid;

    wire B_RX_Zbasis_decoy_wr_rst_busy;
    wire B_RX_Zbasis_decoy_rd_rst_busy;

    B_RX_Zbasis_decoy_fifo B_A2B_fifo (
        .srst(~rst_n), // input wire srst, active high

        .wr_clk(B_RX_Zbasis_decoy_wr_clk), // input wire wr_clk
        .din(B_RX_Zbasis_decoy_wr_din), // input wire [31 : 0] din
        .wr_en(B_RX_Zbasis_decoy_wr_en), // input wire wr_en

        .rd_clk(B_RX_Zbasis_decoy_rd_clk), // input wire rd_clk
        .rd_en(B_RX_Zbasis_decoy_rd_en), // input wire rd_en
        .dout(B_RX_Zbasis_decoy_rd_dout), // output wire [31 : 0] dout

        .full(B_RX_Zbasis_decoy_full), // output wire full
        .wr_ack(B_RX_Zbasis_decoy_wr_ack), // output wire wr_ack
        .empty(B_RX_Zbasis_decoy_empty), // output wire empty
        .valid(B_RX_Zbasis_decoy_rd_valid), // output wire valid
        .wr_rst_busy(B_RX_Zbasis_decoy_wr_rst_busy), // output wire wr_rst_busy
        .rd_rst_busy(B_RX_Zbasis_decoy_rd_rst_busy) // output wire rd_rst_busy
    );
    //****************************** Bob RX decoy FIFO instantiation ******************************
    //****************************** Bob TX FIFO instantiation ******************************
    wire B_TX_detected_wr_clk;
    wire [31:0] B_TX_detected_wr_din;
    wire B_TX_detected_wr_en;
    wire B_TX_detected_full;
    wire B_TX_detected_wr_ack;

    wire B_TX_detected_rd_clk;
    wire B_TX_detected_rd_en;
    wire [31:0] B_TX_detected_rd_dout;
    wire B_TX_detected_empty;
    wire B_TX_detected_rd_valid;

    wire B_TX_detected_wr_rst_busy;
    wire B_TX_detected_rd_rst_busy;

    B_TX_detected_fifo B_B2A_fifo (
        .srst(~rst_n), // input wire srst, active high

        .wr_clk(B_TX_detected_wr_clk), // input wire wr_clk
        .din(B_TX_detected_wr_din), // input wire [31 : 0] din
        .wr_en(B_TX_detected_wr_en), // input wire wr_en

        .rd_clk(B_TX_detected_rd_clk), // input wire rd_clk
        .rd_en(B_TX_detected_rd_en), // input wire rd_en
        .dout(B_TX_detected_rd_dout), // output wire [31 : 0] dout

        .full(B_TX_detected_full), // output wire full
        .wr_ack(B_TX_detected_wr_ack), // output wire wr_ack
        .empty(B_TX_detected_empty), // output wire empty
        .valid(B_TX_detected_rd_valid), // output wire valid
        .wr_rst_busy(B_TX_detected_wr_rst_busy), // output wire wr_rst_busy
        .rd_rst_busy(B_TX_detected_rd_rst_busy) // output wire rd_rst_busy
    );
    //****************************** Bob TX FIFO instantiation ******************************





















    //****************************** B sift ******************************
        wire start_B_TX;
        wire wait_B_TX;
    top_B_sifting top_Bsift (
        .clk(clk),
        .rst_n(rst_n),
        .start_B_sifting(start_sifting),
        
        .start_B_TX(start_B_TX),
        .wait_B_TX(wait_B_TX),

        .B_sifting_finish(B_sifting_finish),

        .Xbasis_detected_pos_doutb(Xbasis_detected_pos_doutb),
        .Xbasis_detected_pos_addrb(Xbasis_detected_pos_addrb),
        .Xbasis_detected_pos_clkb(Xbasis_detected_pos_clkb),
        .Xbasis_detected_pos_enb(Xbasis_detected_pos_enb),
        .Xbasis_detected_pos_rstb(Xbasis_detected_pos_rstb),
        .Xbasis_detected_pos_web(Xbasis_detected_pos_web),

        .Zbasis_detected_pos_doutb(Zbasis_detected_pos_doutb),
        .Zbasis_detected_pos_addrb(Zbasis_detected_pos_addrb),
        .Zbasis_detected_pos_clkb(Zbasis_detected_pos_clkb),
        .Zbasis_detected_pos_enb(Zbasis_detected_pos_enb),
        .Zbasis_detected_pos_rstb(Zbasis_detected_pos_rstb),
        .Zbasis_detected_pos_web(Zbasis_detected_pos_web),

        .B_RX_Zbasis_decoy_rd_clk(B_RX_Zbasis_decoy_rd_clk),
        .B_RX_Zbasis_decoy_rd_en(B_RX_Zbasis_decoy_rd_en),
        .B_RX_Zbasis_decoy_rd_dout(B_RX_Zbasis_decoy_rd_dout),
        .B_RX_Zbasis_decoy_empty(B_RX_Zbasis_decoy_empty),
        .B_RX_Zbasis_decoy_rd_valid(B_RX_Zbasis_decoy_rd_valid),

        .B_TX_detected_wr_clk(B_TX_detected_wr_clk),
        .B_TX_detected_wr_din(B_TX_detected_wr_din),
        .B_TX_detected_wr_en(B_TX_detected_wr_en),
        .B_TX_detected_full(B_TX_detected_full),
        .B_TX_detected_wr_ack(B_TX_detected_wr_ack),
        .B_TX_detected_empty(B_TX_detected_empty_cdc),

        .Bsiftedkey_dina(Bsiftedkey_dina),
        .Bsiftedkey_addra(Bsiftedkey_addra),
        .Bsiftedkey_clka(Bsiftedkey_clka),
        .Bsiftedkey_ena(Bsiftedkey_ena),
        .Bsiftedkey_wea(Bsiftedkey_wea),

        .B_sift_state(B_sift_state)
    );
    //****************************** B sift ******************************
    //********************** cdc for Bob control signal **********************
    wire B_TX_detected_empty_cdc;
    cdc_delay1 u_B_TX_detected_empty_delay(
        .clk_src(clkTX_msg),
        .clk_des(clk),
        .reset(~rst_n),
        .pulse_src(B_TX_detected_empty),
        .pulse_des(B_TX_detected_empty_cdc)
    );
    //********************** cdc for Bob control signal *************************
    //************************ Jtag for B siftkey bram**************************
   (*mark_debug = "true"*) reg [63:0] Bsiftedkey_dina_FF;
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            Bsiftedkey_dina_FF <= 0;
        end else begin
            Bsiftedkey_dina_FF <= Bsiftedkey_dina;
        end
    end
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

    //************************ Jtag for B siftkey bram***********************
    //****************************** B packet  ******************************
    // Input wires
    // wire clk;
    // wire rst_n;
    wire B2A_busy_Net2PP_TX;

    // Output wires and registers
    wire B2A_busy_PP2Net_TX;
    wire B2A_msg_stored;
    wire [10:0] B2A_sizeTX_msg; // Assuming this should be a register based on your module definition

    wire [3:0] B_packet_state;

    // // B_B2A detected fifo connections
    // wire B_TX_detected_rd_clk;
    // wire B_TX_detected_rd_en;
    // wire [31:0] B_TX_detected_rd_dout;
    // wire B_TX_detected_empty;
    // wire B_TX_detected_rd_valid;

    // B_B2A er fifo connections
    wire B_TX_er_rd_clk;
    wire B_TX_er_rd_en;
    wire [31:0] B_TX_er_rd_dout;
    wire B_TX_er_empty;
    wire B_TX_er_rd_valid;

    // TX BRAM connections
    wire B_TX_bram_clkb;
    wire B_TX_bram_enb;
    wire B_TX_bram_web;
    wire [10:0] B_TX_bram_addrb;
    wire [31:0] B_TX_bram_dinb;

    B_packet Bpacket (
        .clk(clkTX_msg), // Clock signal
        .rst_n(rst_n), // Reset signal

        .busy_Net2PP_TX(B2A_busy_Net2PP_TX), // Input indicating the network to post-processing transmission is busy
        .gmii_rx_dv(gmii_rx_dv),
        
        .busy_PP2Net_TX(B2A_busy_PP2Net_TX), // Output indicating post-processing to network transmission is busy
        .msg_stored(B2A_msg_stored), // Output indicating message is stored
        .sizeTX_msg(B2A_sizeTX_msg), // Output register for message size

        .B_packet_state(B_packet_state), // Output state of the B_packet FSM

        // B_B2A detected fifo connections
        .B_TX_detected_rd_clk(B_TX_detected_rd_clk),
        .B_TX_detected_rd_en(B_TX_detected_rd_en),
        .B_TX_detected_rd_dout(B_TX_detected_rd_dout),
        .B_TX_detected_empty(B_TX_detected_empty),
        .B_TX_detected_rd_valid(B_TX_detected_rd_valid),

        // B_B2A er fifo connections
        .B_TX_er_rd_clk(B_TX_er_rd_clk),
        .B_TX_er_rd_en(B_TX_er_rd_en),
        .B_TX_er_rd_dout(B_TX_er_rd_dout),
        .B_TX_er_empty(B_TX_er_empty),
        .B_TX_er_rd_valid(B_TX_er_rd_valid),

        // TX BRAM connections
        .B_TX_bram_clkb(B_TX_bram_clkb),
        .B_TX_bram_enb(B_TX_bram_enb),
        .B_TX_bram_web(B_TX_bram_web),
        .B_TX_bram_addrb(B_TX_bram_addrb),
        .B_TX_bram_dinb(B_TX_bram_dinb)

    );
    //****************************** B packet  ******************************
    //****************************** B2A BRAM instantiation ******************************

    B2A_BRAM_TX B2Abram_TX (
        .clka(clkTX_msg), // input wire clkb
        .ena(1'b1), // input wire enb
        .wea(1'b0), // input wire [0 : 0] web
        .addra(B2A_addrTX_msg), // input wire [10 : 0] addrb
        .dina(), // input wire [31 : 0] dinb
        .douta(B2A_dataTX_msg), // output wire [31 : 0] doutb

        .clkb(B_TX_bram_clkb), // input wire clka
        .enb(B_TX_bram_enb), // input wire ena
        .web(B_TX_bram_web), // input wire [0 : 0] wea
        .addrb(B_TX_bram_addrb), // input wire [10 : 0] addra
        .dinb(B_TX_bram_dinb), // input wire [31 : 0] dina
        .doutb() // output wire [31 : 0] douta
    );


    B2A_BRAM B2Abram_RX (
        .clka(clkRX_msg), // input wire clka
        .ena(1'b1), // input wire ena
        .wea(B2A_weRX_msg), // input wire [0 : 0] wea
        .addra(B2A_addrRX_msg), // input wire [10 : 0] addra
        .dina(B2A_dataRX_msg), // input wire [31 : 0] dina
        .douta(), // output wire [31 : 0] douta

        .clkb(B_RX_bram_clkb), // input wire clkb
        .enb(B_RX_bram_enb), // input wire enb
        .web(B_RX_bram_web), // input wire [0 : 0] web
        .addrb(B_RX_bram_addrb), // input wire [10 : 0] addrb
        .dinb(), // input wire [31 : 0] dinb
        .doutb(B_RX_bram_doutb) // output wire [31 : 0] doutb
    );
    //****************************** B2A BRAM instantiation ******************************
    //------------------------------------Network of B-------------------------------
    wire clkTX_msg;
    wire clkRX_msg;

    wire [31:0] B2A_dataTX_msg; // message from PP 
    wire [10:0] B2A_addrTX_msg; // addr for BRAMMsgTX
    //    wire [10:0] B2A_sizeTX_msg;                // transmitting message size

    wire [31:0] B2A_dataRX_msg; // message pasrsed from Ethernet frame
    wire [10:0] B2A_addrRX_msg; // addr for BRAMMSGRX
    wire B2A_weRX_msg; // write enable for BRAMMsgRX
    //    wire [10:0] B2A_sizeRX_msg;               // receoved message size

    wire  [7:0] gmii_txd; // Transmit data from client MAC.
    wire  gmii_tx_en; // Transmit control signal from client MAC.
    wire  gmii_tx_er; // Transmit control signal from client MAC.

    wire [7:0]     gmii_rxd; // Received Data to client MAC.d
    wire           gmii_rx_dv; // Received control signal to client MAC.
    wire           gmii_rx_er;

    // test signal 
    wire [3:0] network_fsm_TCP_B_TX;
    wire [2:0] transfer_fsm_B_TX;
    wire [2:0] network_fsm_TX_B_TX;
    wire [1:0] network_fsm_RX_B_TX;

    wire start_handle_FrameSniffer_B_TX;

    wire received_valid_B_TX;
    wire need_ack_B_TX;
    wire is_handshake_B_TX;
    wire transfer_en_B_TX;
    wire busy_TX2CentCtrl_B_TX;
    wire transfer_finish_B_TX;
    wire [10:0] index_frame_FrameGenerator_B_TX;
    wire [7:0] frame_data_FrameGenerator_B_TX;
    wire [15:0] total_len_TCP_FrameGenerator_B_TX;
    wire [63:0] douta_FrameGenerator_B_TX;
    wire [7:0] keep_crc32_FrameGenerator_B_TX;
    wire crc32_valid_FrameGenerator_B_TX;
    wire ack_received_B_TX;
    wire ack_received_cdc_after_FrameGenerator_B_TX;
    wire [15:0] sizeTX_msg_buf_FrameGenerator_B_TX;
    wire [15:0] base_addr_tmp_FrameGenerator_B_TX;
    wire [10:0] addr_gmii_FrameSniffer_B_TX;
    wire [15:0] tcp_segment_len_FrameSniffer_B_TX;
    wire [63:0] packet_in_crc32_FrameSniffer_B_TX;
    wire [7:0] keep_crc32_FrameSniffer_B_TX;
    wire [31:0] crc32_out_FrameSniffer_B_TX;
    wire [31:0] seq_RX_B_TX;
    wire [31:0] ack_RX_B_TX;
    wire [25:0] lost_B_TX;
    wire [19:0] tcp_chksum_FrameSniffer_B_TX;
    wire [19:0] network_chksum_FrameSniffer_B_TX;
    wire [31:0] FCS_received_FrameSniffer_B_TX;
    wire packet_valid_FrameSniffer_B_TX;
    wire msg_accessed_en_FrameSniffer_B_TX;
    //    wire lost_cnt_en;
    //------------------------------------TX module of B------------------------
    networkCentCtrl_B #(
    .lost_cycle(26'd30),
    .phy_reset_wait(26'd20)
    ) Unetwork_B2A (
        .reset(~rst_n), // system reset
        //        .clock_100M(clk),            // clock for JTAG module 
        .clk_PP(clk_PP),
        .clkTX_msg(clkTX_msg), // clock for accessing BRAMMsgTX
        .clkRX_msg(clkRX_msg), // clock for accessing BRAMMsgRX

        // Post Processing interface
        //------------------------------------
        .busy_PP2Net_TX(B2A_busy_PP2Net_TX), // BRAMMsgTX is used by PP
        .busy_Net2PP_TX(B2A_busy_Net2PP_TX), // BRAMMsgTX is used by NetworkCentCtrl
        .msg_stored(B2A_msg_stored), // msg is stored in BRAMMsgTX by PP 

        .busy_PP2Net_RX(B2A_busy_PP2Net_RX), // BRAMMsgRX is used by PP
        .busy_Net2PP_RX(B2A_busy_Net2PP_RX), // BRAMMsgRX is used by networkCentCtrl
        .msg_accessed(B2A_msg_accessed), // msg is stored in BRAMMsgTX by networkCentCtrl

        .dataTX_msg(B2A_dataTX_msg), // message from PP 
        .addrTX_msg(B2A_addrTX_msg), // addr for BRAMMsgTX
        .sizeTX_msg(B2A_sizeTX_msg), // transmitting message size

        .dataRX_msg(B2A_dataRX_msg), // message pasrsed from Ethernet frame
        .weRX_msg(B2A_weRX_msg), // write enable for BRAMMsgRX
        .addrRX_msg(B2A_addrRX_msg), // addr for BRAMMSGRX
        .sizeRX_msg(B2A_sizeRX_msg), // receoved message size

        // GMII Interface (client MAC <=> PCS)
        //------------------------------------
        .gmii_tx_clk(gmii_tx_clk), // Transmit clock from client MAC.
        .gmii_rx_clk(gmii_rx_clk), // Receive clock to client MAC.
        .link_status(link_status), // Link status: use status_vector[0]

        .gmii_txd(gmii_txd), // Transmit data from client MAC.
        .gmii_tx_en(gmii_tx_en), // Transmit control signal from client MAC.
        .gmii_tx_er(gmii_tx_er), // Transmit control signal from client MAC.

        .gmii_rxd(gmii_rxd), // Received Data to client MAC.
        .gmii_rx_dv(gmii_rx_dv), // Received control signal to client MAC.
        .gmii_rx_er(gmii_rx_er) // Received control signal to client MAC.

        // Test signal 
        ,
        .network_fsm_TCP(network_fsm_TCP_B_TX),
        .transfer_fsm(transfer_fsm_B_TX),
        .network_fsm_TX(network_fsm_TX_B_TX),
        .network_fsm_RX(network_fsm_RX_B_TX),
        .start_handle_FrameSniffer(start_handle_FrameSniffer_B_TX),
        .received_valid(received_valid_B_TX),
        .need_ack(need_ack_B_TX),
        .is_handshake(is_handshake_B_TX),
        .transfer_finish(transfer_finish_B_TX),
        .transfer_en(transfer_en_B_TX),
        .busy_TX2CentCtrl(busy_TX2CentCtrl_B_TX),
        .index_frame_FrameGenerator(index_frame_FrameGenerator_B_TX),
        .frame_data_FrameGenerator(frame_data_FrameGenerator_B_TX),
        .total_len_TCP_FrameGenerator(total_len_TCP_FrameGenerator_B_TX),
        .douta_FrameGenerator(douta_FrameGenerator_B_TX),
        .keep_crc32_FrameGenerator(keep_crc32_FrameGenerator_B_TX),
        .crc32_valid_FrameGenerator(crc32_valid_FrameGenerator_B_TX),
        .ack_received_cdc_after_FrameGenerator(ack_received_cdc_after_FrameGenerator_B_TX),
        .ack_received(ack_received_B_TX),
        .sizeTX_msg_buf_FrameGenerator(sizeTX_msg_buf_FrameGenerator_B_TX),
        .base_addr_tmp_FrameGenerator(base_addr_tmp_FrameGenerator_B_TX),
        .addr_gmii_FrameSniffer(addr_gmii_FrameSniffer_B_TX),
        .tcp_segment_len_FrameSniffer(tcp_segment_len_FrameSniffer_B_TX),
        .packet_in_crc32_FrameSniffer(packet_in_crc32_FrameSniffer_B_TX),
        .keep_crc32_FrameSniffer(keep_crc32_FrameSniffer_B_TX),
        .crc32_out_FrameSniffer(crc32_out_FrameSniffer_B_TX),
        .seq_RX(seq_RX_B_TX),
        .ack_RX(ack_RX_B_TX),
        .lost(lost_B_TX),
        .FCS_received_FrameSniffer(FCS_received_FrameSniffer_B_TX),
        .packet_valid_FrameSniffer(packet_valid_FrameSniffer_B_TX),
        .tcp_chksum_FrameSniffer(tcp_chksum_FrameSniffer_B_TX),
        .network_chksum_FrameSniffer(network_chksum_FrameSniffer_B_TX),
        .msg_accessed_en_FrameSniffer(msg_accessed_en_FrameSniffer_B_TX)
        //        .lost_cnt_en(lost_cnt_en)
    );
    //--------------------------------------------Network of B--------------------------------------------
    //****************************** B unpacket  ******************************

    // Input
    // wire clk;
    // wire rst_n;
    wire B2A_busy_Net2PP_RX;
    wire B2A_msg_accessed;
    //    wire [10:0] A2B_sizeRX_msg;
    wire reset_er_parameter;
    wire reset_pa_parameter;

    // Output
    wire B2A_busy_PP2Net_RX;
    wire EVrandombit_full;
    wire PArandombit_full;
    wire [3:0] B_unpacket_state;

    // FIFO, BRAM, and other connections
    // wire B_RX_Zbasis_decoy_wr_clk;
    // wire [31:0] B_RX_Zbasis_decoy_wr_din;
    // wire B_RX_Zbasis_decoy_wr_en;
    // wire B_RX_Zbasis_decoy_full;
    // wire B_RX_Zbasis_decoy_wr_ack;

    wire B_RX_er_wr_clk;
    wire [31:0] B_RX_er_wr_din;
    wire B_RX_er_wr_en;
    wire B_RX_er_full;
    assign B_RX_er_full = 1'b0;
    wire B_RX_er_wr_ack;

    wire [63:0] B_RX_EVrandombit_dina;
    wire [13:0] B_RX_EVrandombit_addra;
    wire B_RX_EVrandombit_clka;
    wire B_RX_EVrandombit_ena;
    wire B_RX_EVrandombit_wea;

    wire B_RX_secretkey_length_wr_clk;
    wire [31:0] B_RX_secretkey_length_wr_din;
    wire B_RX_secretkey_length_wr_en;
    wire B_RX_secretkey_length_full;
    wire B_RX_secretkey_length_wr_ack;

    wire [63:0] B_RX_PArandombit_dina;
    wire [13:0] B_RX_PArandombit_addra;
    wire B_RX_PArandombit_clka;
    wire B_RX_PArandombit_ena;
    wire B_RX_PArandombit_wea;

    wire B_RX_bram_clkb;
    wire B_RX_bram_enb;
    wire B_RX_bram_web;
    wire [10:0] B_RX_bram_addrb;
    wire [31:0] B_RX_bram_doutb;


    B_unpacket Bunpacket (
        .clk(clkRX_msg), // Clock signal
        //        .clkTX_msg(clkTX_msg_BRX),                           // TX Network Clock signal
        .rst_n(rst_n), // Reset signal

        .busy_Net2PP_RX(B2A_busy_Net2PP_RX), // Input indicating the network to post-processing reception is busy
        .msg_accessed(B2A_msg_accessed), // Input indicating message access
        .sizeRX_msg(B2A_sizeRX_msg), // Input for size of RX message

        .busy_PP2Net_RX(B2A_busy_PP2Net_RX), // Output indicating post-processing to network reception is busy

        .reset_er_parameter(reset_er_parameter), // Input to reset error reconciliation parameter
        .EVrandombit_full(EVrandombit_full), // Output indicating EV random bit buffer is full

        .reset_pa_parameter(B_reset_pa_parameter), // Input to reset post-authentication parameter
        .PArandombit_full(PArandombit_full), // Output indicating PA random bit buffer is full

        .B_unpacket_state(B_unpacket_state), // Output state of the B_unpacket FSM

        // B_A2B decoy fifo connections
        .B_RX_Zbasis_decoy_wr_clk(B_RX_Zbasis_decoy_wr_clk),
        .B_RX_Zbasis_decoy_wr_din(B_RX_Zbasis_decoy_wr_din),
        .B_RX_Zbasis_decoy_wr_en(B_RX_Zbasis_decoy_wr_en),
        .B_RX_Zbasis_decoy_full(B_RX_Zbasis_decoy_full),
        .B_RX_Zbasis_decoy_wr_ack(B_RX_Zbasis_decoy_wr_ack),

        // B_A2B ER fifo connections
        .B_RX_er_wr_clk(B_RX_er_wr_clk),
        .B_RX_er_wr_din(B_RX_er_wr_din),
        .B_RX_er_wr_en(B_RX_er_wr_en),
        .B_RX_er_full(B_RX_er_full),
        .B_RX_er_wr_ack(B_RX_er_wr_ack),

        // B_A2B EV random bit bram connections
        .B_RX_EVrandombit_dina(B_RX_EVrandombit_dina),
        .B_RX_EVrandombit_addra(B_RX_EVrandombit_addra),
        .B_RX_EVrandombit_clka(B_RX_EVrandombit_clka),
        .B_RX_EVrandombit_ena(B_RX_EVrandombit_ena),
        .B_RX_EVrandombit_wea(B_RX_EVrandombit_wea),

        // B_A2B secret key length fifo connections
        .B_RX_secretkey_length_wr_clk(B_RX_secretkey_length_wr_clk),
        .B_RX_secretkey_length_wr_din(B_RX_secretkey_length_wr_din),
        .B_RX_secretkey_length_wr_en(B_RX_secretkey_length_wr_en),
        .B_RX_secretkey_length_full(B_RX_secretkey_length_full),
        .B_RX_secretkey_length_wr_ack(B_RX_secretkey_length_wr_ack),

        // B_A2B PA randombit bram connections
        .B_RX_PArandombit_dina(B_RX_PArandombit_dina),
        .B_RX_PArandombit_addra(B_RX_PArandombit_addra),
        .B_RX_PArandombit_clka(B_RX_PArandombit_clka),
        .B_RX_PArandombit_ena(B_RX_PArandombit_ena),
        .B_RX_PArandombit_wea(B_RX_PArandombit_wea),

        // RX BRAM connections
        .B_RX_bram_clkb(B_RX_bram_clkb),
        .B_RX_bram_enb(B_RX_bram_enb),
        .B_RX_bram_web(B_RX_bram_web),
        .B_RX_bram_addrb(B_RX_bram_addrb),
        .B_RX_bram_doutb(B_RX_bram_doutb)

    );

    //****************************** B unpacket  ******************************
endmodule

