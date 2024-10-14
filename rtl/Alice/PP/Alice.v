

`timescale 1ns/1ps


`include "./sifting_parameter.v"


module Alice (
    input clk,
    //----------------------------new add signal------------------------------
    input clk_100M,
    //    input clk_GMII,
    input gmii_tx_clk,
    input gmii_rx_clk,


    input clk_PP,
    input link_status,

    output A2B_busy_PP2Net_TX,
    output A2B_busy_Net2PP_TX,

    output A2B_busy_PP2Net_RX,
    output A2B_busy_Net2PP_RX,


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

    input start_A_TX, //start A transport
    output wait_A_TX, //indicate A is in wait state

    //    output B_sifting_finish,                //sifting is done
    output [`NVIS_WIDTH-1:0] nvis, //nvis
    output [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1, //A_checkkey_1
    output [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0, //A_checkkey_0
    output [`COMPARE_1_WIDTH-1:0] A_compare_1, //A_compare_1
    output [`COMPARE_0_WIDTH-1:0] A_compare_0, //A_compare_0
    output A_visibility_valid, //visibility parameter is valid
    output A_sifting_finish, //sifting is done

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    //    output wire [63:0] Bsiftedkey_dina,     //Alice sifted key 
    //    output wire [14:0] Bsiftedkey_addra,    //0~32767
    //    output wire Bsiftedkey_clka,
    //    output wire Bsiftedkey_ena,                    //1'b1
    //    output wire Bsiftedkey_wea,              //





    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output wire [63:0] Asiftedkey_dina, //Alice sifted key 
    output wire [14:0] Asiftedkey_addra, //0~32767
    output wire Asiftedkey_clka,
    output wire Asiftedkey_ena, //1'b1
    output wire Asiftedkey_wea //

    ,
    output wire A_sift_state
    //test signal TX
    ,
    output [3:0] network_fsm_TCP_A_TX,
    output [2:0] transfer_fsm_A_TX,
    output [2:0] network_fsm_TX_A_TX,
    output [1:0] network_fsm_RX_A_TX,
    output start_handle_FrameSniffer_A_TX,
    output received_valid_TX_A_TX,
    output need_ack_A_TX,
    output is_handshake_A_TX,
    output transfer_finish_A_TX,
    output transfer_en_A_TX,
    output busy_TX2CentCtrl_A_TX,
    output [10:0] index_frame_FrameGenerator_A_TX,
    output [7:0] frame_data_FrameGenerator_A_TX,
    output [15:0] total_len_TCP_FrameGenerator_A_TX,
    output [63:0] douta_FrameGenerator_A_TX,
    output [7:0] keep_crc32_FrameGenerator_A_TX,
    output crc32_valid_FrameGenerator_A_TX,
    output ack_received_A_TX,

    output [15:0] sizeTX_msg_buf_FrameGenerator_A_TX,
    output [15:0] base_addr_tmp_FrameGenerator_A_TX,
    output ack_received_cdc_after_FrameGenerator_A_TX,
    output [10:0] addr_gmii_FrameSniffer_A_TX,
    output [15:0] tcp_segment_len_FrameSniffer_A_TX,
    output [63:0] packet_in_crc32_FrameSniffer_A_TX,
    output [7:0] keep_crc32_FrameSniffer_A_TX,
    output [31:0] crc32_out_FrameSniffer_A_TX,
    output [31:0] seq_RX_A_TX,
    output [31:0] ack_RX_A_TX,
    output [28-1:0] lost_A_TX,
    output [31:0] FCS_received_FrameSniffer_A_TX,
    output packet_valid_FrameSniffer_A_TX,
    output [19:0] tcp_chksum_FrameSniffer_A_TX,
    output [19:0] network_chksum_FrameSniffer_A_TX,
    output msg_accessed_en_FrameSniffer_A_TX
);
    //****************************** start compute ******************************
    wire start_sifting;
    reg [15:0] start_switch_cnt;

    always @(posedge clk) begin
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

    assign start_sifting = (start_switch_cnt==16'b0000_0000_1000_0000)? 1'b1:1'b0;
    //****************************** start compute ******************************









    //****************************** init A qubit instantiation ******************************
    wire [63:0] Qubit_doutb; //qubit from AXI manager
    wire [14:0] Qubit_addrb; //0~32767
    wire Qubit_clkb;
    wire Qubit_enb; //1'b1
    wire Qubit_rstb; //1'b0
    wire [7:0] Qubit_web; //8 bit write enable , 8'b0

    A_qubit_bram qubit_bram (
        .clka(), // input wire clka
        .ena(1'b0), // input wire ena
        .wea(1'b0), // input wire [0 : 0] wea
        .addra(), // input wire [14 : 0] addra
        .dina(), // input wire [63 : 0] dina
        .douta(), // output wire [63 : 0] douta

        .clkb(Qubit_clkb), // input wire clkb
        .enb(Qubit_enb), // input wire enb
        .web((|Qubit_web)), // input wire [0 : 0] web
        .addrb(Qubit_addrb), // input wire [14 : 0] addrb
        .dinb(), // input wire [63 : 0] dinb
        .doutb(Qubit_doutb) // output wire [63 : 0] doutb
    );
    //****************************** init A qubit instantiation ******************************













    //****************************** Alice RX Xbasis FIFO instantiation ******************************

    wire A_RX_Xbasis_detected_wr_clk;
    wire [63:0] A_RX_Xbasis_detected_wr_din;
    wire A_RX_Xbasis_detected_wr_en;
    wire A_RX_Xbasis_detected_full;
    wire A_RX_Xbasis_detected_wr_ack;

    wire A_RX_Xbasis_detected_rd_clk;
    wire A_RX_Xbasis_detected_rd_en;
    wire [63:0] A_RX_Xbasis_detected_rd_dout;
    wire A_RX_Xbasis_detected_empty;
    wire A_RX_Xbasis_detected_rd_valid;

    wire A_RX_Xbasis_detected_wr_rst_busy;
    wire A_RX_Xbasis_detected_rd_rst_busy;

    A_RX_Xbasis_detected_fifo A_B2A_xbasis_fifo (
        .srst(~rst_n), // input wire srst, active high

        .wr_clk(A_RX_Xbasis_detected_wr_clk), // input wire wr_clk
        .din(A_RX_Xbasis_detected_wr_din), // input wire [63 : 0] din
        .wr_en(A_RX_Xbasis_detected_wr_en), // input wire wr_en

        .rd_clk(A_RX_Xbasis_detected_rd_clk), // input wire rd_clk
        .rd_en(A_RX_Xbasis_detected_rd_en), // input wire rd_en
        .dout(A_RX_Xbasis_detected_rd_dout), // output wire [63 : 0] dout

        .full(A_RX_Xbasis_detected_full), // output wire full
        .wr_ack(A_RX_Xbasis_detected_wr_ack), // output wire wr_ack
        .empty(A_RX_Xbasis_detected_empty), // output wire empty
        .valid(A_RX_Xbasis_detected_rd_valid), // output wire valid
        .wr_rst_busy(A_RX_Xbasis_detected_wr_rst_busy), // output wire wr_rst_busy
        .rd_rst_busy(A_RX_Xbasis_detected_rd_rst_busy) // output wire rd_rst_busy
    );
    //****************************** Alice RX Xbasis FIFO instantiation ******************************
    //****************************** Alice RX Zbasis FIFO instantiation ******************************


    wire A_RX_Zbasis_detected_wr_clk;
    wire [31:0] A_RX_Zbasis_detected_wr_din;
    wire A_RX_Zbasis_detected_wr_en;
    wire A_RX_Zbasis_detected_full;
    wire A_RX_Zbasis_detected_wr_ack;

    wire A_RX_Zbasis_detected_rd_clk;
    wire A_RX_Zbasis_detected_rd_en;
    wire [31:0] A_RX_Zbasis_detected_rd_dout;
    wire A_RX_Zbasis_detected_empty;
    wire A_RX_Zbasis_detected_rd_valid;

    wire A_RX_Zbasis_detected_wr_rst_busy;
    wire A_RX_Zbasis_detected_rd_rst_busy;


    A_RX_Zbasis_detected_fifo A_B2A_zbasis_fifo (
        .srst(~rst_n), // input wire srst, active high

        .wr_clk(A_RX_Zbasis_detected_wr_clk), // input wire wr_clk
        .din(A_RX_Zbasis_detected_wr_din), // input wire [31 : 0] din
        .wr_en(A_RX_Zbasis_detected_wr_en), // input wire wr_en

        .rd_clk(A_RX_Zbasis_detected_rd_clk), // input wire rd_clk
        .rd_en(A_RX_Zbasis_detected_rd_en), // input wire rd_en
        .dout(A_RX_Zbasis_detected_rd_dout), // output wire [31 : 0] dout

        .full(A_RX_Zbasis_detected_full), // output wire full
        .wr_ack(A_RX_Zbasis_detected_wr_ack), // output wire wr_ack
        .empty(A_RX_Zbasis_detected_empty), // output wire empty
        .valid(A_RX_Zbasis_detected_rd_valid), // output wire valid
        .wr_rst_busy(A_RX_Zbasis_detected_wr_rst_busy), // output wire wr_rst_busy
        .rd_rst_busy(A_RX_Zbasis_detected_rd_rst_busy) // output wire rd_rst_busy
    );

    //****************************** Alice RX Zbasis FIFO instantiation ******************************
    //****************************** Alice TX decoy FIFO instantiation ******************************

    wire A_TX_decoy_wr_clk;
    wire [31:0] A_TX_decoy_wr_din;
    wire A_TX_decoy_wr_en;
    wire A_TX_decoy_full;
    wire A_TX_decoy_wr_ack;

    wire A_TX_decoy_rd_clk;
    wire A_TX_decoy_rd_en;
    wire [31:0] A_TX_decoy_rd_dout;
    wire A_TX_decoy_empty;
    wire A_TX_decoy_rd_valid;

    wire A_TX_decoy_wr_rst_busy;
    wire A_TX_decoy_rd_rst_busy;

    A_TX_Zbasis_decoy_fifo A_A2B_decoy_fifo (
        .srst(~rst_n), // input wire srst, active high

        .wr_clk(A_TX_decoy_wr_clk), // input wire wr_clk
        .din(A_TX_decoy_wr_din), // input wire [31 : 0] din
        .wr_en(A_TX_decoy_wr_en), // input wire wr_en

        .rd_clk(A_TX_decoy_rd_clk), // input wire rd_clk
        .rd_en(A_TX_decoy_rd_en), // input wire rd_en
        .dout(A_TX_decoy_rd_dout), // output wire [31 : 0] dout

        .full(A_TX_decoy_full), // output wire full
        .wr_ack(A_TX_decoy_wr_ack), // output wire wr_ack
        .empty(A_TX_decoy_empty), // output wire empty
        .valid(A_TX_decoy_rd_valid), // output wire valid
        .wr_rst_busy(A_TX_decoy_wr_rst_busy), // output wire wr_rst_busy
        .rd_rst_busy(A_TX_decoy_rd_rst_busy) // output wire rd_rst_busy
    );
    //****************************** Alice TX decoy FIFO instantiation ****************************








    //****************************** A sift ******************************

    wire Zbasis_Xbasis_fifo_full;


    wire visibility_rd_clk;
    wire visibility_rd_en;
    wire [119 : 0] visibility_rd_dout;
    wire visibility_rd_empty;
    wire visibility_rd_valid;

    top_A_sifting top_Asift (
        .clk(clk),
        .rst_n(rst_n),
        .start_A_sifting(start_sifting),

        .start_A_TX(start_A_TX),
        .wait_A_TX(wait_A_TX),

        .A_sifting_finish(A_sifting_finish),

        .reset_sift_parameter(reset_sift_parameter),

        .Zbasis_Xbasis_fifo_full(Zbasis_Xbasis_fifo_full_cdc),

        // visibility parameter
        .nvis(nvis),
        .A_checkkey_1(A_checkkey_1),
        .A_checkkey_0(A_checkkey_0),
        .A_compare_1(A_compare_1),
        .A_compare_0(A_compare_0),
        .A_visibility_valid(A_visibility_valid),

        ////          visibility parameter fifo
        //         .visibility_rd_clk(visibility_rd_clk),
        //         .visibility_rd_en(visibility_rd_en),
        //         .visibility_rd_dout(visibility_rd_dout),
        //         .visibility_rd_empty(visibility_rd_empty),
        //         .visibility_rd_valid(visibility_rd_valid),


        .Qubit_doutb(Qubit_doutb),
        .Qubit_addrb(Qubit_addrb),
        .Qubit_clkb(Qubit_clkb),
        .Qubit_enb(Qubit_enb),
        .Qubit_rstb(Qubit_rstb),
        .Qubit_web(Qubit_web),

        .A_RX_Xbasis_detected_rd_clk(A_RX_Xbasis_detected_rd_clk),
        .A_RX_Xbasis_detected_rd_en(A_RX_Xbasis_detected_rd_en),
        .A_RX_Xbasis_detected_rd_dout(A_RX_Xbasis_detected_rd_dout),
        .A_RX_Xbasis_detected_empty(A_RX_Xbasis_detected_empty),
        .A_RX_Xbasis_detected_rd_valid(A_RX_Xbasis_detected_rd_valid),
        .A_RX_Xbasis_detected_full(A_RX_Xbasis_detected_full_cdc),

        .A_RX_Zbasis_detected_rd_clk(A_RX_Zbasis_detected_rd_clk),
        .A_RX_Zbasis_detected_rd_en(A_RX_Zbasis_detected_rd_en),
        .A_RX_Zbasis_detected_rd_dout(A_RX_Zbasis_detected_rd_dout),
        .A_RX_Zbasis_detected_empty(A_RX_Zbasis_detected_empty),
        .A_RX_Zbasis_detected_rd_valid(A_RX_Zbasis_detected_rd_valid),
        .A_RX_Zbasis_detected_full(A_RX_Zbasis_detected_full_cdc),

        .A_TX_decoy_wr_clk(A_TX_decoy_wr_clk),
        .A_TX_decoy_wr_din(A_TX_decoy_wr_din),
        .A_TX_decoy_wr_en(A_TX_decoy_wr_en),
        .A_TX_decoy_full(A_TX_decoy_full),
        .A_TX_decoy_wr_ack(A_TX_decoy_wr_ack),
        .A_TX_decoy_empty(A_TX_decoy_empty_cdc),

        .Asiftedkey_dina(Asiftedkey_dina),
        .Asiftedkey_addra(Asiftedkey_addra),
        .Asiftedkey_clka(Asiftedkey_clka),
        .Asiftedkey_ena(Asiftedkey_ena),
        .Asiftedkey_wea(Asiftedkey_wea),

        .A_sift_state(A_sift_state)
    );


    //****************************** A sift ******************************
    //********************* cdc for Alice control signal **********************
    wire A_RX_Xbasis_detected_full_cdc;
    wire A_RX_Zbasis_detected_full_cdc;
    wire A_TX_decoy_empty_cdc;

    cdc_delay1 u_A_RX_Xbasis_detected_full_delay(
        .clk_src(clkRX_msg),
        .clk_des(clk),
        .reset(~rst_n),
        .pulse_src(A_RX_Xbasis_detected_full),
        .pulse_des(A_RX_Xbasis_detected_full_cdc)
    );

    cdc_delay1 u_A_RX_Zbasis_detected_full_delay(
        .clk_src(clkRX_msg),
        .clk_des(clk),
        .reset(~rst_n),
        .pulse_src(A_RX_Zbasis_detected_full),
        .pulse_des(A_RX_Zbasis_detected_full_cdc)
    );

    cdc_delay1 u_A_TX_decoy_empty_delay(
        .clk_src(clkTX_msg),
        .clk_des(clk),
        .reset(~rst_n),
        .pulse_src(A_TX_decoy_empty),
        .pulse_des(A_TX_decoy_empty_cdc)
    );
    //********************* cdc for Alice control signal **********************
    //************************ Jtag for A siftkey bram***********************
   (*mark_debug = "true"*) reg [63:0] Asiftedkey_dina_FF;
    always@(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            Asiftedkey_dina_FF <= 0;
        end else begin
            Asiftedkey_dina_FF <= Asiftedkey_dina;
        end
    end
//    JTAG_wrapper U_JTAG(
//        .bram_addrb_1({14'b0, Asiftedkey_addra, 3'b0}),
//        .bram_clkb_1(Asiftedkey_clka),
//        .bram_dinb_1(Asiftedkey_dina),
//        .bram_enb_1(Asiftedkey_ena),
//        .bram_rstb_1(~rst_n),
//        .bram_web_1({8{Asiftedkey_wea}}),

//        .clk_in_100M(clk_100M),
//        .reset(~rst_n)
//    );
    //************************ Jtag for A siftkey bram***********************
    //****************************** A unpacket  ******************************
    // Input wires and registers
    // wire clk;
    // wire rst_n;
    wire A2B_busy_Net2PP_RX;
    wire A2B_msg_accessed;
    wire [10:0] A2B_sizeRX_msg; // Assuming it's a wire as per your definition

    // Output wires and registers
    wire A2B_busy_PP2Net_RX;
    //    wire Zbasis_Xbasis_fifo_full;
    wire [3:0] A_unpacket_state;

    // FIFO, BRAM, and other connections
    // wire A_RX_Zbasis_detected_wr_clk;
    // wire [31:0] A_RX_Zbasis_detected_wr_din;
    // wire A_RX_Zbasis_detected_wr_en;
    // wire A_RX_Zbasis_detected_full;
    // wire A_RX_Zbasis_detected_wr_ack;

    // wire A_RX_Xbasis_detected_wr_clk;
    // wire [63:0] A_RX_Xbasis_detected_wr_din;
    // wire A_RX_Xbasis_detected_wr_en;
    // wire A_RX_Xbasis_detected_full;
    // wire A_RX_Xbasis_detected_wr_ack;

    wire A_RX_er_wr_clk;
    wire [31:0] A_RX_er_wr_din;
    wire A_RX_er_wr_en;
    wire A_RX_er_full;
    wire A_RX_er_wr_ack;

    wire A_RX_bram_clkb;
    wire A_RX_bram_enb;
    wire A_RX_bram_web;
    wire [10:0] A_RX_bram_addrb; // Declared as a register as per your module definition
    wire [31:0] A_RX_bram_doutb;

    A_unpacket Aunpacket (
        .clk(clkRX_msg), // Clock signal
        //        .clkTX_msg(clkRX_msg_ATX),                       // Clock signal
        .rst_n(rst_n), // Reset signal

        .busy_Net2PP_RX(A2B_busy_Net2PP_RX), // Input indicating the network to post-processing reception is busy
        .msg_accessed(A2B_msg_accessed), // Input indicating message access
        .sizeRX_msg(A2B_sizeRX_msg), // Input for size of RX message

        .busy_PP2Net_RX(A2B_busy_PP2Net_RX), // Output indicating post-processing to network reception is busy

        .reset_sift_parameter(reset_sift_parameter_cdc), // Input to reset sift parameters
        .Zbasis_Xbasis_fifo_full(Zbasis_Xbasis_fifo_full), // Output indicating fifo full status

        .A_unpacket_state(A_unpacket_state), // Output state of the A_unpacket FSM

        // A_B2A Zbasis fifo connections
        .A_RX_Zbasis_detected_wr_clk(A_RX_Zbasis_detected_wr_clk),
        .A_RX_Zbasis_detected_wr_din(A_RX_Zbasis_detected_wr_din),
        .A_RX_Zbasis_detected_wr_en(A_RX_Zbasis_detected_wr_en),
        .A_RX_Zbasis_detected_full(A_RX_Zbasis_detected_full),
        .A_RX_Zbasis_detected_wr_ack(A_RX_Zbasis_detected_wr_ack),

        // A_B2A Xbasis fifo connections
        .A_RX_Xbasis_detected_wr_clk(A_RX_Xbasis_detected_wr_clk),
        .A_RX_Xbasis_detected_wr_din(A_RX_Xbasis_detected_wr_din),
        .A_RX_Xbasis_detected_wr_en(A_RX_Xbasis_detected_wr_en),
        .A_RX_Xbasis_detected_full(A_RX_Xbasis_detected_full),
        .A_RX_Xbasis_detected_wr_ack(A_RX_Xbasis_detected_wr_ack),

        // A_B2A er fifo connections
        .A_RX_er_wr_clk(A_RX_er_wr_clk),
        .A_RX_er_wr_din(A_RX_er_wr_din),
        .A_RX_er_wr_en(A_RX_er_wr_en),
        .A_RX_er_full(A_RX_er_full),
        .A_RX_er_wr_ack(A_RX_er_wr_ack),

        // RX BRAM connections
        .A_RX_bram_clkb(A_RX_bram_clkb),
        .A_RX_bram_enb(A_RX_bram_enb),
        .A_RX_bram_web(A_RX_bram_web),
        .A_RX_bram_addrb(A_RX_bram_addrb),
        .A_RX_bram_doutb(A_RX_bram_doutb)

    );

    //****************************** A unpacket  ******************************
    //***********************  CDC for sifting and unpacket *********************** 
    wire reset_sift_parameter_cdc;
    wire Zbasis_Xbasis_fifo_full_cdc;
    clock_domain_crossing u_reset_sift_parameter_pulse(
        .clk_src(clk),
        .clk_des(clkRX_msg),
        .reset(~rst_n),
        .pulse_src(reset_sift_parameter),
        .pulse_des(reset_sift_parameter_cdc)
    );

    cdc_delay1 u_Zbasis_Xbasis_fifo_full_delay(
        .clk_src(clkRX_msg),
        .clk_des(clk),
        .reset(~rst_n),
        .pulse_src(Zbasis_Xbasis_fifo_full),
        .pulse_des(Zbasis_Xbasis_fifo_full_cdc)
    );

    //***********************  CDC for sifting and unpacket *********************** 

    //****************************** A packet  ******************************
    // Input 
    // wire clk;
    // wire rst_n;
    wire A2B_busy_Net2PP_TX;


    // Output 
    wire A2B_busy_PP2Net_TX;
    wire A2B_msg_stored;
    //    wire [10:0] A2B_sizeTX_msg; // Assuming this should be a register based on your module definition

    wire [3:0] A_packet_state;

    // // A_A2B decoy fifo connections
    // wire A_TX_decoy_rd_clk;
    // wire A_TX_decoy_rd_en;
    // wire [31:0] A_TX_decoy_rd_dout;
    // wire A_TX_decoy_empty;
    // wire A_TX_decoy_rd_valid;

    // A_A2B er fifo connections
    wire A_TX_er_rd_clk;
    wire A_TX_er_rd_en;
    wire [31:0] A_TX_er_rd_dout;
    wire A_TX_er_empty;
    wire A_TX_er_rd_valid;

    // A_A2B pa fifo connections
    wire A_TX_pa_rd_clk;
    wire A_TX_pa_rd_en;
    wire [31:0] A_TX_pa_rd_dout;
    wire A_TX_pa_empty;
    wire A_TX_pa_rd_valid;

    // TX BRAM connections
    wire A_TX_bram_clkb;
    wire A_TX_bram_enb;
    wire A_TX_bram_web;
    wire [10:0] A_TX_bram_addrb;
    wire [31:0] A_TX_bram_dinb;

    A_packet Apacket (
        .clk(clkTX_msg), // Clock signal
        //        .clkRX_msg(clkRX_msg_ATX),            // Clock signal
        .rst_n(rst_n), // Reset signal

        .busy_Net2PP_TX(A2B_busy_Net2PP_TX), // Input indicating the network to post-processing transmission is busy
        .gmii_rx_dv(gmii_rx_dv),
        
        .busy_PP2Net_TX(A2B_busy_PP2Net_TX), // Output indicating post-processing to network transmission is busy
        .msg_stored(A2B_msg_stored), // Output indicating message is stored
        .sizeTX_msg(A2B_sizeTX_msg), // Output register for message size

        .A_packet_state(A_packet_state), // Output state of the A_packet FSM

        // A_A2B decoy fifo connections
        .A_TX_decoy_rd_clk(A_TX_decoy_rd_clk),
        .A_TX_decoy_rd_en(A_TX_decoy_rd_en),
        .A_TX_decoy_rd_dout(A_TX_decoy_rd_dout),
        .A_TX_decoy_empty(A_TX_decoy_empty),
        .A_TX_decoy_rd_valid(A_TX_decoy_rd_valid),

        // A_A2B er fifo connections
        .A_TX_er_rd_clk(A_TX_er_rd_clk),
        .A_TX_er_rd_en(A_TX_er_rd_en),
        .A_TX_er_rd_dout(A_TX_er_rd_dout),
        .A_TX_er_empty(A_TX_er_empty),
        .A_TX_er_rd_valid(A_TX_er_rd_valid),

        // A_A2B pa fifo connections
        .A_TX_pa_rd_clk(A_TX_pa_rd_clk),
        .A_TX_pa_rd_en(A_TX_pa_rd_en),
        .A_TX_pa_rd_dout(A_TX_pa_rd_dout),
        .A_TX_pa_empty(A_TX_pa_empty),
        .A_TX_pa_rd_valid(A_TX_pa_rd_valid),

        // TX BRAM connections
        .A_TX_bram_clkb(A_TX_bram_clkb),
        .A_TX_bram_enb(A_TX_bram_enb),
        .A_TX_bram_web(A_TX_bram_web),
        .A_TX_bram_addrb(A_TX_bram_addrb),
        .A_TX_bram_dinb(A_TX_bram_dinb)

    );

    //****************************** A packet  ******************************


    //****************************** A2B BRAM instantiation ******************************

    A2B_BRAM_TX A2Bbram_TX (
        .clka(clkTX_msg), // input wire clkb
        .ena(1'b1), // input wire enb
        .wea(1'b0), // input wire [0 : 0] web
        .addra(A2B_addrTX_msg), // input wire [10 : 0] addrb
        .dina(), // input wire [31 : 0] dinb
        .douta(A2B_dataTX_msg), // output wire [31 : 0] doutb

        .clkb(A_TX_bram_clkb), // input wire clka
        .enb(A_TX_bram_enb), // input wire ena
        .web(A_TX_bram_web), // input wire [0 : 0] wea
        .addrb(A_TX_bram_addrb), // input wire [10 : 0] addra
        .dinb(A_TX_bram_dinb), // input wire [31 : 0] dina
        .doutb() // output wire [31 : 0] douta
    );

    A2B_BRAM A2Bbram_RX (
        .clka(clkRX_msg), // input wire clka
        .ena(1'b1), // input wire ena
        .wea(A2B_weRX_msg), // input wire [0 : 0] wea
        .addra(A2B_addrRX_msg), // input wire [10 : 0] addra
        .dina(A2B_dataRX_msg), // input wire [31 : 0] dina
        .douta(), // output wire [31 : 0] douta

        .clkb(A_RX_bram_clkb), // input wire clkb
        .enb(A_RX_bram_enb), // input wire enb
        .web(A_RX_bram_web), // input wire [0 : 0] web
        .addrb(A_RX_bram_addrb), // input wire [10 : 0] addrb
        .dinb(), // input wire [31 : 0] dinb
        .doutb(A_RX_bram_doutb) // output wire [31 : 0] doutb
    );
    //****************************** A2B BRAM instantiation ******************************

    //--------------------------------------------TX module of A--------------------------
    wire clkTX_msg;
    wire clkRX_msg;

    wire A2B_busy_PP2Net_TX;                   // BRAMMsgTX is used by PP
    wire A2B_busy_Net2PP_TX;                  // BRAMMsgTX is used by NetworkCentCtrl
    wire A2B_msg_stored;                       // msg is stored in BRAMMsgTX by PP 
    
    wire A2B_busy_PP2Net_RX;                   // BRAMMsgRX is used by PP
    wire A2B_busy_Net2PP_RX;                  // BRAMMsgRX is used by networkCentCtrl
    wire A2B_msg_accessed_en_FrameSniffer;
    wire A2B_msg_accessed;                    // msg is stored in BRAMMsgTX by networkCentCtrl
    
    wire [31:0] A2B_dataTX_msg;                // message from PP 
    wire [10:0] A2B_addrTX_msg;               // addr for BRAMMsgTX
    wire [10:0] A2B_sizeTX_msg;                // transmitting message size
    
    wire [31:0] A2B_dataRX_msg;               // message pasrsed from Ethernet frame
    wire           A2B_weRX_msg;                        // write enable for BRAMMsgRX
    wire [10:0] A2B_addrRX_msg;               // addr for BRAMMSGRX
    wire [10:0] A2B_sizeRX_msg;               // receoved message size

    wire  [7:0] gmii_txd; // Transmit data from client MAC.
    wire  gmii_tx_en; // Transmit control signal from client MAC.
    wire  gmii_tx_er; // Transmit control signal from client MAC.

    wire [7:0]   gmii_rxd; // Received Data to client MAC.d
    wire           gmii_rx_dv; // Received control signal to client MAC.
    wire           gmii_rx_er;

    wire [3:0] network_fsm_TCP_A_TX;
    wire [2:0] transfer_fsm_A_TX;
    wire [2:0] network_fsm_TX_A_TX;
    wire [1:0] network_fsm_RX_A_TX;

    wire start_handle_FrameSniffer_A_TX;

    wire received_valid_A_TX;
    wire need_ack_A_TX;
    wire is_handshake_A_TX;
    wire transfer_en_A_TX;
    wire busy_TX2CentCtrl_A_TX;
    wire transfer_finish_A_TX;
    wire [10:0] index_frame_FrameGenerator_A_TX;
    wire [7:0] frame_data_FrameGenerator_A_TX;
    wire [15:0] total_len_TCP_FrameGenerator_A_TX;
    wire [63:0] douta_FrameGenerator_A_TX;
    wire [7:0] keep_crc32_FrameGenerator_A_TX;
    wire crc32_valid_FrameGenerator_A_TX;
    wire ack_received_A_TX;
    wire ack_received_cdc_after_FrameGenerator_A_TX;
    wire [15:0] sizeTX_msg_buf_FrameGenerator_A_TX;
    wire [15:0] base_addr_tmp_FrameGenerator_A_TX;
    wire [10:0] addr_gmii_FrameSniffer_A_TX;
    wire [15:0] tcp_segment_len_FrameSniffer_A_TX;
    wire [63:0] packet_in_crc32_FrameSniffer_A_TX;
    wire [7:0] keep_crc32_FrameSniffer_A_TX;
    wire [31:0] crc32_out_FrameSniffer_A_TX;
    wire [31:0] seq_RX_A_TX;
    wire [31:0] ack_RX_A_TX;
    wire [25:0] lost_A_TX;
    wire [19:0] tcp_chksum_FrameSniffer_A_TX;
    wire [19:0] network_chksum_FrameSniffer_A_TX;
    wire [31:0] FCS_received_FrameSniffer_A_TX;
    wire packet_valid_FrameSniffer_A_TX;
    wire msg_accessed_en_FrameSniffer_A_TX;
    //    wire lost_cnt_en;
    
    //---------------------------------------------TX module of A-------------------------
    networkCentCtrl #(
    .lost_cycle(26'd30),
    .phy_reset_wait(26'd20)
    ) Unetwork_A2B(
        .reset(~rst_n), // system reset
        //        .clock_100M(clk),            // clock for JTAG module 
        .clk_PP(clk_PP),
        .clkTX_msg(clkTX_msg), // clock for accessing BRAMMsgTX
        .clkRX_msg(clkRX_msg), // clock for accessing BRAMMsgRX

        // Post Processing interface
        //------------------------------------
        .busy_PP2Net_TX(A2B_busy_PP2Net_TX), // BRAMMsgTX is used by PP
        .busy_Net2PP_TX(A2B_busy_Net2PP_TX), // BRAMMsgTX is used by NetworkCentCtrl
        .msg_stored(A2B_msg_stored), // msg is stored in BRAMMsgTX by PP 

        .busy_PP2Net_RX(A2B_busy_PP2Net_RX), // BRAMMsgRX is used by PP
        .busy_Net2PP_RX(A2B_busy_Net2PP_RX), // BRAMMsgRX is used by networkCentCtrl
        .msg_accessed(A2B_msg_accessed), // msg is stored in BRAMMsgTX by networkCentCtrl

        .dataTX_msg(A2B_dataTX_msg), // message from PP 
        .addrTX_msg(A2B_addrTX_msg), // addr for BRAMMsgTX
        .sizeTX_msg(A2B_sizeTX_msg), // transmitting message size

        .dataRX_msg(A2B_dataRX_msg), // message pasrsed from Ethernet frame
        .weRX_msg(A2B_weRX_msg), // write enable for BRAMMsgRX
        .addrRX_msg(A2B_addrRX_msg), // addr for BRAMMSGRX
        .sizeRX_msg(A2B_sizeRX_msg), // receoved message size

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
        .network_fsm_TCP(network_fsm_TCP_A_TX),
        .transfer_fsm(transfer_fsm_A_TX),
        .network_fsm_TX(network_fsm_TX_A_TX),
        .network_fsm_RX(network_fsm_RX_A_TX),
        .start_handle_FrameSniffer(start_handle_FrameSniffer_A_TX),
        .received_valid(received_valid_A_TX),
        .need_ack(need_ack_A_TX),
        .is_handshake(is_handshake_A_TX),
        .transfer_finish(transfer_finish_A_TX),
        .transfer_en(transfer_en_A_TX),
        .busy_TX2CentCtrl(busy_TX2CentCtrl_A_TX),
        .index_frame_FrameGenerator(index_frame_FrameGenerator_A_TX),
        .frame_data_FrameGenerator(frame_data_FrameGenerator_A_TX),
        .total_len_TCP_FrameGenerator(total_len_TCP_FrameGenerator_A_TX),
        .douta_FrameGenerator(douta_FrameGenerator_A_TX),
        .keep_crc32_FrameGenerator(keep_crc32_FrameGenerator_A_TX),
        .crc32_valid_FrameGenerator(crc32_valid_FrameGenerator_A_TX),
        .ack_received_cdc_after_FrameGenerator(ack_received_cdc_after_FrameGenerator_A_TX),
        .ack_received(ack_received_A_TX),
        .sizeTX_msg_buf_FrameGenerator(sizeTX_msg_buf_FrameGenerator_A_TX),
        .base_addr_tmp_FrameGenerator(base_addr_tmp_FrameGenerator_A_TX),
        .addr_gmii_FrameSniffer(addr_gmii_FrameSniffer_A_TX),
        .tcp_segment_len_FrameSniffer(tcp_segment_len_FrameSniffer_A_TX),
        .packet_in_crc32_FrameSniffer(packet_in_crc32_FrameSniffer_A_TX),
        .keep_crc32_FrameSniffer(keep_crc32_FrameSniffer_A_TX),
        .crc32_out_FrameSniffer(crc32_out_FrameSniffer_A_TX),
        .seq_RX(seq_RX_A_TX),
        .ack_RX(ack_RX_A_TX),
        .lost(lost_A_TX),
        .FCS_received_FrameSniffer(FCS_received_FrameSniffer_A_TX),
        .packet_valid_FrameSniffer(packet_valid_FrameSniffer_A_TX),
        .tcp_chksum_FrameSniffer(tcp_chksum_FrameSniffer_A_TX),
        .network_chksum_FrameSniffer(network_chksum_FrameSniffer_A_TX),
        .msg_accessed_en_FrameSniffer(msg_accessed_en_FrameSniffer_A_TX)
        //        .lost_cnt_en(lost_cnt_en)
    );
    //--------------------------------TX module of A------------------------------------
endmodule

