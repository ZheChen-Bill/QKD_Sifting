

`timescale 1ns/1ps


`include "./sifting_parameter.v"



module top_AB_sifting (
    input clk,
    input rst_n,

    input start_switch,


    output B_sifting_finish,                //sifting is done
    output [`NVIS_WIDTH-1:0] nvis,                  //nvis
    output [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1,  //A_checkkey_1
    output [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0,  //A_checkkey_0
    output [`COMPARE_1_WIDTH-1:0] A_compare_1,      //A_compare_1
    output [`COMPARE_0_WIDTH-1:0] A_compare_0,      //A_compare_0
    output A_visibility_valid,                      //visibility parameter is valid
    output A_sifting_finish,                        //sifting is done

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output wire [63:0] Bsiftedkey_dina,     //Alice sifted key 
    output wire [14:0] Bsiftedkey_addra,    //0~32767
    output wire Bsiftedkey_clka,
    output wire Bsiftedkey_ena,                    //1'b1
    output wire Bsiftedkey_wea,              //





    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    output wire [63:0] Asiftedkey_dina,     //Alice sifted key 
    output wire [14:0] Asiftedkey_addra,    //0~32767
    output wire Asiftedkey_clka,
    output wire Asiftedkey_ena,                    //1'b1
    output wire Asiftedkey_wea              //
    
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

assign start_sifting = (start_switch_cnt==16'b0000_0000_1000_0000)? 1'b1:1'b0;
//****************************** start compute ******************************











//****************************** init B Xbasis detected instantiation ******************************
    wire Xbasis_detected_pos_clkb;
    wire Xbasis_detected_pos_enb;
    wire [7:0] Xbasis_detected_pos_web;
    wire [14:0] Xbasis_detected_pos_addrb;
    wire [63:0] Xbasis_detected_pos_doutb;
    wire Xbasis_detected_pos_rstb;


    B_Xbasis_qubit_bram Xbasis_detected_bram (
        .clka(),    // input wire clka
        .ena(1'b0),      // input wire ena
        .wea(1'b0),      // input wire [0 : 0] wea
        .addra(),  // input wire [14 : 0] addra
        .dina(),    // input wire [63 : 0] dina
        .douta(),  // output wire [63 : 0] douta

        .clkb(Xbasis_detected_pos_clkb),    // input wire clkb
        .enb(Xbasis_detected_pos_enb),      // input wire enb
        .web((|Xbasis_detected_pos_web)),      // input wire [0 : 0] web
        .addrb(Xbasis_detected_pos_addrb),  // input wire [14 : 0] addrb
        .dinb(),    // input wire [63 : 0] dinb
        .doutb(Xbasis_detected_pos_doutb)  // output wire [63 : 0] doutb
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
        .clka(),    // input wire clka
        .ena(1'b0),      // input wire ena
        .wea(1'b0),      // input wire [0 : 0] wea
        .addra(),  // input wire [14 : 0] addra
        .dina(),    // input wire [63 : 0] dina
        .douta(),  // output wire [63 : 0] douta

        .clkb(Zbasis_detected_pos_clkb),    // input wire clkb
        .enb(Zbasis_detected_pos_enb),      // input wire enb
        .web((|Zbasis_detected_pos_web)),      // input wire [0 : 0] web
        .addrb(Zbasis_detected_pos_addrb),  // input wire [14 : 0] addrb
        .dinb(),    // input wire [63 : 0] dinb
        .doutb(Zbasis_detected_pos_doutb)  // output wire [63 : 0] doutb
    );
//****************************** init B Zbasis detected instantiation ******************************
//****************************** init A qubit instantiation ******************************
    wire [63:0] Qubit_doutb;     //qubit from AXI manager
    wire [14:0] Qubit_addrb;    //0~32767
    wire Qubit_clkb;
    wire Qubit_enb;                    //1'b1
    wire Qubit_rstb;                   //1'b0
    wire [7:0] Qubit_web;              //8 bit write enable , 8'b0

    A_qubit_bram qubit_bram (
        .clka(),    // input wire clka
        .ena(1'b0),      // input wire ena
        .wea(1'b0),      // input wire [0 : 0] wea
        .addra(),  // input wire [14 : 0] addra
        .dina(),    // input wire [63 : 0] dina
        .douta(),  // output wire [63 : 0] douta

        .clkb(Qubit_clkb),    // input wire clkb
        .enb(Qubit_enb),      // input wire enb
        .web((|Qubit_web)),      // input wire [0 : 0] web
        .addrb(Qubit_addrb),  // input wire [14 : 0] addrb
        .dinb(),    // input wire [63 : 0] dinb
        .doutb(Qubit_doutb)  // output wire [63 : 0] doutb
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
        .srst(~rst_n),                // input wire srst, active high

        .wr_clk(A_RX_Xbasis_detected_wr_clk),            // input wire wr_clk
        .din(A_RX_Xbasis_detected_wr_din),                  // input wire [63 : 0] din
        .wr_en(A_RX_Xbasis_detected_wr_en),              // input wire wr_en

        .rd_clk(A_RX_Xbasis_detected_rd_clk),            // input wire rd_clk
        .rd_en(A_RX_Xbasis_detected_rd_en),              // input wire rd_en
        .dout(A_RX_Xbasis_detected_rd_dout),                // output wire [63 : 0] dout

        .full(A_RX_Xbasis_detected_full),                // output wire full
        .wr_ack(A_RX_Xbasis_detected_wr_ack),            // output wire wr_ack
        .empty(A_RX_Xbasis_detected_empty),              // output wire empty
        .valid(A_RX_Xbasis_detected_rd_valid),              // output wire valid
        .wr_rst_busy(A_RX_Xbasis_detected_wr_rst_busy),  // output wire wr_rst_busy
        .rd_rst_busy(A_RX_Xbasis_detected_rd_rst_busy)  // output wire rd_rst_busy
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
        .srst(~rst_n),                // input wire srst, active high

        .wr_clk(A_RX_Zbasis_detected_wr_clk),            // input wire wr_clk
        .din(A_RX_Zbasis_detected_wr_din),                  // input wire [31 : 0] din
        .wr_en(A_RX_Zbasis_detected_wr_en),              // input wire wr_en

        .rd_clk(A_RX_Zbasis_detected_rd_clk),            // input wire rd_clk
        .rd_en(A_RX_Zbasis_detected_rd_en),              // input wire rd_en
        .dout(A_RX_Zbasis_detected_rd_dout),                // output wire [31 : 0] dout

        .full(A_RX_Zbasis_detected_full),                // output wire full
        .wr_ack(A_RX_Zbasis_detected_wr_ack),            // output wire wr_ack
        .empty(A_RX_Zbasis_detected_empty),              // output wire empty
        .valid(A_RX_Zbasis_detected_rd_valid),              // output wire valid
        .wr_rst_busy(A_RX_Zbasis_detected_wr_rst_busy),  // output wire wr_rst_busy
        .rd_rst_busy(A_RX_Zbasis_detected_rd_rst_busy)  // output wire rd_rst_busy
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
        .srst(~rst_n),                // input wire srst, active high

        .wr_clk(A_TX_decoy_wr_clk),            // input wire wr_clk
        .din(A_TX_decoy_wr_din),                  // input wire [31 : 0] din
        .wr_en(A_TX_decoy_wr_en),              // input wire wr_en

        .rd_clk(A_TX_decoy_rd_clk),            // input wire rd_clk
        .rd_en(A_TX_decoy_rd_en),              // input wire rd_en
        .dout(A_TX_decoy_rd_dout),                // output wire [31 : 0] dout

        .full(A_TX_decoy_full),                // output wire full
        .wr_ack(A_TX_decoy_wr_ack),            // output wire wr_ack
        .empty(A_TX_decoy_empty),              // output wire empty
        .valid(A_TX_decoy_rd_valid),              // output wire valid
        .wr_rst_busy(A_TX_decoy_wr_rst_busy),  // output wire wr_rst_busy
        .rd_rst_busy(A_TX_decoy_rd_rst_busy)  // output wire rd_rst_busy
    );
//****************************** Alice TX decoy FIFO instantiation ******************************







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
        .srst(~rst_n),                // input wire srst, active high

        .wr_clk(B_RX_Zbasis_decoy_wr_clk),            // input wire wr_clk
        .din(B_RX_Zbasis_decoy_wr_din),                  // input wire [31 : 0] din
        .wr_en(B_RX_Zbasis_decoy_wr_en),              // input wire wr_en

        .rd_clk(B_RX_Zbasis_decoy_rd_clk),            // input wire rd_clk
        .rd_en(B_RX_Zbasis_decoy_rd_en),              // input wire rd_en
        .dout(B_RX_Zbasis_decoy_rd_dout),                // output wire [31 : 0] dout

        .full(B_RX_Zbasis_decoy_full),                // output wire full
        .wr_ack(B_RX_Zbasis_decoy_wr_ack),            // output wire wr_ack
        .empty(B_RX_Zbasis_decoy_empty),              // output wire empty
        .valid(B_RX_Zbasis_decoy_rd_valid),              // output wire valid
        .wr_rst_busy(B_RX_Zbasis_decoy_wr_rst_busy),  // output wire wr_rst_busy
        .rd_rst_busy(B_RX_Zbasis_decoy_rd_rst_busy)  // output wire rd_rst_busy
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
        .srst(~rst_n),                // input wire srst, active high

        .wr_clk(B_TX_detected_wr_clk),            // input wire wr_clk
        .din(B_TX_detected_wr_din),                  // input wire [31 : 0] din
        .wr_en(B_TX_detected_wr_en),              // input wire wr_en

        .rd_clk(B_TX_detected_rd_clk),            // input wire rd_clk
        .rd_en(B_TX_detected_rd_en),              // input wire rd_en
        .dout(B_TX_detected_rd_dout),                // output wire [31 : 0] dout

        .full(B_TX_detected_full),                // output wire full
        .wr_ack(B_TX_detected_wr_ack),            // output wire wr_ack
        .empty(B_TX_detected_empty),              // output wire empty
        .valid(B_TX_detected_rd_valid),              // output wire valid
        .wr_rst_busy(B_TX_detected_wr_rst_busy),  // output wire wr_rst_busy
        .rd_rst_busy(B_TX_detected_rd_rst_busy)  // output wire rd_rst_busy
    );
//****************************** Bob TX FIFO instantiation ******************************





















//****************************** B sift ******************************
    top_B_sifting top_Bsift (
        .clk(clk),
        .rst_n(rst_n),
        .start_B_sifting(start_sifting),
        
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
        .B_TX_detected_empty(B_TX_detected_empty),

        .Bsiftedkey_dina(Bsiftedkey_dina),
        .Bsiftedkey_addra(Bsiftedkey_addra),
        .Bsiftedkey_clka(Bsiftedkey_clka),
        .Bsiftedkey_ena(Bsiftedkey_ena),
        .Bsiftedkey_wea(Bsiftedkey_wea)
    );
//****************************** B sift ******************************


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

        .A_sifting_finish(A_sifting_finish),

        .reset_sift_parameter(reset_sift_parameter),

        .Zbasis_Xbasis_fifo_full(Zbasis_Xbasis_fifo_full),

        // visibility parameter
        .nvis(nvis),
        .A_checkkey_1(A_checkkey_1),
        .A_checkkey_0(A_checkkey_0),
        .A_compare_1(A_compare_1),
        .A_compare_0(A_compare_0),
        .A_visibility_valid(A_visibility_valid),

        // // visibility parameter fifo
        // .visibility_rd_clk(visibility_rd_clk),
        // .visibility_rd_en(visibility_rd_en),
        // .visibility_rd_dout(visibility_rd_dout),
        // .visibility_rd_empty(visibility_rd_empty),
        // .visibility_rd_valid(visibility_rd_valid),


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
        .A_RX_Xbasis_detected_full(A_RX_Xbasis_detected_full),

        .A_RX_Zbasis_detected_rd_clk(A_RX_Zbasis_detected_rd_clk),
        .A_RX_Zbasis_detected_rd_en(A_RX_Zbasis_detected_rd_en),
        .A_RX_Zbasis_detected_rd_dout(A_RX_Zbasis_detected_rd_dout),
        .A_RX_Zbasis_detected_empty(A_RX_Zbasis_detected_empty),
        .A_RX_Zbasis_detected_rd_valid(A_RX_Zbasis_detected_rd_valid),
        .A_RX_Zbasis_detected_full(A_RX_Zbasis_detected_full),

        .A_TX_decoy_wr_clk(A_TX_decoy_wr_clk),
        .A_TX_decoy_wr_din(A_TX_decoy_wr_din),
        .A_TX_decoy_wr_en(A_TX_decoy_wr_en),
        .A_TX_decoy_full(A_TX_decoy_full),
        .A_TX_decoy_wr_ack(A_TX_decoy_wr_ack),
        .A_TX_decoy_empty(A_TX_decoy_empty),

        .Asiftedkey_dina(Asiftedkey_dina),
        .Asiftedkey_addra(Asiftedkey_addra),
        .Asiftedkey_clka(Asiftedkey_clka),
        .Asiftedkey_ena(Asiftedkey_ena),
        .Asiftedkey_wea(Asiftedkey_wea)
    );


//****************************** A sift ******************************


































































// //****************************** B2A test instantiation ******************************

//     wire [3:0] B2A_state;
    
//     wire reset_sift_parameter;
//     wire Zbasis_Xbasis_fifo_full;

//     B2A_test B2Atest(

//         .clk(clk),
//         .rst_n(rst_n),

//         .reset_sift_parameter(reset_sift_parameter),

//         .Zbasis_Xbasis_fifo_full(Zbasis_Xbasis_fifo_full),

//         // B_B2A detected fifo
//         .B_TX_detected_rd_clk(B_TX_detected_rd_clk),
//         .B_TX_detected_rd_en(B_TX_detected_rd_en),
//         .B_TX_detected_rd_dout(B_TX_detected_rd_dout),
//         .B_TX_detected_empty(B_TX_detected_empty),
//         .B_TX_detected_rd_valid(B_TX_detected_rd_valid),


//         // A_B2A Zbasis fifo
//         .A_RX_Zbasis_detected_wr_clk(A_RX_Zbasis_detected_wr_clk),
//         .A_RX_Zbasis_detected_wr_din(A_RX_Zbasis_detected_wr_din),
//         .A_RX_Zbasis_detected_wr_en(A_RX_Zbasis_detected_wr_en),
//         .A_RX_Zbasis_detected_full(A_RX_Zbasis_detected_full),
//         .A_RX_Zbasis_detected_wr_ack(A_RX_Zbasis_detected_wr_ack),

//         // A_B2A Xbasis fifo
//         .A_RX_Xbasis_detected_wr_clk(A_RX_Xbasis_detected_wr_clk),
//         .A_RX_Xbasis_detected_wr_din(A_RX_Xbasis_detected_wr_din),
//         .A_RX_Xbasis_detected_wr_en(A_RX_Xbasis_detected_wr_en),
//         .A_RX_Xbasis_detected_full(A_RX_Xbasis_detected_full),
//         .A_RX_Xbasis_detected_wr_ack(A_RX_Xbasis_detected_wr_ack),



//         .B2A_state(B2A_state)
//     );
// //****************************** B2A test instantiation ******************************











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
        .clk(clk),                                // Clock signal
        .rst_n(rst_n),                            // Reset signal

        .busy_Net2PP_TX(B2A_busy_Net2PP_TX),          // Input indicating the network to post-processing transmission is busy

        .busy_PP2Net_TX(B2A_busy_PP2Net_TX),          // Output indicating post-processing to network transmission is busy
        .msg_stored(B2A_msg_stored),                  // Output indicating message is stored
        .sizeTX_msg(B2A_sizeTX_msg),                  // Output register for message size

        .B_packet_state(B_packet_state),          // Output state of the B_packet FSM

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







//****************************** A unpacket  ******************************
    // Input wires and registers
    // wire clk;
    // wire rst_n;
    wire B2A_busy_Net2PP_RX;
    wire B2A_msg_accessed;
    wire [10:0] B2A_sizeRX_msg; // Assuming it's a wire as per your definition

    // Output wires and registers
    wire B2A_busy_PP2Net_RX;
    wire Zbasis_Xbasis_fifo_full;
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
        .clk(clk),                                       // Clock signal
        .rst_n(rst_n),                                   // Reset signal

        .busy_Net2PP_RX(B2A_busy_Net2PP_RX),                 // Input indicating the network to post-processing reception is busy
        .msg_accessed(B2A_msg_accessed),                     // Input indicating message access
        .sizeRX_msg(B2A_sizeRX_msg),                         // Input for size of RX message

        .busy_PP2Net_RX(B2A_busy_PP2Net_RX),                 // Output indicating post-processing to network reception is busy

        .reset_sift_parameter(reset_sift_parameter),     // Input to reset sift parameters
        .Zbasis_Xbasis_fifo_full(Zbasis_Xbasis_fifo_full), // Output indicating fifo full status

        .A_unpacket_state(A_unpacket_state),             // Output state of the A_unpacket FSM

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


//****************************** B2A BRAM instantiation ******************************
    
    B2A_BRAM B2Abram (
        .clka(B_TX_bram_clkb),    // input wire clka
        .ena(B_TX_bram_enb),      // input wire ena
        .wea(B_TX_bram_web),      // input wire [0 : 0] wea
        .addra(B_TX_bram_addrb),  // input wire [10 : 0] addra
        .dina(B_TX_bram_dinb),    // input wire [31 : 0] dina
        .douta(),  // output wire [31 : 0] douta


        .clkb(A_RX_bram_clkb),    // input wire clkb
        .enb(A_RX_bram_enb),      // input wire enb
        .web(A_RX_bram_web),      // input wire [0 : 0] web
        .addrb(A_RX_bram_addrb),  // input wire [10 : 0] addrb
        .dinb(),    // input wire [31 : 0] dinb
        .doutb(A_RX_bram_doutb)  // output wire [31 : 0] doutb
    );

//****************************** B2A BRAM instantiation ******************************



//****************************** B2A model  ******************************
    // Input 
    // wire clk;
    // wire rst_n;
    // wire busy_PP2Net_TX;
    // wire msg_stored;
    // wire [10:0] sizeTX_msg;
    // wire busy_PP2Net_RX;

    // // Output 
    // wire busy_Net2PP_TX;
    // wire busy_Net2PP_RX;
    // wire msg_accessed;
    // wire [10:0] sizeRX_msg; 
    wire [3:0] B2A_state;

    TXRX_model B2Amodel (
        .clk(clk),                       // Clock signal
        .rst_n(rst_n),                   // Reset signal

        .busy_Net2PP_TX(B2A_busy_Net2PP_TX), // Output indicating the network to post-processing transmission is busy

        .busy_PP2Net_TX(B2A_busy_PP2Net_TX), // Input indicating post-processing to network transmission is busy
        .msg_stored(B2A_msg_stored),         // Input indicating message is stored
        .sizeTX_msg(B2A_sizeTX_msg),         // Input for size of TX message

        .busy_PP2Net_RX(B2A_busy_PP2Net_RX), // Input indicating post-processing to network reception is busy

        .busy_Net2PP_RX(B2A_busy_Net2PP_RX), // Output indicating the network to post-processing reception is busy
        .msg_accessed(B2A_msg_accessed),     // Output indicating message access
        .sizeRX_msg(B2A_sizeRX_msg),         // Output register for size of RX message

        .state(B2A_state)            // Output state of the A2B model FSM
    );

//****************************** B2A model  ******************************


































































































// //****************************** A2B test instantiation ******************************

//     wire [3:0] A2B_state;

//     A2B_test A2Btest(

//         .clk(clk),
//         .rst_n(rst_n),


//         // A_A2B decoy fifo
//         .A_TX_decoy_rd_clk(A_TX_decoy_rd_clk),
//         .A_TX_decoy_rd_en(A_TX_decoy_rd_en),
//         .A_TX_decoy_rd_dout(A_TX_decoy_rd_dout),
//         .A_TX_decoy_empty(A_TX_decoy_empty),
//         .A_TX_decoy_rd_valid(A_TX_decoy_rd_valid),

//         // B_A2B decoy fifo
//         .B_RX_Zbasis_decoy_wr_clk(B_RX_Zbasis_decoy_wr_clk),
//         .B_RX_Zbasis_decoy_wr_din(B_RX_Zbasis_decoy_wr_din),
//         .B_RX_Zbasis_decoy_wr_en(B_RX_Zbasis_decoy_wr_en),
//         .B_RX_Zbasis_decoy_full(B_RX_Zbasis_decoy_full),
//         .B_RX_Zbasis_decoy_wr_ack(B_RX_Zbasis_decoy_wr_ack),



//         .A2B_state(A2B_state)
//     );

// //****************************** A2B test instantiation ******************************




//****************************** A packet  ******************************
    // Input 
    // wire clk;
    // wire rst_n;
    wire A2B_busy_Net2PP_TX;


    // Output 
    wire A2B_busy_PP2Net_TX;
    wire A2B_msg_stored;
    wire [10:0] A2B_sizeTX_msg; // Assuming this should be a register based on your module definition

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
        .clk(clk),                            // Clock signal
        .rst_n(rst_n),                        // Reset signal

        .busy_Net2PP_TX(A2B_busy_Net2PP_TX),      // Input indicating the network to post-processing transmission is busy

        .busy_PP2Net_TX(A2B_busy_PP2Net_TX),      // Output indicating post-processing to network transmission is busy
        .msg_stored(A2B_msg_stored),              // Output indicating message is stored
        .sizeTX_msg(A2B_sizeTX_msg),              // Output register for message size

        .A_packet_state(A_packet_state),      // Output state of the A_packet FSM

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



//****************************** B unpacket  ******************************

    // Input
    // wire clk;
    // wire rst_n;
    wire A2B_busy_Net2PP_RX;
    wire A2B_msg_accessed;
    wire [10:0] A2B_sizeRX_msg;
    wire reset_er_parameter;
    wire reset_pa_parameter;

    // Output
    wire A2B_busy_PP2Net_RX;
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
        .clk(clk),                                       // Clock signal
        .rst_n(rst_n),                                   // Reset signal

        .busy_Net2PP_RX(A2B_busy_Net2PP_RX),                 // Input indicating the network to post-processing reception is busy
        .msg_accessed(A2B_msg_accessed),                     // Input indicating message access
        .sizeRX_msg(A2B_sizeRX_msg),                         // Input for size of RX message

        .busy_PP2Net_RX(A2B_busy_PP2Net_RX),                 // Output indicating post-processing to network reception is busy

        .reset_er_parameter(reset_er_parameter),         // Input to reset error reconciliation parameter
        .EVrandombit_full(EVrandombit_full),             // Output indicating EV random bit buffer is full

        .reset_pa_parameter(B_reset_pa_parameter),         // Input to reset post-authentication parameter
        .PArandombit_full(PArandombit_full),             // Output indicating PA random bit buffer is full

        .B_unpacket_state(B_unpacket_state),             // Output state of the B_unpacket FSM

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



//****************************** A2B BRAM instantiation ******************************
    
    A2B_BRAM A2Bbram (
        .clka(A_TX_bram_clkb),    // input wire clka
        .ena(A_TX_bram_enb),      // input wire ena
        .wea(A_TX_bram_web),      // input wire [0 : 0] wea
        .addra(A_TX_bram_addrb),  // input wire [10 : 0] addra
        .dina(A_TX_bram_dinb),    // input wire [31 : 0] dina
        .douta(),  // output wire [31 : 0] douta


        .clkb(B_RX_bram_clkb),    // input wire clkb
        .enb(B_RX_bram_enb),      // input wire enb
        .web(B_RX_bram_web),      // input wire [0 : 0] web
        .addrb(B_RX_bram_addrb),  // input wire [10 : 0] addrb
        .dinb(),    // input wire [31 : 0] dinb
        .doutb(B_RX_bram_doutb)  // output wire [31 : 0] doutb
    );

//****************************** A2B BRAM instantiation ******************************



//****************************** A2B model  ******************************
    // Input 
    // wire clk;
    // wire rst_n;
    // wire busy_PP2Net_TX;
    // wire msg_stored;
    // wire [10:0] sizeTX_msg;
    // wire busy_PP2Net_RX;

    // // Output 
    // wire busy_Net2PP_TX;
    // wire busy_Net2PP_RX;
    // wire msg_accessed;
    // wire [10:0] sizeRX_msg; 
    wire [3:0] A2B_state;

    TXRX_model A2Bmodel (
        .clk(clk),                       // Clock signal
        .rst_n(rst_n),                   // Reset signal

        .busy_Net2PP_TX(A2B_busy_Net2PP_TX), // Output indicating the network to post-processing transmission is busy

        .busy_PP2Net_TX(A2B_busy_PP2Net_TX), // Input indicating post-processing to network transmission is busy
        .msg_stored(A2B_msg_stored),         // Input indicating message is stored
        .sizeTX_msg(A2B_sizeTX_msg),         // Input for size of TX message

        .busy_PP2Net_RX(A2B_busy_PP2Net_RX), // Input indicating post-processing to network reception is busy

        .busy_Net2PP_RX(A2B_busy_Net2PP_RX), // Output indicating the network to post-processing reception is busy
        .msg_accessed(A2B_msg_accessed),     // Output indicating message access
        .sizeRX_msg(A2B_sizeRX_msg),         // Output register for size of RX message

        .state(A2B_state)            // Output state of the A2B model FSM
    );

//****************************** A2B model  ******************************

endmodule