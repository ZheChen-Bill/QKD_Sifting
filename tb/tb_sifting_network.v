`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/07/07 23:19:33
// Design Name: 
// Module Name: tb_sifting_network
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
`include "sifting_parameter.v"


module tb_sifting_network(

    );
    //    parameter CLK_PERIOD = 8;
    parameter CLK_PERIOD = 10;
    reg clk;
    //----------------------------new add signal------------------------------
    reg clk_100M;
    reg clk_GMII;

    reg gmii_tx_clk;
    reg gmii_rx_clk;

    reg clk_PP;

    wire [7:0]   gmii_rxd_TX; // Received Data to client MAC
    wire           gmii_rx_dv_TX; // Received control signal to client MAC.
    wire           gmii_rx_er_TX;

    wire [7:0]   gmii_txd_TX;
    wire           gmii_tx_en_TX;
    wire           gmii_tx_er_TX;

    wire A2B_busy_PP2Net_TX;
    wire A2B_busy_Net2PP_TX;
    
    wire A2B_busy_Net2PP_RX;
    wire A2B_busy_PP2Net_RX;

    wire B2A_busy_PP2Net_TX;
    wire B2A_busy_Net2PP_TX;
    
    wire B2A_busy_Net2PP_RX;
    wire B2A_busy_PP2Net_RX;
    reg   link_status;
    
    reg rst_n;
    reg start_switch;
    
    reg start_B_TX;
    wire wait_B_TX;
    reg start_A_TX;
    wire wait_A_TX;
// clock control
//=============================================================    
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end


    initial begin
        clk_PP = 1'b0;
        forever begin
            //            #1.315 clk_PP = 1'b1;
            //            #1.315 clk_PP = 1'b0;
            #1.3334 clk_PP = 1'b1;
            #1.3334 clk_PP = 1'b0;
        end
    end

    initial begin
        clk_100M = 1'b0;
        forever begin
            #5 clk_100M = 1'b1;
            #5 clk_100M = 1'b0;
        end
    end
    // clk_GMII = 125MHz
    initial begin
        clk_GMII = 1'b0;
        forever begin
            #4 clk_GMII = 1'b1;
            #4 clk_GMII = 1'b0;
        end
    end

    initial begin
        gmii_tx_clk = 1'b0;
        gmii_rx_clk = 1'b0;
        forever begin
            #4 gmii_tx_clk = 1'b1; gmii_rx_clk = 1'b1;
            #4 gmii_tx_clk = 1'b0; gmii_rx_clk = 1'b0;
        end
    end
//=============================================================
//link status control (simulate SFP module)
//=============================================================
    initial begin
        link_status = 1;
    end
//=============================================================
//start signal and reset signal
//=============================================================
    initial begin
        rst_n = 1;
        #(CLK_PERIOD*2000) rst_n = 0;
        #(CLK_PERIOD*1000) rst_n = 1;
    end
    initial begin
        start_switch = 0;
        wait (rst_n == 0);
        wait (rst_n == 1);
        #(CLK_PERIOD*2000);
        start_switch = 1;
    end
//=============================================================
//wait ALICE TX and wait Bob TX signal
//=============================================================
    initial begin
        start_B_TX = 0;
        wait (wait_B_TX == 1);
        #(CLK_PERIOD*1000);
        start_B_TX = 1;
        #(CLK_PERIOD*10);
        start_B_TX = 0;
    end 
    initial begin
        start_A_TX = 0;
        wait (wait_A_TX == 1);
        #(CLK_PERIOD*2000);
        start_A_TX = 1;
        #(CLK_PERIOD*20);
        start_A_TX = 0;
    end 
//=============================================================

    // Bob sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    wire [63:0] Bsiftedkey_dina;     //Alice sifted key 
    wire [14:0] Bsiftedkey_addra;    //0~32767
    wire Bsiftedkey_clka;
    wire Bsiftedkey_ena;                    //1'b1
    wire Bsiftedkey_wea;              //




    // Alice sifted key BRAM (output)
    // width = 64 , depth = 32768
    // port A
    wire [63:0] Asiftedkey_dina;     //Alice sifted key 
    wire [14:0] Asiftedkey_addra;    //0~32767
    wire Asiftedkey_clka;
    wire Asiftedkey_ena;                    //1'b1
    wire Asiftedkey_wea;              //
    


    wire [`NVIS_WIDTH-1:0] nvis;                  //nvis
    wire [`A_CHECKKEY_1_WIDTH-1:0] A_checkkey_1;  //A_checkkey_1
    wire [`A_CHECKKEY_0_WIDTH-1:0] A_checkkey_0;  //A_checkkey_0
    wire [`COMPARE_1_WIDTH-1:0] A_compare_1;      //A_compare_1
    wire [`COMPARE_0_WIDTH-1:0] A_compare_0;      //A_compare_0
    wire A_visibility_valid;                      //visibility parameter is valid

    wire A_sifting_finish;                //sifting is done
    wire B_sifting_finish;                //sifting is done
    
    Alice test_A(
        .clk(clk),
        .rst_n(rst_n),

        //        .clk_100M(clk_100M),
        //        .clk_GMII(clk_GMII),
        .gmii_tx_clk(gmii_tx_clk),
        .gmii_rx_clk(gmii_rx_clk),
        .clk_PP(clk_PP),
        .link_status(link_status),

        .A2B_busy_PP2Net_TX(A2B_busy_PP2Net_TX),
        .A2B_busy_Net2PP_TX(A2B_busy_Net2PP_TX),

        .A2B_busy_Net2PP_RX(A2B_busy_Net2PP_RX),
        .A2B_busy_PP2Net_RX(A2B_busy_PP2Net_RX),

        .gmii_txd(gmii_rxd_TX), // Transmit data from client MAC.
        .gmii_tx_en(gmii_rx_dv_TX), // Transmit control signal from client MAC.
        .gmii_tx_er(gmii_rx_er_TX), // Transmit control signal from client MAC.
        .gmii_rxd(gmii_txd_TX), // Received Data to client MAC.
        .gmii_rx_dv(gmii_tx_en_TX), // Received control signal to client MAC.
        .gmii_rx_er(gmii_tx_er_TX), // Received control signal to client MAC.
               
        .start_switch(start_switch),
        .start_A_TX(start_A_TX), //start A transport
        .wait_A_TX(wait_A_TX), //indicate A is in wait state


        // Bob sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        //        .Bsiftedkey_dina(Bsiftedkey_dina),     //Alice sifted key 
        //        .Bsiftedkey_addra(Bsiftedkey_addra),    //0~32767
        //        .Bsiftedkey_clka(Bsiftedkey_clka),
        //        .Bsiftedkey_ena(Bsiftedkey_ena),                    //1'b1
        //        .Bsiftedkey_wea(Bsiftedkey_wea),              //


        // Alice sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        .Asiftedkey_dina(Asiftedkey_dina), //Alice sifted key 
        .Asiftedkey_addra(Asiftedkey_addra), //0~32767
        .Asiftedkey_clka(Asiftedkey_clka),
        .Asiftedkey_ena(Asiftedkey_ena), //1'b1
        .Asiftedkey_wea(Asiftedkey_wea), //


        .nvis(nvis),
        .A_checkkey_1(A_checkkey_1),
        .A_checkkey_0(A_checkkey_0),
        .A_compare_1(A_compare_1),
        .A_compare_0(A_compare_0),
        .A_visibility_valid(A_visibility_valid),
        .A_sifting_finish(A_sifting_finish)
        //        .B_sifting_finish(B_sifting_finish)
    );
    
        Bob test_B(
        .clk(clk),
        .rst_n(rst_n),

        //        .clk_100M(clk_100M),
        //        .clk_GMII(clk_GMII),
        .gmii_tx_clk(gmii_tx_clk),
        .gmii_rx_clk(gmii_rx_clk),
        .clk_PP(clk_PP),
        .link_status(link_status),

        .B2A_busy_PP2Net_TX(B2A_busy_PP2Net_TX),
        .B2A_busy_Net2PP_TX(B2A_busy_Net2PP_TX),

        .B2A_busy_Net2PP_RX(B2A_busy_Net2PP_RX),
        .B2A_busy_PP2Net_RX(B2A_busy_PP2Net_RX),

        .gmii_txd(gmii_txd_TX), // Transmit data from client MAC.
        .gmii_tx_en(gmii_tx_en_TX), // Transmit control signal from client MAC.
        .gmii_tx_er(gmii_tx_er_TX), // Transmit control signal from client MAC.
        .gmii_rxd(gmii_rxd_TX), // Received Data to client MAC.
        .gmii_rx_dv(gmii_rx_dv_TX), // Received control signal to client MAC.
        .gmii_rx_er(gmii_rx_er_TX), // Received control signal to client MAC.

        .start_switch(start_switch),

        .start_B_TX(start_B_TX),
        .wait_B_TX(wait_B_TX),

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
        //        .A_sifting_finish(A_sifting_finish)
        .B_sifting_finish(B_sifting_finish)
    );
    
    reg A_finish, B_finish;
    always @(posedge clk ) begin
        if (~rst_n) begin
            A_finish <= 1'b0;
        end
        else if (A_sifting_finish) begin
            A_finish <= 1'b1;
        end
        else begin
            A_finish <= A_finish;
        end
    end

    always @(posedge clk ) begin
        if (~rst_n) begin
            B_finish <= 1'b0;
        end
        else if (B_sifting_finish) begin
            B_finish <= 1'b1;
        end
        else begin
            B_finish <= B_finish;
        end
    end
    
    always @(*) begin
        if (B_finish && A_finish) begin
            $display("[FINISH]");
            $display("[FINISH]");
            $display("[FINISH]");
            $display("[FINISH]");
            $display("[FINISH]");
            $display("[FINISH]");
            $display("[FINISH]");
            #(CLK_PERIOD*10);
            $fclose(A_siftedkey_out);
            $fclose(B_siftedkey_out);
            $fclose(nvis_out);
            $fclose(A_checkkey_1_out);
            $fclose(A_checkkey_0_out);
            $fclose(A_comparekey_1_out);
            $fclose(A_comparekey_0_out);
            #2000;
            $finish;
        end
    end
    
    integer A_siftedkey_out;
    integer B_siftedkey_out;
    integer nvis_out;
    integer A_checkkey_1_out;
    integer A_checkkey_0_out;
    integer A_comparekey_1_out;
    integer A_comparekey_0_out;


    initial begin
       A_siftedkey_out = $fopen("../../../../HW_sim_result/A_siftedkey_out.txt", "w");
       B_siftedkey_out = $fopen("../../../../HW_sim_result/B_siftedkey_out.txt", "w");
       nvis_out = $fopen("../../../../HW_sim_result/nvis_out.txt", "w");
       A_checkkey_1_out = $fopen("../../../../HW_sim_result/A_checkkey_1_out.txt", "w");
       A_checkkey_0_out = $fopen("../../../../HW_sim_result/A_checkkey_0_out.txt", "w");
       A_comparekey_1_out = $fopen("../../../../HW_sim_result/A_comparekey_1_out.txt", "w");
       A_comparekey_0_out = $fopen("../../../../HW_sim_result/A_comparekey_0_out.txt", "w");
    end
    
    always @(*) begin
        if (Asiftedkey_ena && Asiftedkey_wea) begin
            $fdisplay(A_siftedkey_out,"%b",Asiftedkey_dina);
        end
    end

    always @(*) begin
        if (Bsiftedkey_ena && Bsiftedkey_wea) begin
            $fdisplay(B_siftedkey_out,"%b",Bsiftedkey_dina);
        end
    end

    always @(posedge clk) begin
        if (A_visibility_valid) begin
            $fdisplay(nvis_out,"%0d",nvis);
        end
    end

    always @(posedge clk) begin
        if (A_visibility_valid) begin
            $fdisplay(A_checkkey_1_out,"%0d",A_checkkey_1);
        end
    end

    always @(posedge clk) begin
        if (A_visibility_valid) begin
            $fdisplay(A_checkkey_0_out,"%0d",A_checkkey_0);
        end
    end

    always @(posedge clk) begin
        if (A_visibility_valid) begin
            $fdisplay(A_comparekey_1_out,"%0d",A_compare_1);
        end
    end

    always @(posedge clk) begin
        if (A_visibility_valid) begin
            $fdisplay(A_comparekey_0_out,"%0d",A_compare_0);
        end
    end
    
    
endmodule
