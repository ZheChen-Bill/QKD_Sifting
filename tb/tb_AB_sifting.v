

`define POST_STNTHESIS_SIM      0


`timescale 1ns/1ps


`include "sifting_parameter.v"





module tb_AB_sifting ();
    parameter CLK_PERIOD = 8;

    reg clk;
    reg rst_n;
    reg start_switch;

    // ===== Clk fliping ===== //
	initial begin
		clk = 1;
		forever #(CLK_PERIOD/2) clk = ~clk;
	end

	initial begin
        rst_n = 1;
        #(CLK_PERIOD*2000) rst_n = 0;
        #(CLK_PERIOD*1000) rst_n = 1;  
	end


    initial begin
        start_switch = 0;
		wait (rst_n == 0);
		wait (rst_n == 1);
        #(CLK_PERIOD*4000);
        @(negedge clk);
        start_switch = 1;
    end







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

    
    wire A_sifting_finish;                        //sifting is done
    wire B_sifting_finish;                //sifting is done

    top_AB_sifting test_AB_sift(
        .clk(clk),
        .rst_n(rst_n),

        .start_switch(start_switch),



        // Bob sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        .Bsiftedkey_dina(Bsiftedkey_dina),     //Alice sifted key 
        .Bsiftedkey_addra(Bsiftedkey_addra),    //0~32767
        .Bsiftedkey_clka(Bsiftedkey_clka),
        .Bsiftedkey_ena(Bsiftedkey_ena),                    //1'b1
        .Bsiftedkey_wea(Bsiftedkey_wea),              //


        // Alice sifted key BRAM (output)
        // width = 64 , depth = 32768
        // port A
        .Asiftedkey_dina(Asiftedkey_dina),     //Alice sifted key 
        .Asiftedkey_addra(Asiftedkey_addra),    //0~32767
        .Asiftedkey_clka(Asiftedkey_clka),
        .Asiftedkey_ena(Asiftedkey_ena),                    //1'b1
        .Asiftedkey_wea(Asiftedkey_wea),              //


        .nvis(nvis),
        .A_checkkey_1(A_checkkey_1),
        .A_checkkey_0(A_checkkey_0),
        .A_compare_1(A_compare_1),
        .A_compare_0(A_compare_0),
        .A_visibility_valid(A_visibility_valid),
        .A_sifting_finish(A_sifting_finish),
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





    integer A_siftedkey_out;
    integer B_siftedkey_out;
    integer nvis_out;
    integer A_checkkey_1_out;
    integer A_checkkey_0_out;
    integer A_comparekey_1_out;
    integer A_comparekey_0_out;


    initial begin
        if (`POST_STNTHESIS_SIM) begin
            A_siftedkey_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/A_siftedkey_out.txt", "w");
            B_siftedkey_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/B_siftedkey_out.txt", "w");
            nvis_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/nvis_out.txt", "w");
            A_checkkey_1_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/A_checkkey_1_out.txt", "w");
            A_checkkey_0_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/A_checkkey_0_out.txt", "w");
            A_comparekey_1_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/A_comparekey_1_out.txt", "w");
            A_comparekey_0_out = $fopen("../../../../../HW_sim_result/post_synthesis_sim/A_comparekey_0_out.txt", "w");
        end
        else begin
            A_siftedkey_out = $fopen("../../../../HW_sim_result/A_siftedkey_out.txt", "w");
            B_siftedkey_out = $fopen("../../../../HW_sim_result/B_siftedkey_out.txt", "w");
            nvis_out = $fopen("../../../../HW_sim_result/nvis_out.txt", "w");
            A_checkkey_1_out = $fopen("../../../../HW_sim_result/A_checkkey_1_out.txt", "w");
            A_checkkey_0_out = $fopen("../../../../HW_sim_result/A_checkkey_0_out.txt", "w");
            A_comparekey_1_out = $fopen("../../../../HW_sim_result/A_comparekey_1_out.txt", "w");
            A_comparekey_0_out = $fopen("../../../../HW_sim_result/A_comparekey_0_out.txt", "w");
        end
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


endmodule