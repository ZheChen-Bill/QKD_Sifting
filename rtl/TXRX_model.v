




module TXRX_model (
    input clk,
    input rst_n,

    output reg busy_Net2PP_TX,

    input busy_PP2Net_TX,
    input msg_stored,
    input [10:0] sizeTX_msg,


    input busy_PP2Net_RX,

    output reg busy_Net2PP_RX,
    output reg msg_accessed,
    output reg [10:0] sizeRX_msg,

    output reg [3:0] state
);


    localparam IDLE                 = 4'd0;
    localparam START_TRANSMIT_DATA  = 4'd1;
    localparam TRANSMIT_DATA_BUSY   = 4'd2;
    localparam WAIT_MSG_STORED_0    = 4'd3;
    localparam WAIT_PP2NET_RX_1     = 4'd4;
    localparam WAIT_PP2NET_RX_0     = 4'd5;



    wire transmit_finish;
    reg [31:0] transmit_counter;

    always @(posedge clk ) begin
        if(~rst_n) begin
            transmit_counter <= 32'b0;
        end
        else if (state==TRANSMIT_DATA_BUSY) begin
            transmit_counter <= transmit_counter + 1;
        end
        else begin
            transmit_counter <= 32'b0;
        end
    end

    assign transmit_finish = (transmit_counter==100)? 1'b1:1'b0;









    always @(posedge clk ) begin
        if(~rst_n) begin
            sizeRX_msg <= 11'b0;
        end
        else if (msg_stored) begin
            sizeRX_msg <= sizeTX_msg;
        end
        else begin
            sizeRX_msg <= sizeRX_msg;
        end
    end













    reg [3:0] next_state;
    always @(posedge clk ) begin
        if (~rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case (state)
            IDLE: begin
                if (msg_stored && ~busy_PP2Net_TX && ~busy_PP2Net_RX) begin
                    next_state = START_TRANSMIT_DATA;
                    busy_Net2PP_TX = 1'b0;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b0;
                end
                else begin
                    next_state = IDLE;
                    busy_Net2PP_TX = 1'b0;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b0;
                end
            end

            START_TRANSMIT_DATA: begin
                next_state = TRANSMIT_DATA_BUSY;
                busy_Net2PP_TX = 1'b1;
                busy_Net2PP_RX = 1'b1;
                msg_accessed = 1'b0;
            end

            TRANSMIT_DATA_BUSY: begin
                if (transmit_finish) begin
                    next_state = WAIT_MSG_STORED_0;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b1;
                    msg_accessed = 1'b0;
                end
                else begin
                    next_state = TRANSMIT_DATA_BUSY;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b1;
                    msg_accessed = 1'b0;
                end
            end

            WAIT_MSG_STORED_0: begin
                if (~msg_stored) begin
                    next_state = WAIT_PP2NET_RX_1;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b1;
                    msg_accessed = 1'b0;
                end
                else begin
                    next_state = WAIT_MSG_STORED_0;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b1;
                    msg_accessed = 1'b0;
                end
            end


            WAIT_PP2NET_RX_1: begin
                if (busy_PP2Net_RX) begin
                    next_state = WAIT_PP2NET_RX_0;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b1;
                end
                else begin
                    next_state = WAIT_PP2NET_RX_1;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b1;
                end
            end

            WAIT_PP2NET_RX_0: begin
                if (~busy_PP2Net_RX) begin
                    next_state = IDLE;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b0;
                end
                else begin
                    next_state = WAIT_PP2NET_RX_0;
                    busy_Net2PP_TX = 1'b1;
                    busy_Net2PP_RX = 1'b0;
                    msg_accessed = 1'b0;
                end
            end



            default: begin
                next_state = IDLE;
                busy_Net2PP_TX = 1'b0;
                busy_Net2PP_RX = 1'b0;
                msg_accessed = 1'b0;
            end
        endcase
    end

    
endmodule