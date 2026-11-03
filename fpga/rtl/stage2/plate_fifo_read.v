`timescale 1ns/1ps
// Same-domain replacement for legacy prefetch control. CANCEL does not withdraw
// an issued request or reset the DDR master: the current burst must retire first.
module plate_fifo_read #(
    parameter MEM_DATA_BITS=128, ADDR_BITS=28, BUSRT_BITS=10,
    parameter FIFO_DEPTH=512, BURST_SIZE=160
)(
    input rst, input mem_clk,
    output reg rd_burst_req, output reg [BUSRT_BITS-1:0] rd_burst_len,
    output reg [ADDR_BITS-1:0] rd_burst_addr,
    input rd_burst_data_valid, input rd_burst_finish,
    input read_req, output read_req_ack, output read_finish,
    input [ADDR_BITS-1:0] read_addr_0,read_addr_1,read_addr_2,read_addr_3,
    input [1:0] read_addr_index, output reg [1:0] read_addr_index_used,
    input [ADDR_BITS-1:0] read_len, input read_cancel,
    output fifo_aclr, input [15:0] wrusedw, output read_idle
);
    localparam IDLE=0, CLEAR=1, ACK=2, CHECK=3, BURST=4, DONE=5, CANCEL_ACK=6;
    reg [2:0] state;
    reg [7:0] clear_count;
    reg cancelled;
    reg [ADDR_BITS-1:0] remaining;
    wire [ADDR_BITS-1:0] chunk=remaining<BURST_SIZE ? remaining : BURST_SIZE;
    assign fifo_aclr=(state==CLEAR);
    assign read_req_ack=(state==ACK || state==CANCEL_ACK);
    assign read_finish=(state==DONE || state==CANCEL_ACK);
    assign read_idle=(state==IDLE || state==CANCEL_ACK);
    always @(posedge mem_clk) begin
        if(rst) begin
            state<=IDLE; clear_count<=0; cancelled<=0; remaining<=0;
            rd_burst_req<=0; rd_burst_len<=0; rd_burst_addr<=0;
            read_addr_index_used<=0;
        end else begin
            if(state!=IDLE && read_cancel) cancelled<=1;
            case(state)
                IDLE: if(read_req) begin
                    cancelled<=read_cancel; rd_burst_req<=0;
                    if(read_cancel) state<=CANCEL_ACK;
                    else begin
                        state<=CLEAR; clear_count<=0; remaining<=read_len;
                        read_addr_index_used<=read_addr_index;
                        case(read_addr_index)
                            0:rd_burst_addr<=read_addr_0;
                            1:rd_burst_addr<=read_addr_1;
                            2:rd_burst_addr<=read_addr_2;
                            3:rd_burst_addr<=read_addr_3;
                        endcase
                    end
                end
                CLEAR: begin
                    if(clear_count==31) state<=ACK;
                    else clear_count<=clear_count+1;
                end
                ACK: if(!read_req) state<=CHECK;
                CHECK: begin
                    if(cancelled || read_cancel || remaining==0) state<=DONE;
                    else if(wrusedw<160) begin
                        rd_burst_len<=chunk[BUSRT_BITS-1:0];
                        rd_burst_req<=1; state<=BURST;
                    end
                end
                BURST: begin
                    if(rd_burst_data_valid) rd_burst_req<=0;
                    if(rd_burst_finish) begin
                        rd_burst_req<=0; state<=CHECK;
                        remaining<=remaining-rd_burst_len;
                        rd_burst_addr<=rd_burst_addr+rd_burst_len;
                    end
                end
                DONE: state<=IDLE;
                CANCEL_ACK: if(!read_req) state<=IDLE;
                default: state<=IDLE;
            endcase
        end
    end
endmodule
