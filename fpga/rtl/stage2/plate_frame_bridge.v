`timescale 1ns/1ps
// Lock identity before requesting FIFO reset/address latch; acknowledge the
// source only after the actual FIFO address matches that locked identity.
module plate_frame_bridge (
    input mem_clk, input rst, input [63:0] mclk_tick,
    input wframe_vsync, input write_req_ack, input write_finish,
    input [1:0] write_buf_used, input read_finish,
    input frame_req, input frame_cancel, output reg frame_ack,
    output fifo_req, input fifo_ack, input [1:0] fifo_addr_used,
    output write_req, output [1:0] write_addr_index, output [1:0] read_addr_index,
    output [31:0] source_seq, output [63:0] source_tick,
    output reg address_error
);
    localparam IDLE=0, LOCK=1, FIFO=2, ACK=3;
    reg [1:0] state;
    reg cancel_release;
    wire pool_ack;
    wire acquire=(state==LOCK && !frame_cancel);
    assign fifo_req=(state==FIFO);
    plate_frame_pool_ctrl pool(.mclk(mem_clk),.mrst(rst),.mclk_tick(mclk_tick),
        .wframe_vsync(wframe_vsync),.write_req_ack(write_req_ack),
        .write_finish(write_finish),.write_buf_used(write_buf_used),
        .read_finish(read_finish || cancel_release),.read_acquire(acquire),
        .read_acquire_ack(pool_ack),.write_req(write_req),
        .write_addr_index(write_addr_index),.read_buf(read_addr_index),
        .read_source_seq(source_seq),.read_source_tick(source_tick));
    always @(posedge mem_clk) begin
        if(rst) begin state<=IDLE; frame_ack<=0; cancel_release<=0; address_error<=0; end
        else begin
            cancel_release<=0;
            case(state)
                IDLE: begin
                    frame_ack<=0;
                    if(frame_req) begin
                        if(frame_cancel) begin frame_ack<=1; state<=ACK; end
                        else state<=LOCK;
                    end
                end
                LOCK: begin
                    if(frame_cancel) begin
                        cancel_release<=1; frame_ack<=1; state<=ACK;
                    end else if(pool_ack) state<=FIFO;
                end
                FIFO: if(fifo_ack) begin
                    // A cancel-ack may not latch an address because no DDR read
                    // was issued. It is terminal control, never a valid picture.
                    if(frame_cancel || fifo_addr_used==read_addr_index) begin frame_ack<=1; state<=ACK; end
                    else address_error<=1;
                end
                ACK: if(!frame_req) begin frame_ack<=0; state<=IDLE; end
            endcase
        end
    end
endmodule
