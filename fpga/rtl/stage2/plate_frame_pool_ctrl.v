`timescale 1ns/1ps
// All indices and identity stay in DDR mclk. Only VSYNC crosses as one bit.
// The latest completed buffer remains published so a faster transport can reread
// it honestly; metadata does not change until explicit acquisition of that buffer.
module plate_frame_pool_ctrl (
    input mclk, input mrst, input [63:0] mclk_tick,
    input wframe_vsync, input write_req_ack, input write_finish,
    input [1:0] write_buf_used, input read_finish, input read_acquire,
    output reg write_req, output reg [1:0] write_addr_index,
    output reg read_acquire_ack, output reg [1:0] read_buf,
    output reg [31:0] read_source_seq, output reg [63:0] read_source_tick,
    output dbg_rd_own, output dbg_pub_valid, output [3:0] dbg_busy_mask
);
    reg [2:0] vsync_sync;
    reg [1:0] published_buf;
    reg published_valid, read_owned, acquire_seen;
    reg [31:0] completed_seq, published_seq;
    reg [63:0] published_tick;
    wire [3:0] busy = (4'b1 << write_addr_index) |
        (published_valid ? (4'b1 << published_buf) : 4'b0) |
        (read_owned ? (4'b1 << read_buf) : 4'b0);
    wire [1:0] c1=write_addr_index+2'd1, c2=write_addr_index+2'd2,
               c3=write_addr_index+2'd3;
    wire [1:0] next_buf=!busy[c1] ? c1 : !busy[c2] ? c2 : c3;
    assign dbg_rd_own=read_owned;
    assign dbg_pub_valid=published_valid;
    assign dbg_busy_mask=busy;
    always @(posedge mclk) begin
        if(mrst) begin
            vsync_sync<=0; write_req<=0; write_addr_index<=0;
            published_buf<=0; published_valid<=0; read_owned<=0;
            acquire_seen<=0; read_acquire_ack<=0; read_buf<=0;
            read_source_seq<=0; read_source_tick<=0;
            completed_seq<=0; published_seq<=0; published_tick<=0;
        end else begin
            vsync_sync<={vsync_sync[1:0],wframe_vsync};
            // Keep the working Stage1 VSYNC polarity, do not change sensor timing.
            if(vsync_sync[2] && !vsync_sync[1]) write_req<=1;
            else if(write_req_ack) write_req<=0;
            read_acquire_ack<=0;
            if(!read_acquire) acquire_seen<=0;
            if(read_finish) read_owned<=0;
            if(write_finish) begin
                completed_seq<=completed_seq+1;
                published_seq<=completed_seq+1;
                published_tick<=mclk_tick;
                published_buf<=write_buf_used;
                published_valid<=1;
                write_addr_index<=next_buf;
            end else if(read_acquire && !acquire_seen && !read_owned && published_valid) begin
                read_owned<=1; acquire_seen<=1; read_acquire_ack<=1;
                read_buf<=published_buf;
                read_source_seq<=published_seq;
                read_source_tick<=published_tick;
            end
        end
    end
endmodule
