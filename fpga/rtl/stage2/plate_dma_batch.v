`timescale 1ns/1ps
// Serial frame scheduler. A segment is not complete on request acceptance.
// Record body and final cookie share this same ordered MWr queue.
module plate_dma_batch (
    input clk, input rst_n,
    input cmd_valid, output cmd_ready, input [63:0] cmd_addr,
    input [31:0] cmd_cookie,cmd_epoch,cmd_slot,cmd_bytes,
    input [63:0] cmd_seq, input stop,
    output reg source_start, input source_started, input source_idle,
    output source_abort, output source_retire,
    output segment_valid, input segment_ready,
    output [63:0] segment_addr, output [9:0] segment_dwords,
    output [1:0] segment_kind,
    input segment_done, input segment_aborted,
    output [31:0] completion_status, output reg [31:0] valid_bytes,
    output reg [31:0] record_cookie,record_epoch,record_slot,
    output reg [63:0] record_seq,
    output irq_req, input irq_grant, output busy, output reg command_error
);
    localparam IDLE=0, SOURCE=1, PIX_OFFER=2, PIX_WAIT=3, DRAIN=4,
               BODY_OFFER=5, BODY_WAIT=6, COOKIE_OFFER=7, COOKIE_WAIT=8, IRQ=9;
    reg [3:0] state;
    reg [63:0] base;
    reg stopping;
    reg [12:0] issued_bytes;
    wire [63:0] pixel_addr=base+valid_bytes;
    wire [12:0] to_page=13'd4096-{1'b0,pixel_addr[11:0]};
    wire [31:0] remaining=32'd1843200-valid_bytes;
    wire [12:0] max_chunk=to_page<4080 ? to_page : 13'd4080;
    wire [12:0] chunk=remaining<max_chunk ? remaining[12:0] : max_chunk;
    wire invalid_cmd=cmd_addr[63:32]!=0 || cmd_addr[11:0]!=0 ||
        cmd_addr>64'h00000000ffe3d000 || cmd_cookie==0 || cmd_slot>=4 || cmd_bytes!=1843200;
    assign cmd_ready=(state==IDLE);
    assign busy=(state!=IDLE);
    assign completion_status=stopping ? 32'd1 : 32'd0;
    assign source_abort=stopping && (state==DRAIN || state>=BODY_OFFER);
    assign source_retire=(state>=DRAIN);
    assign segment_valid=(state==PIX_OFFER && !stop && !stopping) ||
                         state==BODY_OFFER || state==COOKIE_OFFER;
    assign segment_kind=(state==PIX_OFFER || state==PIX_WAIT) ? 0 :
                        (state==BODY_OFFER || state==BODY_WAIT) ? 1 : 2;
    assign segment_addr=(state==PIX_OFFER || state==PIX_WAIT) ? pixel_addr :
                        (state==BODY_OFFER || state==BODY_WAIT) ? base+1843200 : base+1843324;
    assign segment_dwords=(state==PIX_OFFER || state==PIX_WAIT) ? chunk[11:2] :
                          (state==BODY_OFFER || state==BODY_WAIT) ? 10'd31 : 10'd1;
    assign irq_req=(state==IRQ);
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE; base<=0; stopping<=0; source_start<=0;
            valid_bytes<=0; issued_bytes<=0; command_error<=0;
            record_cookie<=0; record_epoch<=0; record_slot<=0; record_seq<=0;
        end else begin
            source_start<=0; command_error<=0;
            if(stop && state!=IDLE) stopping<=1;
            case(state)
                IDLE: if(cmd_valid) begin
                    if(invalid_cmd) command_error<=1;
                    else begin
                        base<=cmd_addr; record_cookie<=cmd_cookie; record_epoch<=cmd_epoch;
                        record_slot<=cmd_slot; record_seq<=cmd_seq;
                        valid_bytes<=0; stopping<=0; source_start<=1; state<=SOURCE;
                    end
                end
                SOURCE: if(source_started) state<=stopping || stop ? DRAIN : PIX_OFFER;
                PIX_OFFER: begin
                    if(stopping || stop) state<=DRAIN;
                    else if(segment_ready) begin issued_bytes<=chunk; state<=PIX_WAIT; end
                end
                PIX_WAIT: begin
                    if(segment_done) begin
                        valid_bytes<=valid_bytes+issued_bytes;
                        state<=stopping || stop || valid_bytes+issued_bytes==1843200 ? DRAIN : PIX_OFFER;
                    end else if(segment_aborted) begin stopping<=1; state<=DRAIN; end
                end
                DRAIN: if(source_idle) state<=BODY_OFFER;
                BODY_OFFER: if(segment_ready) state<=BODY_WAIT;
                BODY_WAIT: if(segment_done) state<=COOKIE_OFFER;
                COOKIE_OFFER: if(segment_ready) state<=COOKIE_WAIT;
                COOKIE_WAIT: if(segment_done) state<=IRQ;
                IRQ: if(irq_grant) state<=IDLE;
            endcase
        end
    end
endmodule
