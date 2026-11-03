`timescale 1ns/1ps
// 4-DW MWr64 headers preserve whole 128-bit pixel beats. Same Pango payload
// DWORD byte-swap as the working Stage1 TX. Header TC/RO/IDO remain zero.
// Cache a complete legal segment before its first TLP, then hold valid/data/last
// through arbitrary backpressure. Completion is the FINAL payload handshake.
module plate_mwr_tx_ctrl (
    input clk, input rst_n, input [2:0] cfg_mps, input [15:0] requester_id,
    input req_valid, output req_ready, input [63:0] req_addr,
    input [9:0] req_dwords, input cancel,
    input payload_valid, output payload_ready, input [127:0] payload_data,
    output axis_valid, input axis_ready, output [127:0] axis_data,
    output axis_last, output axis_user,
    output segment_done, output segment_aborted, output busy
);
    localparam IDLE=0,FILL=1,HEADER=2,FETCH=3,PAYLOAD=4,DONE=5,ABORTED=6;
    reg [2:0] state;
    reg [127:0] cache [0:255];
    reg [127:0] word_data;
    reg [7:0] fill_index,read_index;
    reg [8:0] fill_beats;
    reg [63:0] address;
    reg [9:0] remaining,packet_length,packet_remaining;
    reg [9:0] mps_words;
    reg [15:0] requester;
    reg [7:0] tag;
    wire [9:0] cfg_words=10'd32 << cfg_mps;
    wire [9:0] take=packet_remaining<4 ? packet_remaining : 10'd4;
    wire [31:0] header=32'h60000000 | {22'b0,packet_length};
    wire [7:0] dwbe=packet_length==1 ? 8'h0f : 8'hff;
    assign req_ready=(state==IDLE && cfg_mps<=3);
    assign payload_ready=(state==FILL && !cancel);
    assign axis_valid=(state==HEADER || state==PAYLOAD);
    assign axis_data=state==HEADER ? {address[31:2],2'b0,address[63:32],requester,tag,dwbe,header}
                                 : endian_convert(word_data);
    assign axis_last=(state==PAYLOAD && packet_remaining<=4);
    assign axis_user=0;
    assign segment_done=(state==DONE);
    assign segment_aborted=(state==ABORTED);
    assign busy=(state!=IDLE);
    always @(posedge clk) begin
        if(!rst_n) begin
            state<=IDLE; word_data<=0; fill_index<=0; read_index<=0; fill_beats<=0;
            address<=0; remaining<=0; packet_length<=0; packet_remaining<=0;
            mps_words<=0; requester<=0; tag<=0;
        end else case(state)
            IDLE: if(req_valid && req_ready) begin
                if(req_dwords==0 || req_addr[1:0]!=0 ||
                   ({1'b0,req_addr[11:0]}+{1'b0,req_dwords,2'b0})>4096) state<=ABORTED;
                else begin
                    state<=FILL; fill_index<=0; read_index<=0;
                    fill_beats<=({1'b0,req_dwords}+11'd3)>>2;
                    address<=req_addr; remaining<=req_dwords;
                    packet_length<=req_dwords<cfg_words ? req_dwords : cfg_words;
                    mps_words<=cfg_words; requester<=requester_id;
                end
            end
            FILL: begin
                if(cancel) state<=ABORTED;
                else if(payload_valid && payload_ready) begin
                    cache[fill_index]<=payload_data;
                    if({1'b0,fill_index}+9'd1==fill_beats) state<=HEADER;
                    else fill_index<=fill_index+1;
                end
            end
            HEADER: if(axis_ready) begin packet_remaining<=packet_length; state<=FETCH; end
            FETCH: begin word_data<=cache[read_index]; state<=PAYLOAD; end
            PAYLOAD: if(axis_ready) begin
                remaining<=remaining-take;
                read_index<=read_index+1;
                if(packet_remaining<=4) begin
                    tag<=tag+1;
                    if(remaining==take) state<=DONE;
                    else begin
                        address<=address+{52'b0,packet_length,2'b0};
                        packet_length<=remaining-take<mps_words ? remaining-take : mps_words;
                        state<=HEADER;
                    end
                end else begin packet_remaining<=packet_remaining-4; state<=FETCH; end
            end
            DONE: state<=IDLE;
            ABORTED: state<=IDLE;
            default: state<=IDLE;
        endcase
    end
    function [127:0] endian_convert;
        input [127:0] value;
        integer j;
        begin for(j=0;j<4;j=j+1)
            endian_convert[j*32 +:32]={value[j*32 +:8],value[j*32+8 +:8],
                                     value[j*32+16 +:8],value[j*32+24 +:8]};
        end
    endfunction
endmodule
