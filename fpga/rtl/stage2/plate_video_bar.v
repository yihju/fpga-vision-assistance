`include "plate_video_protocol.vh"
// RX exposes 16-byte block addresses with shifted DWORD data/byte enables.
// Keep all address bits; never alias command 0x200 to legacy registers.
module plate_video_bar #(
    parameter MSI_CAPABLE = 1,
    parameter [31:0] CLOCK_HZ = 32'd131250000
)(
    input clk, input rst_n,
    input wr_en, input [11:0] wr_addr,
    input [127:0] wr_data, input [15:0] wr_be,
    input rd_en, input [11:0] rd_addr,
    output reg rd_hit, output reg [127:0] rd_data,
    input dma_busy, input msi_enabled, input [2:0] cfg_mps,
    input [63:0] last_seq, input [31:0] last_bytes, input [31:0] last_status,
    output reg cmd_valid, input cmd_ready,
    output reg [63:0] cmd_addr, output reg [63:0] cmd_seq,
    output reg [31:0] cmd_cookie, output reg [31:0] cmd_epoch,
    output reg [31:0] cmd_slot, output reg [31:0] cmd_bytes,
    output reg stop, output reg [31:0] error_count
);
reg [31:0] words [0:10];
localparam [31:0] FEATURES=MSI_CAPABLE ? 32'hf : 32'hb;
integer i, lane, idx;
function [31:0] merge_bytes;
    input [31:0] old_word, new_word;
    input [3:0] enables;
    integer b;
    begin
        merge_bytes=old_word;
        for(b=0;b<4;b=b+1)
            if(enables[b]) merge_bytes[b*8+:8]=new_word[b*8+:8];
    end
endfunction
wire doorbell = wr_en && wr_addr==12'h022 && wr_be[12];
wire [31:0] bell_data = wr_data[127:96];
wire submit = doorbell && bell_data[0];
wire stop_command = doorbell && bell_data[1];
wire submit_ok = !dma_busy && !cmd_valid && words[8]==0 && words[9]==0 &&
                 words[10]==0 && (bell_data & 32'hfffffffc)==0 &&
                 !stop_command && (!MSI_CAPABLE || msi_enabled) && cfg_mps<=3;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i=0;i<11;i=i+1) words[i]<=0;
        cmd_valid<=0;cmd_addr<=0;cmd_seq<=0;cmd_cookie<=0;cmd_epoch<=0;
        cmd_slot<=0;cmd_bytes<=0;stop<=0;error_count<=0;
    end else begin
        stop<=stop_command;
        if(cmd_valid && cmd_ready)cmd_valid<=0;
        if(wr_en && wr_addr>=12'h020 && wr_addr<=12'h022) begin
            for(lane=0;lane<4;lane=lane+1) begin
                idx=(wr_addr-12'h020)*4+lane;
                if(idx<11 && wr_be[lane*4+:4]!=0)
                    words[idx]<=merge_bytes(words[idx],wr_data[lane*32+:32],wr_be[lane*4+:4]);
            end
        end
        if(stop_command)cmd_valid<=0;
        if(submit) begin
            if(submit_ok) begin
                cmd_addr<={words[1],words[0]};cmd_cookie<=words[2];
                cmd_seq<={words[4],words[3]};cmd_epoch<=words[5];
                cmd_slot<=words[6];cmd_bytes<=words[7];cmd_valid<=1;
            end else error_count<=error_count+1'b1;
        end
    end
end
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin rd_hit<=0;rd_data<=0;end
    else begin
        rd_hit<=rd_en && ((rd_addr>=12'h1a0 && rd_addr<=12'h1a3) ||
                          (rd_addr>=12'h1b0 && rd_addr<=12'h1b3));
        if(rd_en) case(rd_addr)
            12'h1a0:rd_data<={FEATURES,`PLATE_VIDEO_MINOR,`PLATE_VIDEO_MAJOR,`PLATE_VIDEO_MAGIC};
            12'h1a1:rd_data<={`PLATE_VIDEO_FORMAT,32'd2560,32'd720,32'd1280};
            12'h1a2:rd_data<={`PLATE_VIDEO_FRAME_BYTES,`PLATE_VIDEO_SLOT_STRIDE,32'd4,`PLATE_VIDEO_FRAME_BYTES};
            12'h1a3:rd_data<={CLOCK_HZ,32'd1,32'd32,`PLATE_VIDEO_RECORD_BYTES};
            12'h1b0:rd_data<={error_count,{31'd0,msi_enabled},{31'd0,cmd_valid},{31'd0,dma_busy}};
            12'h1b1:rd_data<={last_status,last_bytes,last_seq};
            12'h1b2:rd_data<={96'd0,29'd0,cfg_mps};
            default:rd_data<=0;
        endcase
    end
end
endmodule
