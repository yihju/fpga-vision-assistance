// BAR1 0x250 START/ABORT; 0x260 sequence; 0x264 count; 0x270 packed00RRGGBB.
// BAR0 0x1d00 RPS1/status/sequence/received; 0x1d10 gray/R/G/Y counters;
// 0x1d20 active count/binary ones/{max,min}/pending count.
// RX addresses are full 16-byte block indices, exactly like the existing DMA.
module roi_stats_bar(
    input clk,input rst_n,input key_n,
    input wr_en,input [11:0] wr_addr,input [127:0] wr_data,input [15:0] wr_be,
    input rd_en,input [11:0] rd_addr,output reg rd_hit,output reg [127:0] rd_data
);
reg [31:0] pending_seq,pending_count;
reg [1:0] key_sync;
wire ctrl=wr_en && wr_addr==12'h025;
wire data_write=wr_en && wr_addr==12'h027;
wire full_word=wr_be==16'h000f;
wire command_start=ctrl && full_word && wr_data[31:0]==32'h524f0001;
wire command_abort=ctrl && full_word && wr_data[31:0]==32'h524f0002;
wire data_good=data_write && full_word && wr_data[31:24]==0;
wire bad=(ctrl && !(command_start || command_abort)) || (data_write && !data_good);
wire busy,done,error;
wire [31:0] seq,expected,received,gray_sum,red_count,green_count,yellow_count,binary_ones;
wire [7:0] gray_min,gray_max;
wire [31:0] status={23'd0,~key_sync[1],5'd0,error,done,busy};
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin pending_seq<=0;pending_count<=0;key_sync<=2'b11;end
    else begin
        key_sync<={key_sync[0],key_n};
        if(wr_en && wr_addr==12'h026)begin
            if(wr_be[3:0]==4'hf)pending_seq<=wr_data[31:0];
            if(wr_be[7:4]==4'hf)pending_count<=wr_data[63:32];
        end
    end
end
roi_pixel_stats pixels(.clk(clk),.rst_n(rst_n),.start(command_start),.abort(command_abort),
    .bad_input(bad),.request_seq(pending_seq),.request_count(pending_count),
    .pixel_valid(data_good),.pixel_rgb(wr_data[23:0]),
    .busy(busy),.done(done),.error(error),.sequence_id(seq),.expected_count(expected),
    .received_count(received),.gray_sum(gray_sum),.red_count(red_count),.green_count(green_count),
    .yellow_count(yellow_count),.binary_ones(binary_ones),.gray_min(gray_min),.gray_max(gray_max));
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin rd_hit<=0;rd_data<=0;end
    else begin
        rd_hit<=rd_en && rd_addr>=12'h1d0 && rd_addr<=12'h1d2;
        if(rd_en)case(rd_addr)
            12'h1d0:rd_data<={received,seq,status,32'h52505331};
            12'h1d1:rd_data<={yellow_count,green_count,red_count,gray_sum};
            12'h1d2:rd_data<={pending_count,16'd0,gray_max,gray_min,binary_ones,expected};
            default:rd_data<=0;
        endcase
    end
end
endmodule
