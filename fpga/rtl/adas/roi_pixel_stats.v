// One bounded ROI of actual RGB888 pixels. All inputs/results share clk.
// Gray = floor((77R+150G+29B)/256); binary threshold is gray >= 128.
// Dominant-color counters are features, not a stand-alone traffic permission.
module roi_pixel_stats #(
    parameter [31:0] MAX_PIXELS=4096
)(
    input clk,input rst_n,input start,input abort,input bad_input,
    input [31:0] request_seq,input [31:0] request_count,
    input pixel_valid,input [23:0] pixel_rgb,
    output reg busy,output reg done,output reg error,
    output reg [31:0] sequence_id,output reg [31:0] expected_count,
    output reg [31:0] received_count,output reg [31:0] gray_sum,
    output reg [31:0] red_count,output reg [31:0] green_count,
    output reg [31:0] yellow_count,output reg [31:0] binary_ones,
    output reg [7:0] gray_min,output reg [7:0] gray_max
);
wire [7:0] r=pixel_rgb[23:16],g=pixel_rgb[15:8],b=pixel_rgb[7:0];
wire [17:0] re={10'd0,r},ge={10'd0,g},bl={10'd0,b};
wire [17:0] weighted=(re<<6)+(re<<3)+(re<<2)+re+
                     (ge<<7)+(ge<<4)+(ge<<2)+(ge<<1)+
                     (bl<<4)+(bl<<3)+(bl<<2)+bl;
wire [7:0] gray=weighted[15:8];
wire [8:0] rr={1'b0,r},gg={1'b0,g},bb={1'b0,b};
wire is_red=rr>=85 && rr>=gg+9'd32 && rr>=bb+9'd32;
wire is_green=gg>=85 && gg>=rr+9'd20 && gg>=bb+9'd10;
wire is_yellow=rr>=85 && gg>=85 && rr>=bb+9'd32 && gg>=bb+9'd32;
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        busy<=0;done<=0;error<=0;sequence_id<=0;expected_count<=0;received_count<=0;
        gray_sum<=0;red_count<=0;green_count<=0;yellow_count<=0;binary_ones<=0;
        gray_min<=255;gray_max<=0;
    end else if(abort)begin
        busy<=0;done<=0;error<=0;
    end else if(bad_input)begin
        busy<=0;done<=0;error<=1;
    end else if(start)begin
        if(busy || request_count==0 || request_count>MAX_PIXELS)begin
            busy<=0;done<=0;error<=1;
        end else begin
            busy<=1;done<=0;error<=0;sequence_id<=request_seq;
            expected_count<=request_count;received_count<=0;
            gray_sum<=0;red_count<=0;green_count<=0;yellow_count<=0;binary_ones<=0;
            gray_min<=255;gray_max<=0;
        end
    end else if(pixel_valid)begin
        if(!busy || received_count>=expected_count)begin
            busy<=0;done<=0;error<=1;
        end else begin
            received_count<=received_count+1'b1;
            gray_sum<=gray_sum+{24'd0,gray};
            red_count<=red_count+{31'd0,is_red};green_count<=green_count+{31'd0,is_green};
            yellow_count<=yellow_count+{31'd0,is_yellow};
            binary_ones<=binary_ones+{31'd0,gray>=8'd128};
            if(gray<gray_min)gray_min<=gray;
            if(gray>gray_max)gray_max<=gray;
            if(received_count+1'b1==expected_count)begin busy<=0;done<=1;end
        end
    end
end
endmodule
