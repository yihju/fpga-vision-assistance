// Bounded RGB888 ROI, raster order, one request at a time. Output support is
// x=2..WIDTH-3,y=2..HEIGHT-3: 3x3 Gaussian followed by full |Gx|+|Gy| Sobel.
// All fields travel together through two elastic windows; no latency guessing.
// WIDTH/HEIGHT are static 5..64. This kernel is independent of old RPS1 BARs.
module road_roi_features #(
 parameter WIDTH=64,HEIGHT=64,EDGE_THRESHOLD=96
)(
 input clk,input rst,input start,input cancel,input [31:0] request_seq,
 input in_valid,output in_ready,input [23:0] pixel_rgb,input [10:0] in_x,input [9:0] in_y,
 output out_valid,input out_ready,output reg [31:0] sequence_id,
 output [10:0] out_x,output [9:0] out_y,
 output [7:0] gray,output [7:0] smoothed,output [10:0] edge_mag,
 output binary,output red,output green,output yellow,output white,
 output reg busy,output reg done,output error,
 output reg [31:0] result_count,output reg [31:0] gray_sum,output reg [31:0] edge_count,
 input [5:0] projection_index,output [15:0] row_projection,output [15:0] column_projection
);
localparam TOTAL=(WIDTH-4)*(HEIGHT-4);
reg fault;
wire new_request=start&&!busy;
wire bad_start=start&&busy;
wire stopped=cancel||bad_start||(fault&&!new_request);
wire [7:0] r=pixel_rgb[23:16],g=pixel_rgb[15:8],b=pixel_rgb[7:0];
wire [17:0] rr={10'd0,r},gg={10'd0,g},bb={10'd0,b};
wire [17:0] weighted=(rr<<6)+(rr<<3)+(rr<<2)+rr+
 (gg<<7)+(gg<<4)+(gg<<2)+(gg<<1)+(bb<<4)+(bb<<3)+(bb<<2)+bb;
wire [7:0] current_gray=weighted[15:8];
wire [8:0] er={1'b0,r},eg={1'b0,g},eb={1'b0,b};
wire current_red=er>=85&&er>=eg+9'd32&&er>=eb+9'd32;
wire current_green=eg>=85&&eg>=er+9'd20&&eg>=eb+9'd10;
wire current_yellow=er>=85&&eg>=85&&er>=eb+9'd32&&eg>=eb+9'd32;
wire [7:0] maximum=(r>g)?((r>b)?r:b):((g>b)?g:b);
wire [7:0] minimum=(r<g)?((r<b)?r:b):((g<b)?g:b);
wire current_white=minimum>=160&&({1'b0,maximum}-{1'b0,minimum})<=40;
wire [107:0] first_window;
wire first_valid,first_ready,first_input_ready,first_error;
wire [10:0] first_x;wire [9:0] first_y;
assign in_ready=first_input_ready&&busy&&!error&&!start&&!cancel;
plate_window3 #(.BITS(12),.WIDTH(WIDTH),.HEIGHT(HEIGHT)) first(
 .clk(clk),.rst(rst),.start(new_request),.cancel(stopped),
 .in_valid(in_valid&&busy&&!error&&!start&&!cancel),.in_ready(first_input_ready),
 .in_data({current_gray,current_red,current_green,current_yellow,current_white}),.in_x(in_x),.in_y(in_y),
 .out_valid(first_valid),.out_ready(first_ready),.window(first_window),.out_x(first_x),.out_y(first_y),.error(first_error));
wire [11:0] blur_sum={4'd0,first_window[4+:8]}+({4'd0,first_window[16+:8]}<<1)+{4'd0,first_window[28+:8]}
 +({4'd0,first_window[40+:8]}<<1)+({4'd0,first_window[52+:8]}<<2)+({4'd0,first_window[64+:8]}<<1)
 +{4'd0,first_window[76+:8]}+({4'd0,first_window[88+:8]}<<1)+{4'd0,first_window[100+:8]}+12'd8;
wire [19:0] blur_pixel={first_window[52+:8],blur_sum[11:4],first_window[48+:4]};
wire [179:0] second_window;
wire second_valid,second_ready,second_error;
wire [10:0] second_x;wire [9:0] second_y;
plate_window3 #(.BITS(20),.WIDTH(WIDTH-2),.HEIGHT(HEIGHT-2),.ORIGIN(1)) second(
 .clk(clk),.rst(rst),.start(new_request),.cancel(stopped),
 .in_valid(first_valid),.in_ready(first_ready),.in_data(blur_pixel),.in_x(first_x),.in_y(first_y),
 .out_valid(second_valid),.out_ready(second_ready),.window(second_window),.out_x(second_x),.out_y(second_y),.error(second_error));
wire [10:0] lhs={3'd0,second_window[4+:8]}+({3'd0,second_window[64+:8]}<<1)+{3'd0,second_window[124+:8]};
wire [10:0] rhs={3'd0,second_window[44+:8]}+({3'd0,second_window[104+:8]}<<1)+{3'd0,second_window[164+:8]};
wire [10:0] upper={3'd0,second_window[4+:8]}+({3'd0,second_window[24+:8]}<<1)+{3'd0,second_window[44+:8]};
wire [10:0] lower={3'd0,second_window[124+:8]}+({3'd0,second_window[144+:8]}<<1)+{3'd0,second_window[164+:8]};
wire [10:0] gx=(lhs>=rhs)?lhs-rhs:rhs-lhs;
wire [10:0] gy=(upper>=lower)?upper-lower:lower-upper;
assign gray=second_window[92+:8];
assign smoothed=second_window[84+:8];
assign {red,green,yellow,white}=second_window[80+:4];
assign binary=smoothed>=128;
assign edge_mag=gx+gy;
assign out_x=second_x;assign out_y=second_y;
assign error=fault||first_error||second_error;
assign out_valid=second_valid&&busy&&!error&&!start&&!cancel;
assign second_ready=out_ready&&busy&&!error&&!start&&!cancel;
reg [15:0] rows[0:HEIGHT-1];reg [15:0] columns[0:WIDTH-1];
assign row_projection=done&&projection_index<HEIGHT?rows[projection_index]:16'd0;
assign column_projection=done&&projection_index<WIDTH?columns[projection_index]:16'd0;
integer i;
always @(posedge clk)begin
 if(rst||cancel||start)begin
  busy<=new_request&&!rst&&!cancel;done<=0;fault<=bad_start&&!rst&&!cancel;
  result_count<=0;gray_sum<=0;edge_count<=0;
  if(rst)sequence_id<=0;else if(new_request)sequence_id<=request_seq;
  for(i=0;i<HEIGHT;i=i+1)rows[i]<=0;
  for(i=0;i<WIDTH;i=i+1)columns[i]<=0;
 end else if(first_error||second_error)begin busy<=0;done<=0;fault<=1;end
 else if(out_valid&&out_ready)begin
  result_count<=result_count+1'b1;
  gray_sum<=gray_sum+{24'd0,gray};
  if(edge_mag>=EDGE_THRESHOLD)edge_count<=edge_count+1'b1;
  if(white||yellow)begin rows[second_y]<=rows[second_y]+1'b1;columns[second_x]<=columns[second_x]+1'b1;end
  if(result_count==TOTAL-1)begin done<=1;busy<=0;end
 end
end
endmodule
