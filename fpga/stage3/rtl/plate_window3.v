// Two-line elastic 3x3 window. Only complete support domains are emitted.
// RAM contents need not be reset: first two new rows overwrite stale data.
module plate_window3 #(
 parameter BITS=8,WIDTH=1280,HEIGHT=720,ORIGIN=0
)(
 input clk,input rst,input start,input cancel,
 input in_valid,output in_ready,input [BITS-1:0] in_data,input [10:0] in_x,input [9:0] in_y,
 output reg out_valid,input out_ready,output reg [9*BITS-1:0] window,
 output reg [10:0] out_x,output reg [9:0] out_y,output reg error
);
reg [BITS-1:0] row1[0:WIDTH-1],row2[0:WIDTH-1];
reg [BITS-1:0] a0,a1,b0,b1,c0,c1;
reg [10:0] col;reg [9:0] row;reg active;
reg pending;
reg [BITS-1:0] row1_read,row2_read,saved_data;
reg [10:0] saved_x;reg [9:0] saved_y;
wire advance=!out_valid||out_ready;
wire accept=in_valid&&in_ready;
wire coordinate_ok=in_x==col+ORIGIN && in_y==row+ORIGIN;
wire apply=pending&&advance&&!rst&&!start&&!cancel;
assign in_ready=advance&&active&&!pending&&!rst&&!start&&!cancel&&!error;
// Separate, unreset synchronous RAM ports. READ_FIRST semantics at the
// accepted address preserve the previous row; APPLY occurs the next cycle.
always @(posedge clk)begin
 if(accept&&coordinate_ok)begin
  row1_read<=row1[col];row2_read<=row2[col];row1[col]<=in_data;
 end
 if(apply)row2[col]<=row1_read;
end
always @(posedge clk)begin
 if(rst||cancel||start)begin
  out_valid<=0;window<=0;out_x<=0;out_y<=0;error<=0;
  col<=0;row<=0;a0<=0;a1<=0;b0<=0;b1<=0;c0<=0;c1<=0;
  active<=start&&!rst&&!cancel;pending<=0;saved_data<=0;saved_x<=0;saved_y<=0;
 end else if(advance)begin
  out_valid<=0;
  if(accept)begin
   if(!coordinate_ok)begin error<=1;active<=0;end
   else begin pending<=1;saved_data<=in_data;saved_x<=in_x;saved_y<=in_y;end
  end else if(apply)begin
    pending<=0;
    if(col==0)begin
     a0<=0;a1<=row2_read;b0<=0;b1<=row1_read;c0<=0;c1<=saved_data;
    end else begin
     a0<=a1;a1<=row2_read;b0<=b1;b1<=row1_read;c0<=c1;c1<=saved_data;
    end
    if(row>=2&&col>=2)begin
     // index0 top-left, index8 bottom-right.
     window<={saved_data,c1,c0,row1_read,b1,b0,row2_read,a1,a0};
     out_x<=saved_x-1'b1;out_y<=saved_y-1'b1;out_valid<=1;
    end
    if(col==WIDTH-1)begin
     col<=0;
     if(row==HEIGHT-1)active<=0;else row<=row+1'b1;
    end else col<=col+1'b1;
  end
 end
end
endmodule
