// New non-overlapping feature window. Old LEDI4/RPS1/DMA BARs are untouched.
// BAR1: 0x280 BEGIN(0x52460001), PROCESS(...02), ABORT(...03);
// 0x290 pending sequence; 0x2a0 successive 00RRGGBB words; 0x2b0 result block
// index; 0x2b4 projection index. Addresses below are FULL 16-byte indices.
// BAR0: 0x1e00 magic/status/sequence/output_count; 0x1e10 input/statistics;
// 0x1e20 four packed pixel words; 0x1e30 row/column; 0x1e40 dimensions.
module road_features_bar #(parameter WIDTH=64,HEIGHT=64)(
 input clk,input rst_n,input wr_en,input [11:0] wr_addr,input [127:0] wr_data,input [15:0] wr_be,
 input rd_en,input [11:0] rd_addr,output reg rd_hit,output reg [127:0] rd_data
);
localparam PIXELS=WIDTH*HEIGHT,OUTPUTS=(WIDTH-4)*(HEIGHT-4),BLOCKS=(OUTPUTS+3)/4;
localparam [31:0] PIXELS32=PIXELS,WIDTH32=WIDTH,HEIGHT32=HEIGHT,INNER_WIDTH32=WIDTH-4,INNER_HEIGHT32=HEIGHT-4;
wire control=wr_en&&wr_addr==12'h028;
wire data_write=wr_en&&wr_addr==12'h02a;
wire word_valid=wr_be==16'h000f;
wire begin_command=control&&word_valid&&wr_data[31:0]==32'h52460001;
wire process_command=control&&word_valid&&wr_data[31:0]==32'h52460002;
wire abort_command=control&&word_valid&&wr_data[31:0]==32'h52460003;
reg [31:0] pending_seq,active_seq,uploaded,result_block,projection;
reg uploading,running,complete,fault,kick,core_cancel;
reg [12:0] sent;reg [1:0] phase;
reg [23:0] input_read,input_buffer;
reg [23:0] input_memory[0:PIXELS-1];
reg [31:0] bank0[0:BLOCKS-1],bank1[0:BLOCKS-1],bank2[0:BLOCKS-1],bank3[0:BLOCKS-1];
reg [31:0] result0,result1,result2,result3;
wire ready,out_valid,core_busy,core_done,core_error;
wire [31:0] seq,count,gray_sum,edge_count;
wire [10:0] ox,edge_mag;wire [9:0] oy;wire [7:0] gray,smoothed;
wire binary,red,green,yellow,white;wire [15:0] row_value,column_value;
wire input_valid=running&&phase==2&&sent<PIXELS;
wire [10:0] source_x=sent%WIDTH;
wire [9:0] source_y=sent/WIDTH;
wire [31:0] packed_pixel={gray,smoothed,edge_mag,binary,red,green,yellow,white};
wire bad_control=control&&!(begin_command||process_command||abort_command);
wire bad_pixel=data_write&&(!word_valid||wr_data[31:24]!=0||!uploading||uploaded>=PIXELS);
wire invalid=bad_control||bad_pixel||(begin_command&&running)||
 (process_command&&(!uploading||uploaded!=PIXELS||running));
road_roi_features #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) core(
 .clk(clk),.rst(!rst_n),.start(kick),.cancel(core_cancel),.request_seq(active_seq),
 .in_valid(input_valid),.in_ready(ready),.pixel_rgb(input_buffer),.in_x(source_x),.in_y(source_y),
 .out_valid(out_valid),.out_ready(1'b1),.sequence_id(seq),.out_x(ox),.out_y(oy),.gray(gray),.smoothed(smoothed),
 .edge_mag(edge_mag),.binary(binary),.red(red),.green(green),.yellow(yellow),.white(white),
 .busy(core_busy),.done(core_done),.error(core_error),.result_count(count),.gray_sum(gray_sum),.edge_count(edge_count),
 .projection_index(projection[5:0]),.row_projection(row_value),.column_projection(column_value));
// Independent unreset memory ports. BEGIN resets counters, not RAM; PROCESS
// requires a complete fresh upload before any old memory can be observed.
always @(posedge clk)begin
 if(rst_n&&data_write&&!invalid&&uploading)input_memory[uploaded]<=wr_data[23:0];
 if(rst_n&&running&&phase==0&&sent<PIXELS)input_read<=input_memory[sent];
 if(rst_n&&out_valid&&running)case(count[1:0])
  0:bank0[count>>2]<=packed_pixel;
  1:bank1[count>>2]<=packed_pixel;
  2:bank2[count>>2]<=packed_pixel;
  3:bank3[count>>2]<=packed_pixel;
 endcase
 if(result_block<BLOCKS)begin
  result0<=bank0[result_block];result1<=bank1[result_block];result2<=bank2[result_block];result3<=bank3[result_block];
 end else begin result0<=0;result1<=0;result2<=0;result3<=0;end
end
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
  pending_seq<=0;active_seq<=0;uploaded<=0;result_block<=0;projection<=0;
  uploading<=0;running<=0;complete<=0;fault<=0;kick<=0;core_cancel<=0;sent<=0;phase<=0;input_buffer<=0;
 end else begin
  kick<=0;
  // Pipeline the high-fanout cancellation tree. BAR visibility/running are
  // revoked immediately below; the isolated core clears on the next cycle.
  core_cancel<=abort_command||invalid||fault;
  if(wr_en&&wr_addr==12'h029&&word_valid&&!running)pending_seq<=wr_data[31:0];
  if(wr_en&&wr_addr==12'h02b)begin
   if(wr_be[3:0]==4'hf)result_block<=wr_data[31:0];
   if(wr_be[7:4]==4'hf)projection<=wr_data[63:32];
  end
  if(abort_command||invalid)begin
   uploading<=0;running<=0;complete<=0;fault<=invalid;kick<=0;phase<=0;uploaded<=0;sent<=0;
  end else if(begin_command)begin
   uploading<=1;running<=0;complete<=0;fault<=0;uploaded<=0;sent<=0;phase<=0;active_seq<=pending_seq;
  end else if(process_command)begin
   uploading<=0;running<=1;complete<=0;fault<=0;kick<=1;sent<=0;phase<=0;
  end else if(running)begin
   if(core_error)begin running<=0;complete<=0;fault<=1;end
   else if(core_done)begin running<=0;complete<=1;end
   else if(sent<PIXELS)case(phase)
    0:phase<=1;
    1:begin input_buffer<=input_read;phase<=2;end
    2:if(ready)begin sent<=sent+1'b1;phase<=0;end
   endcase
  end else if(data_write&&uploading)uploaded<=uploaded+1'b1;
 end
end
wire [31:0] status={29'd0,fault,complete,uploading||running};
wire visible=complete&&!fault;
wire [31:0] display0=visible&&result_block*4<OUTPUTS?result0:0;
wire [31:0] display1=visible&&result_block*4+1<OUTPUTS?result1:0;
wire [31:0] display2=visible&&result_block*4+2<OUTPUTS?result2:0;
wire [31:0] display3=visible&&result_block*4+3<OUTPUTS?result3:0;
always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin rd_hit<=0;rd_data<=0;end
 else begin
  rd_hit<=rd_en&&rd_addr>=12'h1e0&&rd_addr<=12'h1e4;
  if(rd_en)case(rd_addr)
   12'h1e0:rd_data<={visible?count:32'd0,active_seq,status,32'h52465431};
   12'h1e1:rd_data<={visible?edge_count:32'd0,visible?gray_sum:32'd0,uploaded,PIXELS32};
   12'h1e2:rd_data<={display3,display2,display1,display0};
   12'h1e3:rd_data<={64'd0,16'd0,visible&&projection<WIDTH?column_value:16'd0,16'd0,visible&&projection<HEIGHT?row_value:16'd0};
   12'h1e4:rd_data<={HEIGHT32,WIDTH32,INNER_HEIGHT32,INNER_WIDTH32};
   default:rd_data<=0;
  endcase
 end
end
endmodule
