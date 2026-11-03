`timescale 1ns/1ps
// Dual-clock 128-bit FIFO. Only Gray pointers cross clock domains; RAM words
// are read after the synchronized write pointer makes them visible. Both
// resets must assert for a pipeline reset; never reset one side per frame.
module plate_pixel_cdc #(parameter AW=10)(
    input wr_clk,input wr_rst,input wr_valid,output wr_ready,input [127:0] wr_data,
    input rd_clk,input rd_rst,output reg rd_valid,input rd_ready,output reg [127:0] rd_data,
    output rd_empty
);
reg [127:0] mem [0:(1<<AW)-1];
reg [AW:0] wbin,wgray,rbin,rgray;
(* ASYNC_REG="TRUE" *) reg [AW:0] rg1,rg2,wg1,wg2;
reg wfull;
wire [AW:0] wnext=wbin+{{AW{1'b0}},wr_valid && wr_ready};
wire [AW:0] wg_next=(wnext>>1)^wnext;
wire [AW:0] rnext=rbin+1'b1;
wire rempty=rgray==wg2;
assign wr_ready=!wfull && !wr_rst;
assign rd_empty=rempty && !rd_valid;
always @(posedge wr_clk or posedge wr_rst) begin
    if(wr_rst)begin wbin<=0;wgray<=0;rg1<=0;rg2<=0;wfull<=0;end
    else begin
        rg1<=rgray;rg2<=rg1;
        wbin<=wnext;wgray<=wg_next;
        wfull<=wg_next=={~rg2[AW:AW-1],rg2[AW-2:0]};
    end
end
// Keep RAM write out of the asynchronous-reset pointer process so synthesis
// can infer a block dual-port RAM, rather than resettable flip-flops.
always @(posedge wr_clk) if(wr_valid && wr_ready)mem[wbin[AW-1:0]]<=wr_data;
always @(posedge rd_clk or posedge rd_rst) begin
    if(rd_rst)begin rbin<=0;rgray<=0;wg1<=0;wg2<=0;rd_valid<=0;rd_data<=0;end
    else begin
        wg1<=wgray;wg2<=wg1;
        if(!rd_valid || rd_ready)begin
            if(rempty)rd_valid<=0;
            else begin
                rd_data<=mem[rbin[AW-1:0]];rd_valid<=1;
                rbin<=rnext;rgray<=(rnext>>1)^rnext;
            end
        end
    end
end
endmodule
