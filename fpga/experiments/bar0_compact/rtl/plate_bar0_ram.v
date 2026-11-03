// 8KiB BAR0 backing store, not the frame DMA payload cache.
// One enabled-read cycle and independent byte write enables, like the
// vendor store. No array reset: resetting PCIe does not clear RAM contents.
module plate_bar0_ram (
    input [127:0] wr_data,input [8:0] wr_addr,input wr_en,
    input [15:0] wr_byte_en,input wr_clk,input wr_rst,
    input [8:0] rd_addr,output reg [127:0] rd_data,
    input rd_clk,input rd_clk_en,input rd_rst
);
reg [127:0] mem [0:511];
integer i;
initial begin
    for(i=0;i<512;i=i+1) mem[i]=128'd0;
end
genvar b;
generate for(b=0;b<16;b=b+1) begin: BYTE
    always @(posedge wr_clk)
        if(wr_en && wr_byte_en[b] && !wr_rst)
            mem[wr_addr][b*8+:8]<=wr_data[b*8+:8];
end endgenerate
always @(posedge rd_clk or posedge rd_rst) begin
    if(rd_rst) rd_data<=128'd0;
    else if(rd_clk_en) rd_data<=mem[rd_addr];
end
endmodule
