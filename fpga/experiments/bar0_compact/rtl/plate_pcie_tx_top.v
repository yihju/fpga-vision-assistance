// Stage2 keeps the known-good Pango BAR-read completion path. Frame MWr goes
// through the independent ordered batch engine, not the legacy DMA controller.
module plate_pcie_tx_top #(parameter ADDR_WIDTH=12)(
    input clk,input rst_n,
    input [7:0] cfg_bus,input [4:0] cfg_dev,input [2:0] cfg_mps,
    input [2:0] mrd_tc,input [2:0] mrd_attr,input [9:0] mrd_length,
    input [15:0] mrd_id,input [7:0] mrd_tag,input [63:0] mrd_addr,
    input cpld_req_valid,output cpld_req_ready,output cpld_tx_ready,
    output bar_rd_en,output [ADDR_WIDTH-1:0] bar_rd_addr,input [127:0] bar_rd_data,
    output axis_valid,input axis_ready,output [127:0] axis_data,output axis_last,output axis_user
);
wire rd_en,hold,tlp,start,last;
wire [9:0] rd_length;
wire [63:0] rd_addr;
wire [127:0] data;
wire [ADDR_WIDTH-1:0] raw_bar_rd_addr;
// Hard IP has already selected the BAR. Normalize its 8KiB host window,
// retaining all nine legal block-offset bits, including 0xa00 vs 0x200.
assign bar_rd_addr=raw_bar_rd_addr & 12'h1ff;
ips2l_pcie_dma_tx_cpld_rd_ctrl #(.ADDR_WIDTH(ADDR_WIDTH)) read_ctrl (
    .clk(clk),.rst_n(rst_n),.i_rd_en(rd_en),.i_rd_length(rd_length),.i_rd_addr(rd_addr),
    .i_cpld_tx_hold(hold),.i_cpld_tlp_tx(tlp),.o_gen_tlp_start(start),.o_rd_data(data),
    .o_last_data(last),.o_bar_rd_clk_en(bar_rd_en),.o_bar_rd_addr(raw_bar_rd_addr),.i_bar_rd_data(bar_rd_data)
);
ips2l_pcie_dma_cpld_tx_ctrl tx_ctrl (
    .clk(clk),.rst_n(rst_n),.i_cfg_pbus_num(cfg_bus),.i_cfg_pbus_dev_num(cfg_dev),
    .i_cfg_max_payload_size(cfg_mps),.i_mrd_tc(mrd_tc),.i_mrd_attr(mrd_attr),
    .i_mrd_length(mrd_length),.i_mrd_id(mrd_id),.i_mrd_tag(mrd_tag),.i_mrd_addr(mrd_addr),
    .i_cpld_req_vld(cpld_req_valid),.o_cpld_req_rdy(cpld_req_ready),.o_cpld_tx_rdy(cpld_tx_ready),
    .o_rd_en(rd_en),.o_rd_length(rd_length),.o_rd_addr(rd_addr),.o_cpld_tx_hold(hold),
    .o_cpld_tlp_tx(tlp),.i_gen_tlp_start(start),.i_rd_data(data),.i_last_data(last),
    .i_axis_slave0_trdy(axis_ready),.o_axis_slave0_tvld(axis_valid),
    .o_axis_slave0_tdata(axis_data),.o_axis_slave0_tlast(axis_last),.o_axis_slave0_tuser(axis_user)
);
endmodule
