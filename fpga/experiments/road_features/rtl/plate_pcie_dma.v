//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019 PANGO MICROSYSTEMS, INC
// ALL RIGHTS REVERVED.
//
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//
//////////////////////////////////////////////////////////////////////////////
//
// Library:
// Filename:ips2l_pcie_dma.v
//////////////////////////////////////////////////////////////////////////////
module plate_pcie_dma #(
    parameter                           DEVICE_TYPE = 3'd0      ,   //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    parameter   integer                 AXIS_SLAVE_NUM = 3      ,
    parameter                           ADDR_WIDTH  = 4'd12,
    parameter MSI_CAPABLE=1

)(
    output wire adas_led_dout,
    input wire led_ref_clk,
    input wire roi_key_n,
    input                               clk                     ,            //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,

    input           [7:0]               i_cfg_pbus_num          ,
    input           [4:0]               i_cfg_pbus_dev_num      ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    input           [2:0]               i_cfg_max_payload_size  ,
    //**********************************************************************
    //axis master interface
    input                               i_axis_master_tvld      ,
    output  wire                        o_axis_master_trdy      ,
    input           [127:0]             i_axis_master_tdata     ,
    input           [3:0]               i_axis_master_tkeep     ,
    input                               i_axis_master_tlast     ,
    input           [7:0]               i_axis_master_tuser     ,
    output  wire    [2:0]               o_trgt1_radm_pkt_halt   ,
//    input           [5:0]               i_radm_grant_tlp_type   ,
    //**********************************************************************
    //axis_slave0 interface
    input                               i_axis_slave0_trdy      ,
    output  wire                        o_axis_slave0_tvld      ,
    output  wire    [127:0]             o_axis_slave0_tdata     ,
    output  wire                        o_axis_slave0_tlast     ,
    output  wire                        o_axis_slave0_tuser     ,
    //axis_slave1 interface
    input                               i_axis_slave1_trdy      ,
    output  wire                        o_axis_slave1_tvld      ,
    output  wire    [127:0]             o_axis_slave1_tdata     ,
    output  wire                        o_axis_slave1_tlast     ,
    output  wire                        o_axis_slave1_tuser     ,
    //axis_slave2 interface
    input                               i_axis_slave2_trdy      ,
    output  wire                        o_axis_slave2_tvld      ,
    output  wire    [127:0]             o_axis_slave2_tdata     ,
    output  wire                        o_axis_slave2_tlast     ,
    output  wire                        o_axis_slave2_tuser     ,
    //**********************************************************************
    //credit interface
    //from pcie
    input                               i_cfg_ido_req_en        ,
    input                               i_cfg_ido_cpl_en        ,
    input           [7:0]               i_xadm_ph_cdts          ,
    input           [11:0]              i_xadm_pd_cdts          ,
    input           [7:0]               i_xadm_nph_cdts         ,
    input           [11:0]              i_xadm_npd_cdts         ,
    input           [7:0]               i_xadm_cplh_cdts        ,
    input           [11:0]              i_xadm_cpld_cdts        ,
    //**********************************************************************
    //apb interface
    input                               i_apb_psel              ,
    input           [8:0]               i_apb_paddr             ,
    input           [31:0]              i_apb_pwdata            ,
    input           [3:0]               i_apb_pstrb             ,
    input                               i_apb_pwrite            ,
    input                               i_apb_penable           ,
    output  wire                        o_apb_prdy              ,
    output  wire    [31:0]              o_apb_prdata            ,
    //cross_4kb_boundary
    output  wire                        o_cross_4kb_boundary    ,

    //**********************************************************************
    // dma write user interface(FPGA-->PC)
    output                            o_dma_write_data_req        ,
    output        [ADDR_WIDTH-1:0]    o_dma_write_addr            ,
    input         [127:0]             i_dma_write_data            ,

    //**********************************************************************
    // dma read user interface(PC-->FPGA)
    input                             i_dma_read_data_req        ,
    input         [ADDR_WIDTH-1:0]    i_dma_read_addr            ,
    output        [127:0]             o_dma_read_data            ,
    // Passive camera status; reserved diagnostic windows override RAM reads.
    input         [639:0]             i_camera_diagnostic,
    input [511:0] i_board_camera_diagnostic,
    input msi_enabled, input msi_grant, output msi_req,
    output source_start,input source_started,input source_idle,
    output source_abort,output source_retire,
    input [127:0] source_seq,input [255:0] source_tick,
    input [63:0] acquire_tick,input [3:0] repeat_mask,
    input pixel_valid,output pixel_ready,input [127:0] pixel_data

    //debug
    //output  wire    [159:0]             o_dbg_bus
);


wire [2:0] mrd_tc,mrd_attr;
wire [9:0] mrd_length;
wire [15:0] mrd_id;
wire [7:0] mrd_tag,cpld_tag;
wire [63:0] mrd_addr,dma_check_result;
wire cpld_req_vld,cpld_req_rdy,cpld_tx_rdy,cpld_rcv;
wire tag_full=1'b0,tx_restart=1'b0;
wire bar0_rd_clk_en,bar1_wr_en,new_rd_hit;
wire [ADDR_WIDTH-1:0] bar0_rd_addr,bar1_wr_addr;
wire [127:0] bar0_rd_data,bar0_ram_rd_data,diag_rd_data,new_rd_data,bar1_wr_data;
wire [15:0] bar1_wr_byte_en;
wire batch_busy,batch_error;
wire led_rd_hit;wire[127:0] led_rd_data;
wire roi_rd_hit;wire [127:0] roi_rd_data;
wire road_rd_hit;wire [127:0] road_rd_data;
assign bar0_rd_data=road_rd_hit ? road_rd_data : roi_rd_hit ? roi_rd_data : led_rd_hit ? led_rd_data : new_rd_hit ? new_rd_data : diag_rd_data;
road_features_bar road_features(.clk(clk),.rst_n(rst_n),
 .wr_en(bar1_wr_en),.wr_addr(bar1_wr_addr),.wr_data(bar1_wr_data),.wr_be(bar1_wr_byte_en),
 .rd_en(bar0_rd_clk_en),.rd_addr(bar0_rd_addr),.rd_hit(road_rd_hit),.rd_data(road_rd_data));
roi_stats_bar roi_features(.clk(clk),.rst_n(rst_n),.key_n(roi_key_n),
    .wr_en(bar1_wr_en),.wr_addr(bar1_wr_addr),.wr_data(bar1_wr_data),.wr_be(bar1_wr_byte_en),
    .rd_en(bar0_rd_clk_en),.rd_addr(bar0_rd_addr),.rd_hit(roi_rd_hit),.rd_data(roi_rd_data));
adas_led_bar led_control(.clk(clk),.led_clk(led_ref_clk),.rst_n(rst_n),.wr_en(bar1_wr_en),.wr_addr(bar1_wr_addr),
    .wr_data(bar1_wr_data),.wr_be(bar1_wr_byte_en),.rd_en(bar0_rd_clk_en),.rd_addr(bar0_rd_addr),
    .rd_hit(led_rd_hit),.rd_data(led_rd_data),.led_dout(adas_led_dout));
assign o_dma_write_data_req=0;
assign o_dma_write_addr=0;
assign o_apb_prdy=1;
assign o_apb_prdata=0;
assign o_cross_4kb_boundary=0;
assign o_axis_slave1_tvld=0;
assign o_axis_slave1_tdata=0;
assign o_axis_slave1_tlast=0;
assign o_axis_slave1_tuser=0;
camera_diag_bar_mux diag_mux (
    .clk(clk),.rst_n(rst_n),.rd_en(bar0_rd_clk_en),.rd_addr(bar0_rd_addr),
    .ram_data(bar0_ram_rd_data),.diagnostic(i_camera_diagnostic),
    .board_diagnostic(i_board_camera_diagnostic),.rd_data(diag_rd_data)
);
plate_pcie_rx_top #(
    .DEVICE_TYPE                (DEVICE_TYPE                ),  //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 )
)
u_ips2l_pcie_dma_rx_top
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    //**********************************************************************
    //axis master interface
    .i_axis_master_tvld         (i_axis_master_tvld         ),
    .o_axis_master_trdy         (o_axis_master_trdy         ),
    .i_axis_master_tdata        (i_axis_master_tdata        ),
    .i_axis_master_tkeep        (i_axis_master_tkeep        ),
    .i_axis_master_tlast        (i_axis_master_tlast        ),
    .i_axis_master_tuser        (i_axis_master_tuser        ),
    .o_trgt1_radm_pkt_halt      (o_trgt1_radm_pkt_halt      ),
//    .i_radm_grant_tlp_type      (i_radm_grant_tlp_type      ),
    //**********************************************************************
    //bar0 rd interface
    .i_bar0_rd_clk_en           (bar0_rd_clk_en             ),
    .i_bar0_rd_addr             (bar0_rd_addr               ),
    .o_bar0_rd_data             (bar0_ram_rd_data           ),
    //bar2 rd interface
    .i_bar2_rd_clk_en           (i_dma_read_data_req        ),
    .i_bar2_rd_addr             (i_dma_read_addr            ),
    .o_bar2_rd_data             (o_dma_read_data            ),
    //bar1 wr interface
    .o_bar1_wr_en               (bar1_wr_en                 ),
    .o_bar1_wr_addr             (bar1_wr_addr               ),
    .o_bar1_wr_data             (bar1_wr_data               ),
    .o_bar1_wr_byte_en          (bar1_wr_byte_en            ),
    //**********************************************************************
    //to tx top
    //req rcv
    .o_mrd_tc                   (mrd_tc                     ),
    .o_mrd_attr                 (mrd_attr                   ),
    .o_mrd_length               (mrd_length                 ),
    .o_mrd_id                   (mrd_id                     ),
    .o_mrd_tag                  (mrd_tag                    ),
    .o_mrd_addr                 (mrd_addr                   ),

    .o_cpld_req_vld             (cpld_req_vld               ),
    .i_cpld_req_rdy             (cpld_req_rdy               ),
    .i_cpld_tx_rdy              (cpld_tx_rdy                ),
    //cpld rcv
    .o_cpld_rcv                 (cpld_rcv                   ),
    .o_cpld_tag                 (cpld_tag                   ),
    .i_tag_full                 (tag_full                   ),
    //rst tlp cnt
    .o_dma_check_result         (dma_check_result           ),
    .i_tx_restart               (tx_restart                 )

    //.o_dbg_bus_rx_ctrl          (dbg_bus_rx_ctrl            )
);


plate_pcie_tx_top #(.ADDR_WIDTH(ADDR_WIDTH)) cpld_tx (
    .clk(clk),.rst_n(rst_n),.cfg_bus(i_cfg_pbus_num),.cfg_dev(i_cfg_pbus_dev_num),
    .cfg_mps(i_cfg_max_payload_size),.mrd_tc(mrd_tc),.mrd_attr(mrd_attr),
    .mrd_length(mrd_length),.mrd_id(mrd_id),.mrd_tag(mrd_tag),.mrd_addr(mrd_addr),
    .cpld_req_valid(cpld_req_vld),.cpld_req_ready(cpld_req_rdy),.cpld_tx_ready(cpld_tx_rdy),
    .bar_rd_en(bar0_rd_clk_en),.bar_rd_addr(bar0_rd_addr),.bar_rd_data(bar0_rd_data),
    .axis_valid(o_axis_slave0_tvld),.axis_ready(i_axis_slave0_trdy),
    .axis_data(o_axis_slave0_tdata),.axis_last(o_axis_slave0_tlast),.axis_user(o_axis_slave0_tuser)
);
plate_dma_engine #(.MSI_CAPABLE(MSI_CAPABLE)) engine (
    .clk(clk),.rst_n(rst_n),.wr_en(bar1_wr_en),.wr_addr(bar1_wr_addr),
    .wr_data(bar1_wr_data),.wr_be(bar1_wr_byte_en),
    .rd_en(bar0_rd_clk_en),.rd_addr(bar0_rd_addr),.rd_hit(new_rd_hit),.rd_data(new_rd_data),
    .cfg_mps(i_cfg_max_payload_size),.requester_id({i_cfg_pbus_num,i_cfg_pbus_dev_num,3'b0}),
    .msi_enabled(msi_enabled),.msi_grant(msi_grant),.msi_req(msi_req),
    .stop_external(1'b0),.busy(batch_busy),.command_error(batch_error),
    .source_start(source_start),.source_started(source_started),.source_idle(source_idle),
    .source_abort(source_abort),.source_retire(source_retire),
    .source_seq(source_seq),.source_tick(source_tick),.acquire_tick(acquire_tick),.repeat_mask(repeat_mask),
    .pixel_valid(pixel_valid),.pixel_ready(pixel_ready),.pixel_data(pixel_data),
    .axis_valid(o_axis_slave2_tvld),.axis_ready(i_axis_slave2_trdy),
    .axis_data(o_axis_slave2_tdata),.axis_last(o_axis_slave2_tlast),.axis_user(o_axis_slave2_tuser)
);
endmodule

