`include "plate_video_protocol.vh"
// PCIe-domain composition. Pixels and metadata use one ordered TX queue.
// Body fields are snapshotted once, never reread halfway through a STOP.
module plate_dma_engine #(parameter MSI_CAPABLE=1)(
    input clk, input rst_n,
    input wr_en, input [11:0] wr_addr, input [127:0] wr_data, input [15:0] wr_be,
    input rd_en, input [11:0] rd_addr, output rd_hit, output [127:0] rd_data,
    input [2:0] cfg_mps, input [15:0] requester_id,
    input msi_enabled, input msi_grant, output msi_req,
    input stop_external, output busy, output command_error,
    output source_start, input source_started, input source_idle,
    output source_abort, output source_retire,
    input [127:0] source_seq, input [255:0] source_tick,
    input [63:0] acquire_tick, input [3:0] repeat_mask,
    input pixel_valid, output pixel_ready, input [127:0] pixel_data,
    output axis_valid, input axis_ready, output [127:0] axis_data,
    output axis_last, output axis_user
);
wire cmd_valid,cmd_ready,bar_stop;
wire [63:0] cmd_addr,cmd_seq;
wire [31:0] cmd_cookie,cmd_epoch,cmd_slot,cmd_bytes,error_count;
wire stop=bar_stop || stop_external;
wire segment_valid,segment_ready,segment_done,segment_aborted,irq_req;
wire [63:0] segment_addr,record_seq;
wire [9:0] segment_dwords;
wire [1:0] segment_kind;
wire [31:0] completion_status,valid_bytes,record_cookie,record_epoch,record_slot;
reg [1:0] payload_kind;
reg [3:0] body_index;
reg [1023:0] record_body;
wire payload_ready;
wire payload_valid=payload_kind==0 ? pixel_valid : 1'b1;
wire [127:0] payload_data=payload_kind==0 ? pixel_data :
                        payload_kind==1 ? record_body[body_index*128+:128] : {96'd0,record_cookie};
assign pixel_ready=payload_kind==0 && payload_ready;
assign msi_req=MSI_CAPABLE && irq_req && msi_enabled;
plate_video_bar #(.MSI_CAPABLE(MSI_CAPABLE)) bar (
    .clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_addr(wr_addr),.wr_data(wr_data),.wr_be(wr_be),
    .rd_en(rd_en),.rd_addr(rd_addr),.rd_hit(rd_hit),.rd_data(rd_data),
    .dma_busy(busy),.msi_enabled(msi_enabled),.cfg_mps(cfg_mps),
    .last_seq(record_seq),.last_bytes(valid_bytes),.last_status(record_body[255:224]),
    .cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_addr(cmd_addr),.cmd_seq(cmd_seq),
    .cmd_cookie(cmd_cookie),.cmd_epoch(cmd_epoch),.cmd_slot(cmd_slot),.cmd_bytes(cmd_bytes),
    .stop(bar_stop),.error_count(error_count)
);
plate_dma_batch batch (
    .clk(clk),.rst_n(rst_n),.cmd_valid(cmd_valid),.cmd_ready(cmd_ready),.cmd_addr(cmd_addr),
    .cmd_cookie(cmd_cookie),.cmd_epoch(cmd_epoch),.cmd_slot(cmd_slot),.cmd_bytes(cmd_bytes),
    .cmd_seq(cmd_seq),.stop(stop),.source_start(source_start),.source_started(source_started),
    .source_idle(source_idle),.source_abort(source_abort),.source_retire(source_retire),
    .segment_valid(segment_valid),.segment_ready(segment_ready),.segment_addr(segment_addr),
    .segment_dwords(segment_dwords),.segment_kind(segment_kind),
    .segment_done(segment_done),.segment_aborted(segment_aborted),
    .completion_status(completion_status),.valid_bytes(valid_bytes),.record_cookie(record_cookie),
    .record_epoch(record_epoch),.record_slot(record_slot),.record_seq(record_seq),
    .irq_req(irq_req),.irq_grant(MSI_CAPABLE ? msi_enabled && msi_grant : 1'b1),
    .busy(busy),.command_error(command_error)
);
plate_mwr_tx_ctrl tx (
    .clk(clk),.rst_n(rst_n),.cfg_mps(cfg_mps),.requester_id(requester_id),
    .req_valid(segment_valid),.req_ready(segment_ready),.req_addr(segment_addr),
    .req_dwords(segment_dwords),.cancel(stop && payload_kind==0),
    .payload_valid(payload_valid),.payload_ready(payload_ready),.payload_data(payload_data),
    .axis_valid(axis_valid),.axis_ready(axis_ready),.axis_data(axis_data),
    .axis_last(axis_last),.axis_user(axis_user),
    .segment_done(segment_done),.segment_aborted(segment_aborted),.busy()
);
always @(posedge clk) begin
    if(!rst_n) begin payload_kind<=0;body_index<=0;record_body<=0;end
    else begin
        if(segment_valid && segment_ready) begin
            payload_kind<=segment_kind;body_index<=0;
            if(segment_kind==1)
                record_body<={32'd0,224'd0,acquire_tick,`PLATE_VIDEO_FORMAT,{28'd0,repeat_mask},
                    source_tick,source_seq,completion_status,valid_bytes,record_seq,
                    record_slot,record_epoch,`PLATE_VIDEO_MAJOR,`PLATE_VIDEO_MAGIC};
        end else if(payload_valid && payload_ready && payload_kind==1) body_index<=body_index+1'b1;
    end
end
endmodule
