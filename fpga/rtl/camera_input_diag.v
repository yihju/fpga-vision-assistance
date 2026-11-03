// Passive camera/SCCB diagnostics. Sensor counters cross as registered Gray;
// the changing, multi-field SCCB result crosses through a request/ack snapshot.
module camera_input_diag (
    input cam_pclk, input cam_rst_n, input cam_vsync, input cam_href,
    input cam_data7, input cam_sda, input cam_scl,
    input cfg_clk, input cfg_rst_n, input cfg_done,
    input [15:0] cfg_ack_count, input [15:0] cfg_nack_count,
    input [15:0] cfg_fail_address, input [3:0] cfg_ack_bits,
    input [8:0] cfg_index,
    input mon_clk, input mon_rst_n,
    output [255:0] diag_data
);
(* async_reg = "true" *) reg [1:0] sensor_reset_pipe;
reg [31:0] pixel_count, byte_count;
reg [15:0] frame_count;
reg [31:0] pixel_gray, byte_gray;
reg [15:0] frame_gray;
reg vsync_prev;
wire [31:0] pixel_next = pixel_count + 1'b1;
wire [31:0] byte_next = byte_count + 1'b1;
wire [15:0] frame_next = frame_count + 1'b1;
always @(posedge cam_pclk or negedge cam_rst_n) begin
    if (!cam_rst_n) sensor_reset_pipe <= 0;
    else sensor_reset_pipe <= {sensor_reset_pipe[0], 1'b1};
end
always @(posedge cam_pclk or negedge cam_rst_n) begin
    if (!cam_rst_n) begin
        pixel_count <= 0; pixel_gray <= 0;
        byte_count <= 0; byte_gray <= 0;
        frame_count <= 0; frame_gray <= 0; vsync_prev <= 0;
    end else if (!sensor_reset_pipe[1]) begin
        pixel_count <= 0; pixel_gray <= 0;
        byte_count <= 0; byte_gray <= 0;
        frame_count <= 0; frame_gray <= 0; vsync_prev <= 0;
    end else begin
        pixel_count <= pixel_next;
        pixel_gray <= pixel_next ^ (pixel_next >> 1);
        if (cam_href) begin
            byte_count <= byte_next;
            byte_gray <= byte_next ^ (byte_next >> 1);
        end
        vsync_prev <= cam_vsync;
        if (cam_vsync && !vsync_prev) begin
            frame_count <= frame_next;
            frame_gray <= frame_next ^ (frame_next >> 1);
        end
    end
end

(* async_reg = "true" *) reg [31:0] pixel_gray_meta, pixel_gray_sync;
(* async_reg = "true" *) reg [31:0] byte_gray_meta, byte_gray_sync;
(* async_reg = "true" *) reg [15:0] frame_gray_meta, frame_gray_sync;
(* async_reg = "true" *) reg [5:0] pins_meta, pins_sync;
always @(posedge mon_clk or negedge mon_rst_n) begin
    if (!mon_rst_n) begin
        pixel_gray_meta <= 0; pixel_gray_sync <= 0;
        byte_gray_meta <= 0; byte_gray_sync <= 0;
        frame_gray_meta <= 0; frame_gray_sync <= 0;
        pins_meta <= 0; pins_sync <= 0;
    end else begin
        pixel_gray_meta <= pixel_gray; pixel_gray_sync <= pixel_gray_meta;
        byte_gray_meta <= byte_gray; byte_gray_sync <= byte_gray_meta;
        frame_gray_meta <= frame_gray; frame_gray_sync <= frame_gray_meta;
        pins_meta <= {cam_data7,cam_vsync,cam_href,cam_scl,cam_sda,cam_rst_n};
        pins_sync <= pins_meta;
    end
end
function [31:0] gray32_to_binary;
    input [31:0] value;
    integer bit_index;
    begin
        gray32_to_binary[31] = value[31];
        for (bit_index=30; bit_index>=0; bit_index=bit_index-1)
            gray32_to_binary[bit_index] = gray32_to_binary[bit_index+1] ^ value[bit_index];
    end
endfunction
function [15:0] gray16_to_binary;
    input [15:0] value;
    integer bit_index;
    begin
        gray16_to_binary[15] = value[15];
        for (bit_index=14; bit_index>=0; bit_index=bit_index-1)
            gray16_to_binary[bit_index] = gray16_to_binary[bit_index+1] ^ value[bit_index];
    end
endfunction

reg request_toggle;
reg [3:0] refresh_count;
(* async_reg = "true" *) reg request_meta, request_sync;
reg cfg_ack_toggle;
reg [61:0] cfg_hold;
always @(posedge cfg_clk or negedge cfg_rst_n) begin
    if (!cfg_rst_n) begin
        request_meta <= 0; request_sync <= 0;
        cfg_ack_toggle <= 0; cfg_hold <= 0;
    end else begin
        request_meta <= request_toggle;
        request_sync <= request_meta;
        if (request_sync != cfg_ack_toggle) begin
            cfg_hold <= {cfg_done,cfg_index,cfg_ack_bits,cfg_fail_address,
                         cfg_nack_count,cfg_ack_count};
            cfg_ack_toggle <= request_sync;
        end
    end
end
(* async_reg = "true" *) reg cfg_ack_meta, cfg_ack_sync;
reg cfg_ack_seen;
reg [61:0] cfg_snapshot;
reg [31:0] snapshot_count;
always @(posedge mon_clk or negedge mon_rst_n) begin
    if (!mon_rst_n) begin
        request_toggle <= 0; refresh_count <= 0;
        cfg_ack_meta <= 0; cfg_ack_sync <= 0; cfg_ack_seen <= 0;
        cfg_snapshot <= 0; snapshot_count <= 0;
    end else begin
        cfg_ack_meta <= cfg_ack_toggle;
        cfg_ack_sync <= cfg_ack_meta;
        if (request_toggle == cfg_ack_sync) begin
            refresh_count <= refresh_count + 1'b1;
            if (&refresh_count) request_toggle <= !request_toggle;
        end
        if (cfg_ack_sync != cfg_ack_seen) begin
            cfg_snapshot <= cfg_hold;
            cfg_ack_seen <= cfg_ack_sync;
            snapshot_count <= snapshot_count + 1'b1;
        end
    end
end
wire configured_without_nack = cfg_snapshot[61] &&
    cfg_snapshot[15:0] != 0 && cfg_snapshot[31:16] == 0;
wire [31:0] status = {24'b0,pins_sync[5:1],configured_without_nack,
                       cfg_snapshot[61],pins_sync[0]};
assign diag_data = {
    32'd0, snapshot_count, gray32_to_binary(byte_gray_sync),
    16'd0, gray16_to_binary(frame_gray_sync),
    gray32_to_binary(pixel_gray_sync),
    3'd0,cfg_snapshot[60:52],cfg_snapshot[51:48],cfg_snapshot[47:32],
    cfg_snapshot[31:16],cfg_snapshot[15:0],status
};
endmodule
