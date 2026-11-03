`timescale 1ns / 1ps

// Keep the physical-camera to DDR-channel relationship explicit.  The
// reference design routed CAM3 to both channel 2 and channel 3; this module
// makes channel 3's ownership of the real fourth camera independently
// verifiable.
module camera_stream_router (
    input         cam1_pclk,
    input         cam1_vsync,
    input         cam1_de,
    input  [15:0] cam1_data,
    input         cam2_pclk,
    input         cam2_vsync,
    input         cam2_de,
    input  [15:0] cam2_data,
    input         cam3_pclk,
    input         cam3_vsync,
    input         cam3_de,
    input  [15:0] cam3_data,
    input         cam4_pclk,
    input         cam4_vsync,
    input         cam4_de,
    input  [15:0] cam4_data,

    output        ch0_pclk,
    output        ch0_vsync,
    output        ch0_de,
    output [15:0] ch0_data,
    output        ch1_pclk,
    output        ch1_vsync,
    output        ch1_de,
    output [15:0] ch1_data,
    output        ch2_pclk,
    output        ch2_vsync,
    output        ch2_de,
    output [15:0] ch2_data,
    output        ch3_pclk,
    output        ch3_vsync,
    output        ch3_de,
    output [15:0] ch3_data
);

assign ch0_pclk  = cam1_pclk;
assign ch0_vsync = cam1_vsync;
assign ch0_de    = cam1_de;
assign ch0_data  = cam1_data;

assign ch1_pclk  = cam2_pclk;
assign ch1_vsync = cam2_vsync;
assign ch1_de    = cam2_de;
assign ch1_data  = cam2_data;

assign ch2_pclk  = cam3_pclk;
assign ch2_vsync = cam3_vsync;
assign ch2_de    = cam3_de;
assign ch2_data  = cam3_data;

assign ch3_pclk  = cam4_pclk;
assign ch3_vsync = cam4_vsync;
assign ch3_de    = cam4_de;
assign ch3_data  = cam4_data;

endmodule
