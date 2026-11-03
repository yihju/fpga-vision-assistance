`timescale 1ns/1ps
// DDR-domain source. A channel request reads exactly one FIFO beat; response
// may be delayed. line_ready means FIFO NOT EMPTY, not the legacy 160-beat flag.
// At most one read is outstanding; output is held across PCIe backpressure.
// frame_ack is the actual DDR FIFO/address-latch ack with matching pool identity.
// Abort during acquisition must be acknowledged by the bridge (cancel unissued,
// drain issued). ddr_idle may only assert after all issued DDR reads have retired.
module plate_frame_source (
    input mclk, input mrst, input [63:0] mclk_tick,
    input start, input abort, output source_idle,
    output reg [3:0] frame_req, input [3:0] frame_ack,
    output frame_cancel,
    input [127:0] frame_source_seq, input [255:0] frame_source_tick,
    input [3:0] line_ready, input [511:0] channel_data,
    input [3:0] channel_data_valid, output reg [3:0] channel_data_req,
    input [3:0] ddr_idle,
    output [127:0] pixel_data, output pixel_valid, input pixel_ready,
    output pixel_last,
    output reg [127:0] source_seq, output reg [255:0] source_tick,
    output reg [63:0] acquire_tick, output reg [3:0] repeated_source_mask
);
    localparam IDLE=0, ACQUIRE=1, STREAM=2, DRAIN=3;
    reg [1:0] state;
    reg [3:0] acquired;
    reg cancelled, pending, held;
    reg [1:0] pending_channel;
    reg [127:0] held_data, previous_seq;
    reg previous_valid;
    reg [7:0] col;
    reg [9:0] row;
    wire [1:0] selected={row>=360, col>=80};
    wire last=(col==159 && row==719);
    assign source_idle=(state==IDLE);
    assign frame_cancel=cancelled || abort;
    assign pixel_data=held_data;
    assign pixel_valid=(state==STREAM) && held && !abort && !cancelled;
    assign pixel_last=pixel_valid && last;
    integer k;
    always @* begin
        channel_data_req=0;
        if(state==STREAM && !held && !pending && !abort && line_ready[selected])
            channel_data_req[selected]=1;
        else if(state==DRAIN && !pending) begin
            if(line_ready[0]) channel_data_req=1;
            else if(line_ready[1]) channel_data_req=2;
            else if(line_ready[2]) channel_data_req=4;
            else if(line_ready[3]) channel_data_req=8;
        end
    end
    always @(posedge mclk) begin
        if(mrst) begin
            state<=IDLE; frame_req<=0; acquired<=0; cancelled<=0;
            pending<=0; pending_channel<=0; held<=0; held_data<=0;
            col<=0; row<=0; source_seq<=0; source_tick<=0;
            acquire_tick<=0; repeated_source_mask<=0;
            previous_seq<=0; previous_valid<=0;
        end else begin
            if(state!=IDLE && abort) cancelled<=1;
            if(channel_data_req!=0) begin
                pending<=1;
                case(channel_data_req)
                    1: pending_channel<=0;
                    2: pending_channel<=1;
                    4: pending_channel<=2;
                    8: pending_channel<=3;
                endcase
            end
            if(pending && channel_data_valid[pending_channel]) begin
                pending<=0;
                if(state==STREAM && !abort && !cancelled) begin
                    held_data<=channel_data[pending_channel*128 +: 128];
                    held<=1;
                end
            end
            case(state)
                IDLE: if(start && !abort) begin
                    state<=ACQUIRE; frame_req<=15; acquired<=0;
                    cancelled<=0; held<=0; pending<=0; col<=0; row<=0;
                end
                ACQUIRE: begin
                    frame_req<=frame_req & ~frame_ack;
                    acquired<=acquired | frame_ack;
                    for(k=0;k<4;k=k+1) if(frame_ack[k] && !acquired[k]) begin
                        source_seq[k*32 +:32]<=frame_source_seq[k*32 +:32];
                        source_tick[k*64 +:64]<=frame_source_tick[k*64 +:64];
                        repeated_source_mask[k]<=previous_valid &&
                            previous_seq[k*32 +:32]==frame_source_seq[k*32 +:32];
                    end
                    if((acquired | frame_ack)==15) begin
                        acquire_tick<=mclk_tick;
                        state<=cancelled || abort ? DRAIN : STREAM;
                    end
                end
                STREAM: begin
                    if(abort || cancelled) begin
                        state<=DRAIN; held<=0;
                    end else if(pixel_valid && pixel_ready) begin
                        held<=0;
                        if(last) begin
                            state<=DRAIN; previous_seq<=source_seq; previous_valid<=1;
                        end else if(col==159) begin col<=0; row<=row+1; end
                        else col<=col+1;
                    end
                end
                DRAIN: begin
                    held<=0;
                    if(ddr_idle==15 && line_ready==0 && !pending && channel_data_req==0)
                        state<=IDLE;
                end
            endcase
        end
    end
endmodule
