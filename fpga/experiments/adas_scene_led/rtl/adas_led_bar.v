// LEDI4: idle/timeout OFF; permitted direction green, other directions steady red; masked yellow risk.
// PCIe CSR clock may be Gen1=62.5MHz or Gen2=125MHz.
// All LED timing and watchdog intervals use independent board free_clk=25MHz.
// BAR1 byte0x240:0x4c44_TT_MM; BAR0 byte0x1c00:ID/command/TTL/mode.
// BAR0 byte0x1c10:LDV2/free-clock counter/TTL/{force,sending,blink,mode}.
module adas_led_bar #(
    parameter FRAME_CYCLES=125000
)(
    input clk,input led_clk,input rst_n,input wr_en,input[11:0] wr_addr,
    input[127:0] wr_data,input[15:0] wr_be,input rd_en,input[11:0] rd_addr,
    output reg rd_hit,output reg[127:0] rd_data,output reg led_dout=0
);
// Configuration INIT is supported by PDS (also used by existing frame DDR FSMs).
// This POR belongs only to the serial engine: PCIe reset cannot freeze a green
// WS2812 latch by stopping all future red packets.
reg[3:0] serial_por=0;
wire serial_ready=&serial_por;
always @(posedge led_clk)if(!serial_ready)serial_por<=serial_por+1'b1;

// PCIe reset: asynchronous assertion and two-free-clock synchronous release.
reg[1:0] core_release=0;
always @(posedge led_clk or negedge rst_n)begin
    if(!rst_n)core_release<=0;
    else core_release<={core_release[0],1'b1};
end
wire led_core_ready=core_release[1];

reg[31:0] last_command,command_hold;
reg command_req,command_ack,force_stop;
reg[1:0] ack_sync;
wire busy=command_req!=ack_sync[1];
wire address_write=wr_en && wr_addr==12'h024;
wire full_write=wr_be[3:0]==4'hf;
wire legal=wr_data[31:16]==16'h4c44 && wr_data[15:8]!=0 &&
           wr_data[15:8]<=200 && wr_data[7:4]==0 &&
           (wr_data[3:0]<=4 || wr_data[3:0]>=9);
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        last_command<=0;command_hold<=0;command_req<=0;force_stop<=1;ack_sync<=0;
    end else begin
        ack_sync<={ack_sync[0],command_ack};
        if(address_write)begin
            if(full_write)last_command<=wr_data[31:0];
            if(full_write && legal && !busy)begin
                // Bundled-data contract: holding word stays unchanged until
                // destination ACK has traversed both source synchronizers.
                command_hold<=wr_data[31:0];command_req<=~command_req;force_stop<=0;
            end else force_stop<=1;
        end
    end
end

reg[1:0] request_sync,stop_sync;
always @(posedge led_clk or negedge led_core_ready)begin
    if(!led_core_ready)begin request_sync<=0;stop_sync<=2'b11;end
    else begin
        request_sync<={request_sync[0],command_req};
        stop_sync<={stop_sync[0],force_stop};
    end
end

reg[14:0] millisecond=0;
reg[8:0] blink_ms=0;
reg blink_phase=1;
reg[31:0] free_ticks=0;
wire ms_tick=millisecond==24999;
always @(posedge led_clk)begin
    if(!serial_ready)begin millisecond<=0;blink_ms<=0;blink_phase<=1;free_ticks<=0;end
    else begin
        free_ticks<=free_ticks+1'b1;
        if(ms_tick)begin
            millisecond<=0;
            if(blink_ms==499)begin blink_ms<=0;blink_phase<=~blink_phase;end
            else blink_ms<=blink_ms+1'b1;
        end else millisecond<=millisecond+1'b1;
    end
end

reg[7:0] ttl_ms;
reg[3:0] mode;
always @(posedge led_clk or negedge led_core_ready)begin
    if(!led_core_ready)begin ttl_ms<=0;mode<=0;command_ack<=0;end
    else begin
        // Always consume/ACK a request, including while forced STOP. Clearing
        // force_stop later therefore cannot resurrect an old pending GO.
        if(request_sync[1]!=command_ack)command_ack<=request_sync[1];
        if(stop_sync[1])begin ttl_ms<=0;mode<=0;end
        else if(request_sync[1]!=command_ack)begin
            ttl_ms<=command_hold[15:8];mode<=command_hold[3:0];
        end else if(ms_tick && ttl_ms!=0)begin
            ttl_ms<=ttl_ms-1'b1;if(ttl_ms==1)mode<=0;
        end
    end
end

// Reverse coherent status mailbox: data held until the request's ACK returns.
// A BAR read is delayed by only a few clocks; this never affects the watchdog.
reg[63:0] status_hold,status_snapshot;
reg status_req,status_ack;
reg[1:0] status_ack_sync,status_req_sync;
reg[6:0] bits_left=0;
reg sending=0;
always @(posedge led_clk or negedge led_core_ready)begin
    if(!led_core_ready)begin status_hold<=0;status_req<=0;status_ack_sync<=0;end
    else begin
        status_ack_sync<={status_ack_sync[0],status_ack};
        if(status_req==status_ack_sync[1])begin
            status_hold<={10'd0,bits_left,stop_sync[1],sending,blink_phase,mode,ttl_ms,free_ticks};
            status_req<=~status_req;
        end
    end
end
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin status_snapshot<=0;status_req_sync<=0;status_ack<=0;end
    else begin
        status_req_sync<={status_req_sync[0],status_req};
        if(status_req_sync[1]!=status_ack)begin
            status_snapshot<=status_hold;status_ack<=status_req_sync[1];
        end
    end
end
always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin rd_hit<=0;rd_data<=0;end
    else begin
        rd_hit<=rd_en && (rd_addr==12'h1c0 || rd_addr==12'h1c1);
        if(rd_en)begin
            if(rd_addr==12'h1c1)
                rd_data<={25'd0,status_snapshot[46:40],24'd0,status_snapshot[39:32],
                          status_snapshot[31:0],32'h4c445633};
            else rd_data<={28'd0,status_snapshot[43:40],24'd0,status_snapshot[39:32],
                           last_command,32'h4c444934};
        end
    end
end

function[23:0] color;
input[1:0] direction;input[3:0] selected;input[7:0] lease;input blink;
begin
    if(lease==0 || selected==0)color=24'h000000;
    else if(selected==4)color=24'h001000;
    else if(selected>=9)color=(blink && selected[direction-1'b1])?24'h101000:24'h000000;
    else if(selected==direction)color=24'h100000;
    else color=24'h001000;
end endfunction
reg[16:0] frame_clock=FRAME_CYCLES-1;
reg[13:0] reset_clock=0;
reg[4:0] bit_clock=0;
reg[71:0] shift=0;
always @(posedge led_clk)begin
    if(!serial_ready)begin
        frame_clock<=FRAME_CYCLES-1;reset_clock<=0;bit_clock<=0;
        bits_left<=0;shift<=0;sending<=0;led_dout<=0;
    end else begin
        if(frame_clock<FRAME_CYCLES-1)frame_clock<=frame_clock+1'b1;
        if(sending)begin
            led_dout<=bit_clock<(shift[71]?16:8); // 640ns / 320ns at fixed25MHz
            if(bit_clock==31)begin
                bit_clock<=0;shift<={shift[70:0],1'b0};bits_left<=bits_left-1'b1;
                if(bits_left==1)begin sending<=0;reset_clock<=0;led_dout<=0;end
            end else bit_clock<=bit_clock+1'b1;
        end else begin
            led_dout<=0;
            if(reset_clock<10000)reset_clock<=reset_clock+1'b1;
            if(frame_clock==FRAME_CYCLES-1 && reset_clock==10000)begin
                shift<={color(1,mode,ttl_ms,blink_phase),color(2,mode,ttl_ms,blink_phase),
                        color(3,mode,ttl_ms,blink_phase)};
                sending<=1;bits_left<=72;bit_clock<=0;frame_clock<=0;
            end
        end
    end
end
endmodule
