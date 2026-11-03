`timescale 1ns/1ps
// Start/finish toggles transfer stable bundles, not independent binary bits.
// Source finish alone does NOT flush pixels: the final segment may still be
// prefilling. Flush only on batch retire, and acknowledge idle after drain.
module plate_source_cdc(
    input pclk,input prst_n,input mclk,input mrst,
    input p_start,input p_abort,input p_retire,output reg p_started,output p_idle,
    output p_pixel_valid,input p_pixel_ready,output [127:0] p_pixel_data,
    output reg [127:0] p_source_seq,output reg [255:0] p_source_tick,
    output reg [63:0] p_acquire_tick,output reg [3:0] p_repeat_mask,
    output reg m_start,output m_abort,input m_source_idle,
    input m_pixel_valid,output m_pixel_ready,input [127:0] m_pixel_data,
    input [127:0] m_source_seq,input [255:0] m_source_tick,
    input [63:0] m_acquire_tick,input [3:0] m_repeat_mask
);
reg request_toggle,m_ack,m_finish,m_seen;
(* ASYNC_REG="TRUE" *) reg req1,req2,abort1,abort2,prst1,prst2;
(* ASYNC_REG="TRUE" *) reg ack1,ack2,finish1,finish2,mrst1,mrst2;
reg [1:0] ms;
reg [1:0] ps;
reg bundle_received;
reg [2:0] quiet;
reg [127:0] m_seq_hold;
reg [255:0] m_tick_hold;
reg [63:0] m_acquire_hold;
reg [3:0] m_repeat_hold;
wire fifo_valid,fifo_empty,fifo_write_ready;
wire draining=ps!=0 && p_retire;
assign p_idle=ps==0;
assign m_abort=mrst || !prst2 || abort2;
assign m_pixel_ready=fifo_write_ready && !m_abort;
assign p_pixel_valid=fifo_valid && ps==2 && !p_retire;
plate_pixel_cdc fifo (
    .wr_clk(mclk),.wr_rst(mrst || !prst2),.wr_valid(m_pixel_valid && !m_abort),
    .wr_ready(fifo_write_ready),.wr_data(m_pixel_data),
    .rd_clk(pclk),.rd_rst(!prst_n || mrst2),.rd_valid(fifo_valid),
    .rd_ready(draining || (ps==2 && p_pixel_ready)),.rd_data(p_pixel_data),.rd_empty(fifo_empty)
);
always @(posedge mclk) begin
    if(mrst)begin req1<=0;req2<=0;abort1<=0;abort2<=0;prst1<=0;prst2<=0;end
    else begin req1<=request_toggle;req2<=req1;abort1<=p_abort;abort2<=abort1;prst1<=prst_n;prst2<=prst1;end
    if(mrst || !prst2)begin
        ms<=0;m_ack<=0;m_finish<=0;m_seen<=0;m_start<=0;
        m_seq_hold<=0;m_tick_hold<=0;m_acquire_hold<=0;m_repeat_hold<=0;
    end else begin
        m_start<=0;
        case(ms)
            0:if(req2!=m_seen && m_source_idle)begin
                m_seen<=req2;m_ack<=req2;
                if(abort2)ms<=3;
                else begin m_start<=1;ms<=1;end
            end
            1:if(!m_source_idle)ms<=2;else if(abort2)ms<=3;
            2:if(m_source_idle)ms<=3;
            3:begin
                m_seq_hold<=m_source_seq;m_tick_hold<=m_source_tick;
                m_acquire_hold<=m_acquire_tick;m_repeat_hold<=m_repeat_mask;
                m_finish<=m_seen;ms<=0;
            end
        endcase
    end
end
always @(posedge pclk) begin
    if(!prst_n)begin ack1<=0;ack2<=0;finish1<=0;finish2<=0;mrst1<=1;mrst2<=1;end
    else begin ack1<=m_ack;ack2<=ack1;finish1<=m_finish;finish2<=finish1;mrst1<=mrst;mrst2<=mrst1;end
    if(!prst_n || mrst2)begin
        request_toggle<=0;ps<=0;bundle_received<=0;p_started<=0;quiet<=0;
        p_source_seq<=0;p_source_tick<=0;p_acquire_tick<=0;p_repeat_mask<=0;
    end else begin
        p_started<=0;
        if(ps==0)begin
            if(p_start)begin request_toggle<=~request_toggle;ps<=1;bundle_received<=0;quiet<=0;end
        end else begin
            if(ps==1 && ack2==request_toggle)begin ps<=2;p_started<=1;end
            if(ps==2 && !bundle_received && finish2==request_toggle)begin
                // MD holds these fields from before the synchronized finish
                // is observed until the NEXT completed frame handshake.
                p_source_seq<=m_seq_hold;p_source_tick<=m_tick_hold;
                p_acquire_tick<=m_acquire_hold;p_repeat_mask<=m_repeat_hold;
                bundle_received<=1;
            end
            if(bundle_received && p_retire && fifo_empty)begin
                if(quiet==4)begin ps<=0;quiet<=0;end
                else quiet<=quiet+1'b1;
            end else quiet<=0;
        end
    end
end
endmodule
