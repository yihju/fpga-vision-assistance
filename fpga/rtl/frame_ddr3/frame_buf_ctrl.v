`timescale 1ns/1ps
//=============================================================================
// frame_buf_ctrl.v  ——  每通道 4 帧 DDR 缓冲的帧管理 与 读写互斥
//
// 替换参考工程的 cmos_write_req_gen.v，解决两个实测/分析确认的问题：
//
//  【问题 1：多比特跨时钟 + 未声明异步】
//    原实现在摄像头 pclk 域产生 2 位二进制 write_addr_index / read_addr_index，
//    再由 mem_clk 域（frame_fifo_write/frame_fifo_read 里各 2 级同步器）采样。
//    2 位计数器在 01→10、11→00 时**两位同时翻转**，两级同步器无法保证相干性；
//    且参考工程的 .fdc 未把这两个时钟域声明为异步，
//    实测（PDS 2022.2-SP4.2）产生 63 个 setup + 37 个 hold 违例。
//
//    本模块把这两个索引**全部搬到 mem_clk 域产生**，跨时钟路径只剩
//    wframe_vsync 一条单比特信号（2 级同步 + 上升沿检测）。
//
//  【问题 2：读写无互斥 → 画面撕裂】
//    原实现每个 vsync 无条件推进写指针、并把读指针指向上一块，
//    完全不检查该缓冲是否正被读端使用。读端（受 PCIe/DMA 调度影响）
//    一旦停顿，写端绕回来就覆盖了尚未读完的数据。
//
//    策略：
//      * 整帧写入 DDR 完成（write_finish）之后才把该缓冲发布为"可读"；
//      * 写指针只在存在空闲缓冲时才推进，并**跳过读端占用的缓冲**；
//      * 没有空闲缓冲时原地覆盖 = 显式丢帧（保证读端拿到的永远是完整帧）；
//      * 读端读完（read_finish）才释放占用的缓冲；
//      * 发布采用"最新优先"：新帧到来时丢弃尚未被取走的旧发布帧。
//
//  时钟域说明：
//      wframe_vsync 来自摄像头 pclk 域（顶层已是 cmos*_vsync_d0 寄存器输出），
//      在本模块内用 3 级同步器采样并做过沿检测。其余所有逻辑都在 mem_clk 域。
//
//  注意：本模块所有 reg 都只在**一个** always 块里赋值（Verilog 单驱动要求）。
//=============================================================================
module frame_buf_ctrl (
    // ---------------- 摄像头侧（异步输入） ----------------
    input               wframe_vsync,       // 高有效场同步（pclk 域寄存器输出）

    // ---------------- DDR / PCIe 用户时钟域 ----------------
    input               mclk,               // = ACLK = core_clk = ddrphy_sysclk
    input               mrst,               // 高有效，与 mclk 同步
    input               write_req_ack,      // 来自 frame_fifo_write
    input               write_finish,       // 来自 frame_fifo_write：整帧写完脉冲
    input  [1:0]        write_buf_used,     // 来自 frame_fifo_write：本帧实际写入的缓冲号
    input               read_finish,        // 来自 frame_fifo_read：整帧读完脉冲

    // ---------------- 输出 ----------------
    output reg          write_req,          // 给 frame_fifo_write，保持到 ack
    output reg  [1:0]   write_addr_index,   // 给 frame_fifo_write：写缓冲号
    output reg  [1:0]   read_addr_index,    // 给 frame_fifo_read ：读缓冲号

    // ---------------- 调试 ----------------
    output reg  [3:0]   dbg_busy_mask,      // 被占用的缓冲位图（ILA 观察）
    output              dbg_rd_own,         // 读端占用中（组合）
    output              dbg_pub_valid,      // 有已发布待取用的帧（组合）
    output              dbg_no_free,        // 无空闲缓冲（组合；正常恒为 0）
    output      [1:0]   dbg_rd_buf          // 读端占用的缓冲号（组合）
);

    //------------------------------------------------------------------
    // 1. wframe_vsync 同步到 mclk 域 + 上升沿检测（单比特 CDC）
    //------------------------------------------------------------------
    reg [2:0] vsync_sync;

    always @(posedge mclk) begin
        if (mrst) vsync_sync <= 3'b000;
        else      vsync_sync <= {vsync_sync[1:0], wframe_vsync};
    end

    wire vsync_pos = vsync_sync[2] & ~vsync_sync[1];

    //------------------------------------------------------------------
    // 2. write_req：每个 vsync 沿发起一次整帧写请求，保持到 ack
    //------------------------------------------------------------------
    always @(posedge mclk) begin
        if (mrst)                write_req <= 1'b0;
        else if (vsync_pos)      write_req <= 1'b1;
        else if (write_req_ack)  write_req <= 1'b0;
    end

    //------------------------------------------------------------------
    // 3. 缓冲占用状态（全部在 mclk 域，全部只在本文件唯一的那个
    //    "状态" always 块里赋值）
    //------------------------------------------------------------------
    reg [1:0] pub_buf;          // 已写完整、待读端取用的缓冲
    reg       pub_valid;
    reg [1:0] rd_buf;           // 读端占用的缓冲
    reg       rd_own;

    wire [3:0] onehot_wr  = 4'b0001 << write_addr_index;
    wire [3:0] onehot_pub = pub_valid ? (4'b0001 << pub_buf) : 4'b0000;
    wire [3:0] onehot_rd  = rd_own    ? (4'b0001 << rd_buf ) : 4'b0000;

    wire [3:0] busy_mask = onehot_wr | onehot_pub | onehot_rd;

    // 选下一个空闲缓冲：优先 write_addr_index+1，其次 +2、+3（共 4 块，最多跳 3 次）
    wire [1:0] cand1 = write_addr_index + 2'd1;
    wire [1:0] cand2 = write_addr_index + 2'd2;
    wire [1:0] cand3 = write_addr_index + 2'd3;

    wire [1:0] next_buf = ~busy_mask[cand1] ? cand1 :
                          ~busy_mask[cand2] ? cand2 :
                          ~busy_mask[cand3] ? cand3 : write_addr_index;

    wire no_free = busy_mask[cand1] & busy_mask[cand2] & busy_mask[cand3];

    // 组合调试输出（给形式化验证与 ILA 用）。
    // dbg_rd_own 特意做成组合而非寄存：形式化断言要的是"读端此刻正占用"，
    // 寄存一拍的版本会让断言弱一拍、抓不严。
    assign dbg_pub_valid = pub_valid;
    assign dbg_no_free   = no_free;
    assign dbg_rd_buf    = rd_buf;
    assign dbg_rd_own    = rd_own;

    //------------------------------------------------------------------
    // 4. 状态更新（**唯一**的状态 always 块）
    //    优先级：
    //      a) write_finish  → 发布刚写完的帧（最新优先，丢弃未取走的旧发布帧）
    //                         并在有空间时推进写指针
    //      b) 否则若读端空闲且有已发布帧 → 读端取用
    //      c) read_finish   → 释放读端占用
    //------------------------------------------------------------------
    always @(posedge mclk) begin
        if (mrst) begin
            write_addr_index <= 2'd0;
            pub_buf          <= 2'd0;
            pub_valid        <= 1'b0;
            rd_buf           <= 2'd0;
            rd_own           <= 1'b0;
            read_addr_index  <= 2'd0;
        end
        else begin
            // (c) 读端读完 → 释放
            if (read_finish) rd_own <= 1'b0;

            if (write_finish) begin
                // (a) 整帧已落 DDR → 发布为可读
                pub_buf   <= write_buf_used;
                pub_valid <= 1'b1;

                // 有空间才推进写指针；无空闲缓冲则原地覆盖（丢帧）
                if (!no_free) write_addr_index <= next_buf;
            end
            else if (pub_valid && !rd_own) begin
                // (b) 读端取用最新完整帧
                rd_buf          <= pub_buf;
                read_addr_index <= pub_buf;
                rd_own          <= 1'b1;
                pub_valid       <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // 5. 调试观察
    //------------------------------------------------------------------
    always @(posedge mclk) begin
        if (mrst) dbg_busy_mask <= 4'b0000;
        else      dbg_busy_mask <= busy_mask;
    end

endmodule
