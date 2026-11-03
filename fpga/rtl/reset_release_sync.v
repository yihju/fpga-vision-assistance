`timescale 1ns/1ps

// Asynchronous assertion, two-clock synchronous release.
// Use this when a ready/reset indication originates in another clock domain.
module reset_release_sync (
    input  wire clk,
    input  wire async_ready,
    output wire sync_ready
);

reg [1:0] sync_ff;

always @(posedge clk or negedge async_ready) begin
    if (!async_ready)
        sync_ff <= 2'b00;
    else
        sync_ff <= {sync_ff[0], 1'b1};
end

assign sync_ready = sync_ff[1];

endmodule
