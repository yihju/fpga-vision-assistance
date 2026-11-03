// Passive SCCB observer. A low level in every ACK slot means success.
module sccb_ack_diag (
    input clk, input rst_n, input transaction_done,
    input [3:0] ack_bits, input [15:0] register_address,
    output reg [15:0] ack_count, output reg [15:0] nack_count,
    output reg [15:0] last_fail_address, output reg [3:0] last_ack_bits
);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ack_count <= 0;
        nack_count <= 0;
        last_fail_address <= 0;
        last_ack_bits <= 0;
    end else if (transaction_done) begin
        last_ack_bits <= ack_bits;
        if (|ack_bits) begin
            nack_count <= nack_count + 1'b1;
            last_fail_address <= register_address;
        end else
            ack_count <= ack_count + 1'b1;
    end
end
endmodule
