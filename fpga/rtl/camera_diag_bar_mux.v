// BAR0 byte offsets 0x1800..0x184f and 0x1900..0x193f are read-only diagnosis.
// BAR addresses here index 16-byte RAM words, not bytes. Keep exactly the
// RAM's one-cycle enabled-read latency and preserve every other BAR0 read.
module camera_diag_bar_mux (
    input clk, input rst_n, input rd_en, input [11:0] rd_addr,
    input [127:0] ram_data, input [639:0] diagnostic,
    input [511:0] board_diagnostic,
    output [127:0] rd_data
);
reg selected;
reg [127:0] snapshot;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        selected <= 0; snapshot <= 0;
    end else if (rd_en) begin
        selected <= (rd_addr >= 12'h180 && rd_addr <= 12'h184) ||
                    (rd_addr >= 12'h190 && rd_addr <= 12'h193);
        case (rd_addr)
            12'h180: snapshot <= diagnostic[127:0];
            12'h181: snapshot <= diagnostic[255:128];
            12'h182: snapshot <= diagnostic[383:256];
            12'h183: snapshot <= diagnostic[511:384];
            12'h184: snapshot <= diagnostic[639:512];
            12'h190: snapshot <= board_diagnostic[127:0];
            12'h191: snapshot <= board_diagnostic[255:128];
            12'h192: snapshot <= board_diagnostic[383:256];
            12'h193: snapshot <= board_diagnostic[511:384];
            default: snapshot <= 0;
        endcase
    end
end
assign rd_data = selected ? snapshot : ram_data;
endmodule
