// Original 5x7 pixel glyphs for the two status lines used by this design.
// Each row is doubled vertically within an 8x16 cell; the surrounding
// blank pixels provide spacing. Unsupported characters render as spaces.
// The combinational interface returns SSD1306 page bytes, bit 0 at the top.
module font_8x16 (
    input  wire [6:0] ascii,
    input  wire [3:0] col,
    input  wire       page_sel,
    output reg  [7:0] data
);
    // Seven rows, left pixel first.  The status text needs decimal digits,
    // hexadecimal A-F, M/O/P and the colon; space is the default.
    reg [34:0] glyph;
    always @* begin
        case (ascii)
            "0": glyph = {5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
            "1": glyph = {5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
            "2": glyph = {5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
            "3": glyph = {5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
            "4": glyph = {5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
            "5": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
            "6": glyph = {5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
            "7": glyph = {5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
            "8": glyph = {5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
            "9": glyph = {5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
            "A": glyph = {5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
            "B": glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
            "C": glyph = {5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
            "D": glyph = {5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
            "E": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
            "F": glyph = {5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
            "M": glyph = {5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
            "O": glyph = {5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
            "P": glyph = {5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
            ":": glyph = {5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
            default: glyph = 35'd0;
        endcase
    end

    integer bit_in_page;
    integer pixel_row;
    integer glyph_row;
    always @* begin
        data = 8'd0;
        for (bit_in_page = 0; bit_in_page < 8; bit_in_page = bit_in_page + 1) begin
            pixel_row = (page_sel ? 8 : 0) + bit_in_page;
            glyph_row = (pixel_row - 1) / 2;
            if (pixel_row >= 1 && pixel_row <= 14 && col >= 1 && col <= 5)
                data[bit_in_page] = glyph[34 - glyph_row * 5 - (col - 1)];
        end
    end
endmodule
