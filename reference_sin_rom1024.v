// 1024-sample offset-binary sine table. The registered read preserves the
// one-clock latency expected by the DDS and I/Q phase detector.
// Regenerate the table with: python generate_sine_table.py
module reference_sin_rom1024 (
    input wire [9:0] address,
    input wire clock,
    output reg [13:0] q
);
    reg [13:0] samples [0:1023];

    initial $readmemh("sin_1024x14.hex", samples);

    always @(posedge clock)
        q <= samples[address];
endmodule
