// Small synchronous byte FIFO for UART receive buffering.
module uart_byte_fifo #(
    parameter ADDR_WIDTH = 4
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [7:0]            wr_data,
    input  wire                  wr_en,
    input  wire                  rd_en,
    output wire [7:0]            rd_data,
    output wire                  empty,
    output wire                  full
);
    localparam integer DEPTH = 1 << ADDR_WIDTH;
    reg [7:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [ADDR_WIDTH:0] count;

    assign empty = (count == 0);
    assign full = (count == DEPTH);
    assign rd_data = mem[rd_ptr];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
            rd_ptr <= {ADDR_WIDTH{1'b0}};
            count  <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin mem[wr_ptr] <= wr_data; wr_ptr <= wr_ptr + 1'b1; count <= count + 1'b1; end
                2'b01: begin rd_ptr <= rd_ptr + 1'b1; count <= count - 1'b1; end
                2'b11: begin mem[wr_ptr] <= wr_data; wr_ptr <= wr_ptr + 1'b1; rd_ptr <= rd_ptr + 1'b1; end
                default: begin end
            endcase
        end
    end
endmodule