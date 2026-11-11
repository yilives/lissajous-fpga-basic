// UART Transmitter - 8N1, configurable baud rate
// Default: 115200 baud @ 50 MHz (divider = 434)
module uart_tx #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,

    // Byte interface
    input  wire [7:0] tx_data,
    input  wire       tx_valid,  // pulse to start transmission
    output reg        tx_busy,

    // Serial output
    output reg        uart_txd
);

    localparam integer DIV = CLK_FREQ / BAUD;
    localparam [15:0] DIV_TICKS = DIV[15:0] - 1'b1;

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] baud_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shift;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            baud_cnt <= 16'd0;
            bit_idx  <= 3'd0;
            shift    <= 8'd0;
            uart_txd <= 1'b1;
            tx_busy  <= 1'b0;
        end else begin
            case (state)
            S_IDLE: begin
                uart_txd <= 1'b1;
                tx_busy  <= 1'b0;
                if (tx_valid) begin
                    shift   <= tx_data;
                    tx_busy <= 1'b1;
                    state   <= S_START;
                    baud_cnt<= 16'd0;
                end
            end

            S_START: begin
                uart_txd <= 1'b0; // Start bit
                if (baud_cnt == DIV_TICKS) begin
                    baud_cnt <= 16'd0;
                    bit_idx  <= 3'd0;
                    state    <= S_DATA;
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end

            S_DATA: begin
                uart_txd <= shift[bit_idx]; // LSB first
                if (baud_cnt == DIV_TICKS) begin
                    baud_cnt <= 16'd0;
                    if (bit_idx == 3'd7) begin
                        state <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 1'b1;
                    end
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end

            S_STOP: begin
                uart_txd <= 1'b1; // Stop bit
                if (baud_cnt == DIV_TICKS) begin
                    baud_cnt <= 16'd0;
                    state    <= S_IDLE;
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
