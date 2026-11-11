// UART Receiver - 8N1, configurable baud rate
// Default: 115200 baud @ 50 MHz (divider = 434)
// Samples at mid-bit for robust reception
module uart_rx #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD     = 115200
)(
    input  wire       clk,
    input  wire       rst_n,

    // Byte output
    output reg  [7:0] rx_data,
    output reg        rx_valid,  // pulse when byte received

    // Serial input
    input  wire       uart_rxd
);

    localparam integer DIV = CLK_FREQ / BAUD;
    localparam [15:0] DIV_TICKS = DIV[15:0] - 1'b1;
    localparam [15:0] HALF_DIV  = DIV_TICKS / 2;

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    // Input synchroniser (2-FF)
    reg rxd_meta, rxd_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rxd_meta <= 1'b1; rxd_sync <= 1'b1; end
        else        begin rxd_meta <= uart_rxd; rxd_sync <= rxd_meta; end
    end

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
            rx_data  <= 8'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            case (state)
            S_IDLE: begin
                if (rxd_sync == 1'b0) begin // Falling edge = start bit
                    state    <= S_START;
                    baud_cnt <= 16'd0;
                end
            end

            S_START: begin
                // Wait half a bit period to reach centre of start bit
                if (baud_cnt == HALF_DIV) begin
                    if (rxd_sync == 1'b0) begin // Confirm still low
                        baud_cnt <= 16'd0;
                        bit_idx  <= 3'd0;
                        state    <= S_DATA;
                    end else begin
                        state <= S_IDLE; // False start
                    end
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end

            S_DATA: begin
                if (baud_cnt == DIV_TICKS) begin
                    baud_cnt <= 16'd0;
                    shift[bit_idx] <= rxd_sync; // LSB first
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
                if (baud_cnt == DIV_TICKS) begin
                    baud_cnt <= 16'd0;
                    if (rxd_sync == 1'b1) begin // Valid stop bit
                        rx_data  <= shift;
                        rx_valid <= 1'b1;
                    end
                    state <= S_IDLE;
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule
