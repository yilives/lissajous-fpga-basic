// I2C Master Controller - Cyclone IV E @ 50 MHz
// Command-driven: START / WRITE / READ / STOP
// SCL ~400 kHz (configurable via I2C_FREQ parameter)
module i2c_master #(
    parameter CLK_FREQ = 50000000,
    parameter I2C_FREQ = 400000
)(
    input  wire       clk,
    input  wire       rst_n,

    // Command interface
    input  wire       cmd_valid,   // pulse to start command
    input  wire [1:0] cmd,         // 0=START, 1=WRITE, 2=READ, 3=STOP
    input  wire [7:0] wr_data,
    output reg  [7:0] rd_data,
    output reg        cmd_done,    // pulse when complete
    output reg        ack_err,     // 1 if slave NACKed during WRITE

    // I2C open-drain pins
    inout  wire       scl,
    inout  wire       sda
);

    localparam [1:0] C_START = 2'd0, C_WRITE = 2'd1, C_READ = 2'd2, C_STOP = 2'd3;

    // SCL quarter-period in system clocks
    localparam integer QPD = CLK_FREQ / (I2C_FREQ * 4);
    localparam [15:0] QP = QPD[15:0] - 1'b1;

    // Open-drain: 1=release(pull-up), 0=drive low
    reg scl_oe, sda_oe;
    assign scl = scl_oe ? 1'bz : 1'b0;
    assign sda = sda_oe ? 1'bz : 1'b0;

    // SDA input synchroniser
    reg sda_s0, sda_s1;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin sda_s0 <= 1'b1; sda_s1 <= 1'b1; end
        else        begin sda_s0 <= sda;  sda_s1 <= sda_s0; end

    // FSM
    localparam [4:0]
        ST_IDLE      = 5'd0,
        ST_START_A   = 5'd1,  // SDA=1, SCL=1 setup
        ST_START_B   = 5'd2,  // SDA=0, SCL=1 (start condition)
        ST_START_C   = 5'd3,  // SCL=0
        ST_WR_DATA   = 5'd4,  // SCL=0, set SDA
        ST_WR_HIGH   = 5'd5,  // SCL=1, hold
        ST_WR_LOW    = 5'd6,  // SCL=0, next bit
        ST_WR_ACK_H  = 5'd7,  // SCL=1, sample ACK
        ST_WR_ACK_L  = 5'd8,  // SCL=0 after ACK
        ST_RD_LOW    = 5'd9,  // SCL=0, release SDA
        ST_RD_HIGH   = 5'd10, // SCL=1, sample data
        ST_RD_NEXT   = 5'd11, // SCL=0, advance
        ST_RD_ACK_H  = 5'd12, // SCL=1, master ACK
        ST_RD_ACK_L  = 5'd13, // SCL=0
        ST_STOP_A    = 5'd14, // SCL=0, SDA=0
        ST_STOP_B    = 5'd15, // SCL=1, SDA=0
        ST_STOP_C    = 5'd16; // SDA=1, bus free time

    reg [4:0]  state;
    reg [15:0] cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  sreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            cnt      <= 16'd0;
            bit_idx  <= 4'd0;
            sreg     <= 8'd0;
            scl_oe   <= 1'b1;
            sda_oe   <= 1'b1;
            cmd_done <= 1'b0;
            ack_err  <= 1'b0;
            rd_data  <= 8'd0;
        end else begin
            cmd_done <= 1'b0;

            case (state)
            // ----------------------------------------------------------
            ST_IDLE: begin
                scl_oe <= 1'b1;
                sda_oe <= 1'b1;
                if (cmd_valid) begin
                    ack_err <= 1'b0;
                    case (cmd)
                        C_START: begin state <= ST_START_A; cnt <= 16'd0; end
                        C_WRITE: begin state <= ST_WR_DATA; sreg <= wr_data; bit_idx <= 4'd7; cnt <= 16'd0; end
                        C_READ:  begin state <= ST_RD_LOW;  sreg <= 8'd0;  bit_idx <= 4'd7; cnt <= 16'd0; end
                        C_STOP:  begin state <= ST_STOP_A;  cnt <= 16'd0; end
                        default: cmd_done <= 1'b1;
                    endcase
                end
            end

            // --- START condition: SDA falls while SCL high ---
            ST_START_A: begin // Ensure SCL=1, SDA=1
                scl_oe <= 1'b1; sda_oe <= 1'b1;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_START_B; end
                else cnt <= cnt + 1'b1;
            end
            ST_START_B: begin // SDA -> 0 while SCL=1
                scl_oe <= 1'b1; sda_oe <= 1'b0;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_START_C; end
                else cnt <= cnt + 1'b1;
            end
            ST_START_C: begin // SCL -> 0
                scl_oe <= 1'b0; sda_oe <= 1'b0;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_IDLE; cmd_done <= 1'b1; end
                else cnt <= cnt + 1'b1;
            end

            // --- WRITE byte, MSB first ---
            ST_WR_DATA: begin // SCL=0, present data bit
                scl_oe <= 1'b0;
                sda_oe <= sreg[7]; // 1=release(high), 0=drive low
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_WR_HIGH; end
                else cnt <= cnt + 1'b1;
            end
            ST_WR_HIGH: begin // SCL=1 (data valid)
                scl_oe <= 1'b1;
                if (cnt == QP * 2) begin cnt <= 16'd0; state <= ST_WR_LOW; end
                else cnt <= cnt + 1'b1;
            end
            ST_WR_LOW: begin // SCL=0, advance bit
                scl_oe <= 1'b0;
                if (cnt == QP) begin
                    cnt <= 16'd0;
                    if (bit_idx == 4'd0) begin
                        state <= ST_WR_ACK_H; // go read ACK
                        sda_oe <= 1'b1;       // release SDA for slave
                    end else begin
                        bit_idx <= bit_idx - 1'b1;
                        sreg    <= {sreg[6:0], 1'b0};
                        state   <= ST_WR_DATA;
                    end
                end else cnt <= cnt + 1'b1;
            end
            ST_WR_ACK_H: begin // SCL=1, sample slave ACK (SDA=0 means ACK)
                scl_oe <= 1'b1; sda_oe <= 1'b1;
                if (cnt == QP) begin
                    ack_err <= sda_s1; // 1 = NACK
                    cnt <= 16'd0; state <= ST_WR_ACK_L;
                end else cnt <= cnt + 1'b1;
            end
            ST_WR_ACK_L: begin // SCL=0
                scl_oe <= 1'b0;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_IDLE; cmd_done <= 1'b1; end
                else cnt <= cnt + 1'b1;
            end

            // --- READ byte, MSB first ---
            ST_RD_LOW: begin // SCL=0, release SDA (slave drives)
                scl_oe <= 1'b0; sda_oe <= 1'b1;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_RD_HIGH; end
                else cnt <= cnt + 1'b1;
            end
            ST_RD_HIGH: begin // SCL=1, sample SDA
                scl_oe <= 1'b1;
                if (cnt == QP) begin
                    sreg <= {sreg[6:0], sda_s1};
                    cnt <= 16'd0; state <= ST_RD_NEXT;
                end else cnt <= cnt + 1'b1;
            end
            ST_RD_NEXT: begin // SCL=0, advance
                scl_oe <= 1'b0;
                if (cnt == QP) begin
                    cnt <= 16'd0;
                    if (bit_idx == 4'd0) begin
                        rd_data <= sreg;
                        state <= ST_RD_ACK_H;
                    end else begin
                        bit_idx <= bit_idx - 1'b1;
                        state   <= ST_RD_LOW;
                    end
                end else cnt <= cnt + 1'b1;
            end
            ST_RD_ACK_H: begin // Single-byte read ends with master NACK
                sda_oe <= 1'b1; scl_oe <= 1'b1;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_RD_ACK_L; end
                else cnt <= cnt + 1'b1;
            end
            ST_RD_ACK_L: begin // SCL=0
                scl_oe <= 1'b0;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_IDLE; cmd_done <= 1'b1; end
                else cnt <= cnt + 1'b1;
            end

            // --- STOP condition: SDA rises while SCL high ---
            ST_STOP_A: begin // SCL=0, SDA=0
                scl_oe <= 1'b0; sda_oe <= 1'b0;
                if (cnt == QP) begin cnt <= 16'd0; state <= ST_STOP_B; end
                else cnt <= cnt + 1'b1;
            end
            ST_STOP_B: begin // SCL=1, SDA=0 (setup)
                scl_oe <= 1'b1; sda_oe <= 1'b0;
                if (cnt == QP) begin
                    cnt <= 16'd0; state <= ST_STOP_C;
                end else cnt <= cnt + 1'b1;
            end
            ST_STOP_C: begin // SDA rises while SCL high = STOP, then bus free
                scl_oe <= 1'b1; sda_oe <= 1'b1;
                // Wait 4 quarter-periods as bus free time (t_BUF)
                if (cnt == QP * 4) begin
                    cnt <= 16'd0; state <= ST_IDLE; cmd_done <= 1'b1;
                end else cnt <= cnt + 1'b1;
            end

            default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
