// SSD1306 128x32 OLED Controller via I2C
// 2 lines x 16 characters (8x16 font)
// Continuously refreshes display from text buffer
module ssd1306_oled #(
    parameter CLK_FREQ = 50000000
)(
    input  wire        clk,
    input  wire        rst_n,

    // Text input: 2 lines x 16 chars (ASCII)
    input  wire [127:0] line0,  // chars [15:0], char15 in [127:120]
    input  wire [127:0] line1,

    // I2C pins
    inout  wire        oled_scl,
    inout  wire        oled_sda,
    output wire [3:0]  debug_state,
    output wire        debug_addr_3d,
    output wire        debug_addr_ack,
    output wire [7:0]  debug_nack_count
);

    // SSD1306 I2C address (7-bit 0x3C, write = 0x78)
    // Start at 0x3C and switch to 0x3D only after an address NACK.
    localparam [7:0] I2C_ADDR_3C_W = 8'h78;
    localparam [7:0] I2C_ADDR_3D_W = 8'h7A;
    localparam [7:0] CTRL_CMD   = 8'h00; // Co=0, D/C#=0
    localparam [7:0] CTRL_DATA  = 8'h40; // Co=0, D/C#=1

    // I2C master interface
    reg        i2c_cmd_valid;
    reg  [1:0] i2c_cmd;
    reg  [7:0] i2c_wr_data;
    wire [7:0] i2c_rd_data;
    wire       i2c_cmd_done;
    wire       i2c_ack_err;

    i2c_master #(
        .CLK_FREQ(CLK_FREQ),
        .I2C_FREQ(50000)
    ) u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .cmd_valid(i2c_cmd_valid),
        .cmd      (i2c_cmd),
        .wr_data  (i2c_wr_data),
        .rd_data  (i2c_rd_data),
        .cmd_done (i2c_cmd_done),
        .ack_err  (i2c_ack_err),
        .scl      (oled_scl),
        .sda      (oled_sda)
    );

    // I2C command shortcuts
    localparam [1:0] I2C_START = 2'd0, I2C_WRITE = 2'd1, I2C_READ = 2'd2, I2C_STOP = 2'd3;

    // Init command table (SSD1306 128x32)
    localparam INIT_LEN = 25;
    reg [7:0] init_cmd;
    always @* begin
        case (init_count)
        5'd0:  init_cmd = 8'hAE; // Display OFF
        5'd1:  init_cmd = 8'hD5; // Set clock div
        5'd2:  init_cmd = 8'h80; //   ratio
        5'd3:  init_cmd = 8'hA8; // Set multiplex
        5'd4:  init_cmd = 8'h1F; //   32-1
        5'd5:  init_cmd = 8'hD3; // Display offset
        5'd6:  init_cmd = 8'h00; //   none
        5'd7:  init_cmd = 8'h40; // Start line 0
        5'd8:  init_cmd = 8'h8D; // Charge pump
        5'd9:  init_cmd = 8'h14; //   enable
        5'd10: init_cmd = 8'h20; // Memory mode
        5'd11: init_cmd = 8'h00; //   horizontal
        5'd12: init_cmd = 8'hA1; // Seg remap
        5'd13: init_cmd = 8'hC8; // COM scan dir
        5'd14: init_cmd = 8'hDA; // COM pins
        5'd15: init_cmd = 8'h02; //   sequential
        5'd16: init_cmd = 8'h81; // Contrast
        5'd17: init_cmd = 8'hCF; //   value
        5'd18: init_cmd = 8'hD9; // Precharge
        5'd19: init_cmd = 8'hF1; //   value
        5'd20: init_cmd = 8'hDB; // VCOMH
        5'd21: init_cmd = 8'h40; //   value
        5'd22: init_cmd = 8'hA4; // Entire ON (resume)
        5'd23: init_cmd = 8'hA6; // Normal display
        5'd24: init_cmd = 8'hAF; // Display ON
        default: init_cmd = 8'h00;
        endcase
    end

    // Font ROM instance
    reg  [6:0] font_ascii;
    reg  [3:0] font_col;
    reg        font_page;
    wire [7:0] font_data;
    font_8x16 u_font (
        .ascii   (font_ascii),
        .col     (font_col),
        .page_sel(font_page),
        .data    (font_data)
    );

    // Main state machine
    localparam [3:0]
        S_RESET_WAIT = 4'd0,
        S_INIT_CMD   = 4'd1,  // send one init command (START+addr+ctrl+cmd+STOP)
        S_INIT_WAIT  = 4'd2,
        S_DISP_PAGE  = 4'd3,  // set page/column address
        S_DISP_ADDR  = 4'd4,
        S_DISP_DATA  = 4'd5,  // send 128 data bytes
        S_DISP_STOP  = 4'd6,
        S_DISP_NEXT  = 4'd7,
        S_IDLE       = 4'd8;

    reg [3:0]  state;
    reg [3:0]  sub_state; // sub-step within a transaction
    reg        addr_3d;
    reg        addr_ack_seen;
    reg [7:0]  nack_count;
    reg [4:0]  init_count;
    reg [1:0]  page;
    reg [7:0]  col_cnt;
    // 50 MHz * 100 ms = 5,000,000, which requires 23 bits.
    reg [22:0] reset_cnt;
    reg [22:0] refresh_cnt; // delay between full refreshes

    // Character and column tracking for display rendering
    reg [3:0]  char_idx;  // 0..15
    reg [2:0]  char_col;  // 0..7 within character
    reg        char_page; // 0=top half, 1=bottom half
    wire [7:0] i2c_addr_w = addr_3d ? I2C_ADDR_3D_W : I2C_ADDR_3C_W;
    assign debug_state      = state;
    assign debug_addr_3d    = addr_3d;
    assign debug_addr_ack   = addr_ack_seen;
    assign debug_nack_count = nack_count;

    // Get ASCII for current position
    wire [7:0] current_char;
    wire [3:0] line_char_idx = char_idx;
    // line0: char[0] in bits [7:0], char[15] in bits [127:120]
    assign current_char = (page < 2'd2) ?
        line0[line_char_idx*8 +: 8] :
        line1[line_char_idx*8 +: 8];

    // Font lookup for current render position
    always @* begin
        font_ascii = current_char[6:0];
        font_col   = {1'b0, char_col[2:0]};
        font_page  = page[0]; // pages 0,2 = top half; pages 1,3 = bottom half
    end

    // I2C transaction sequencer
    // sub_state: 0=START, 1=addr, 2=ctrl, 3=data/cmd, 4=STOP, 5=done
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_RESET_WAIT;
            sub_state    <= 4'd0;
            addr_3d      <= 1'b0;
            addr_ack_seen<= 1'b0;
            nack_count   <= 8'd0;
            i2c_cmd_valid<= 1'b0;
            i2c_cmd      <= I2C_START;
            i2c_wr_data  <= 8'd0;
            init_count   <= 5'd0;
            page         <= 2'd0;
            col_cnt      <= 8'd0;
            reset_cnt    <= 23'd0;
            refresh_cnt  <= 23'd0;
            char_idx     <= 4'd0;
            char_col     <= 3'd0;
            char_page    <= 1'b0;
        end else begin
            i2c_cmd_valid <= 1'b0;

            if (i2c_cmd_done && !i2c_ack_err && sub_state == 4'd2)
                addr_ack_seen <= 1'b1;
            if (i2c_cmd_done && i2c_ack_err) begin
                // A NACK at sub-step 2 is the address byte; try 0x3D/0x3C next.
                if (sub_state == 4'd2) begin
                    addr_3d <= ~addr_3d;
                    addr_ack_seen <= 1'b0;
                end
                nack_count <= nack_count + 1'b1;
                state      <= S_RESET_WAIT;
                sub_state  <= 4'd0;
                init_count <= 5'd0;
                page       <= 2'd0;
                reset_cnt  <= 23'd0;
            end else case (state)
            // Wait ~100ms after power-on for SSD1306 to stabilise
            S_RESET_WAIT: begin
                if (reset_cnt == CLK_FREQ / 10) begin // 100 ms
                    state <= S_INIT_CMD;
                    sub_state <= 4'd0;
                    init_count <= 5'd0;
                end else begin
                    reset_cnt <= reset_cnt + 1'b1;
                end
            end

            // Send init commands one by one
            S_INIT_CMD: begin
                case (sub_state)
                4'd0: begin i2c_cmd <= I2C_START; i2c_cmd_valid <= 1'b1; sub_state <= 4'd1; end
                4'd1: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= i2c_addr_w; i2c_cmd_valid <= 1'b1; sub_state <= 4'd2; end
                4'd2: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= CTRL_CMD; i2c_cmd_valid <= 1'b1; sub_state <= 4'd3; end
                4'd3: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= init_cmd; i2c_cmd_valid <= 1'b1; sub_state <= 4'd4; end
                4'd4: if (i2c_cmd_done) begin i2c_cmd <= I2C_STOP; i2c_cmd_valid <= 1'b1; sub_state <= 4'd5; end
                4'd5: if (i2c_cmd_done) begin
                    if (init_count == INIT_LEN - 1) begin
                        state <= S_DISP_PAGE;
                        page <= 2'd0;
                        refresh_cnt <= 23'd0;
                    end else begin
                        init_count <= init_count + 1'b1;
                        sub_state <= 4'd0;
                    end
                end
                default: sub_state <= 4'd0;
                endcase
            end

            // Set page and column address before data transfer
            S_DISP_PAGE: begin
                case (sub_state)
                4'd0: begin i2c_cmd <= I2C_START; i2c_cmd_valid <= 1'b1; sub_state <= 4'd1; end
                4'd1: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= i2c_addr_w; i2c_cmd_valid <= 1'b1; sub_state <= 4'd2; end
                4'd2: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= CTRL_CMD; i2c_cmd_valid <= 1'b1; sub_state <= 4'd3; end
                // Set page address: 0xB0 | page
                4'd3: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= 8'hB0 | {6'd0, page}; i2c_cmd_valid <= 1'b1; sub_state <= 4'd4; end
                // Set lower column address: 0x00
                4'd4: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= 8'h00; i2c_cmd_valid <= 1'b1; sub_state <= 4'd5; end
                // Set higher column address: 0x10
                4'd5: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= 8'h10; i2c_cmd_valid <= 1'b1; sub_state <= 4'd6; end
                4'd6: if (i2c_cmd_done) begin i2c_cmd <= I2C_STOP; i2c_cmd_valid <= 1'b1; sub_state <= 4'd7; end
                4'd7: if (i2c_cmd_done) begin
                    state <= S_DISP_DATA;
                    sub_state <= 4'd0;
                    col_cnt <= 8'd0;
                    char_idx <= 4'd0;
                    char_col <= 3'd0;
                    char_page <= page[0];
                end
                default: sub_state <= 4'd0;
                endcase
            end

            // Send 128 bytes of display data
            S_DISP_DATA: begin
                case (sub_state)
                4'd0: begin i2c_cmd <= I2C_START; i2c_cmd_valid <= 1'b1; sub_state <= 4'd1; end
                4'd1: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= i2c_addr_w; i2c_cmd_valid <= 1'b1; sub_state <= 4'd2; end
                4'd2: if (i2c_cmd_done) begin i2c_cmd <= I2C_WRITE; i2c_wr_data <= CTRL_DATA; i2c_cmd_valid <= 1'b1; sub_state <= 4'd3; end
                4'd3: if (i2c_cmd_done) begin
                    // Send font data byte
                    i2c_cmd <= I2C_WRITE;
                    i2c_wr_data <= font_data;
                    i2c_cmd_valid <= 1'b1;
                    sub_state <= 4'd4;
                end
                4'd4: if (i2c_cmd_done) begin
                    // Advance column position
                    if (char_col == 3'd7) begin
                        char_col <= 3'd0;
                        if (char_idx == 4'd15) begin
                            // All 128 columns sent
                            i2c_cmd <= I2C_STOP;
                            i2c_cmd_valid <= 1'b1;
                            sub_state <= 4'd5;
                        end else begin
                            char_idx <= char_idx + 1'b1;
                            sub_state <= 4'd3;
                        end
                    end else begin
                        char_col <= char_col + 1'b1;
                        sub_state <= 4'd3;
                    end
                end
                4'd5: if (i2c_cmd_done) begin
                    state <= S_DISP_NEXT;
                end
                default: sub_state <= 4'd0;
                endcase
            end

            // Advance to next page or restart refresh cycle
            S_DISP_NEXT: begin
                if (page == 2'd3) begin
                    state <= S_IDLE;
                    refresh_cnt <= 23'd0;
                end else begin
                    page <= page + 1'b1;
                    state <= S_DISP_PAGE;
                    sub_state <= 4'd0;
                end
            end

            // Wait between refreshes (~50 ms)
            S_IDLE: begin
                if (refresh_cnt == CLK_FREQ / 20) begin
                    state <= S_DISP_PAGE;
                    page <= 2'd0;
                    sub_state <= 4'd0;
                end else begin
                    refresh_cnt <= refresh_cnt + 1'b1;
                end
            end

            default: state <= S_RESET_WAIT;
            endcase
        end
    end

endmodule
