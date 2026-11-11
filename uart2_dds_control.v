// UART2 binary command endpoint for the PC visual closed loop.
//
// Frame: A5 5A | version | opcode | seq_le:u16 | length:u8 | payload |
//        CRC16-CCITT-FALSE(version through payload), little endian.
//
// Only one command may be outstanding.  The PC retries a command if its ACK
// does not arrive.  SET_DDS is idempotent, so retrying is safe.
module uart2_dds_control #(
    parameter CLK_FREQ = 50000000,
    parameter BAUD = 115200
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        visual_mode_enabled,
    input  wire        uart_rxd,
    output wire        uart_txd,

    output reg         ctrl_enabled,
    output wire        ctrl_live,
    output reg  [1:0]  ctrl_waveform,
    output reg  [7:0]  ctrl_ramp_p,
    output reg  [31:0] ctrl_phase_step,
    output reg  [31:0] ctrl_phase,
    output reg         ctrl_phase_apply,
    output reg  [9:0]  ctrl_amplitude,
    output reg         ctrl_commit_toggle
);

    localparam [7:0] PROTOCOL_VERSION = 8'h04;
    localparam [7:0] CMD_SET_DDS       = 8'h10;
    localparam [7:0] CMD_DISARM        = 8'h11;
    localparam [7:0] CMD_PING          = 8'h12;
    localparam [7:0] RSP_ACK           = 8'h90;

    localparam [7:0] STATUS_OK          = 8'h00;
    localparam [7:0] STATUS_BAD_LENGTH  = 8'h01;
    localparam [7:0] STATUS_BAD_PAYLOAD = 8'h02;
    localparam [7:0] STATUS_UNSUPPORTED = 8'h03;

    localparam [3:0] S_WAIT_A5 = 4'd0,
                     S_WAIT_5A = 4'd1,
                     S_VERSION = 4'd2,
                     S_OPCODE  = 4'd3,
                     S_SEQ_LO  = 4'd4,
                     S_SEQ_HI  = 4'd5,
                     S_LENGTH  = 4'd6,
                     S_PAYLOAD = 4'd7,
                     S_CRC_LO  = 4'd8,
                     S_CRC_HI  = 4'd9;

    function [15:0] crc16_byte;
        input [15:0] crc_input;
        input [7:0] data;
        integer bit_index;
        reg [15:0] crc;
        begin
            crc = crc_input ^ {data, 8'h00};
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if (crc[15])
                    crc = (crc << 1) ^ 16'h1021;
                else
                    crc = crc << 1;
            end
            crc16_byte = crc;
        end
    endfunction

    wire [7:0] rx_data;
    wire       rx_valid;
    wire [7:0] tx_data;
    wire       tx_valid;
    wire       tx_busy;

    uart_rx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart2_rx (
        .clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_valid(rx_valid),
        .uart_rxd(uart_rxd)
    );

    uart_tx #(.CLK_FREQ(CLK_FREQ), .BAUD(BAUD)) u_uart2_tx (
        .clk(clk), .rst_n(rst_n), .tx_data(tx_data), .tx_valid(tx_valid),
        .tx_busy(tx_busy), .uart_txd(uart_txd)
    );

    reg [3:0]  parser_state;
    reg [7:0]  frame_version;
    reg [7:0]  frame_opcode;
    reg [15:0] frame_sequence;
    reg [4:0]  frame_length;
    reg [4:0]  payload_index;
    reg [15:0] frame_crc;
    reg [7:0]  received_crc_lo;

    reg [7:0]  candidate_flags;
    reg [7:0]  candidate_waveform;
    reg [7:0]  candidate_ramp_p;
    reg [31:0] candidate_phase_step;
    reg [31:0] candidate_phase;
    reg [15:0] candidate_amplitude;

    // ACK payload: sequence:u16, echoed opcode:u8, status:u8, active:u8.
    reg        response_active;
    reg [3:0]  response_index;
    reg [15:0] response_sequence;
    reg [7:0]  response_opcode;
    reg [7:0]  response_status;
    reg        response_live;

    wire [15:0] response_crc =
        crc16_byte(
        crc16_byte(
        crc16_byte(
        crc16_byte(
        crc16_byte(
        crc16_byte(
        crc16_byte(16'hFFFF, RSP_ACK), 8'd5),
                   response_sequence[7:0]),
                   response_sequence[15:8]),
                   response_opcode),
                   response_status),
                   {7'd0, response_live});

    function [7:0] response_byte;
        input [3:0] index;
        begin
            case (index)
                4'd0:  response_byte = 8'hA5;
                4'd1:  response_byte = 8'h5A;
                4'd2:  response_byte = RSP_ACK;
                4'd3:  response_byte = 8'd5;
                4'd4:  response_byte = response_sequence[7:0];
                4'd5:  response_byte = response_sequence[15:8];
                4'd6:  response_byte = response_opcode;
                4'd7:  response_byte = response_status;
                4'd8:  response_byte = {7'd0, response_live};
                4'd9:  response_byte = response_crc[7:0];
                default: response_byte = response_crc[15:8];
            endcase
        end
    endfunction

    assign tx_data = response_byte(response_index);
    assign tx_valid = response_active && !tx_busy;
    wire response_fire = response_active && !tx_busy;

    // A valid SET_DDS owns DDS output until an explicit DISARM or S3 returns
    // to normal mode. Later SET_DDS frames update that persistent output.
    assign ctrl_live = ctrl_enabled && visual_mode_enabled;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parser_state       <= S_WAIT_A5;
            frame_version      <= 8'd0;
            frame_opcode       <= 8'd0;
            frame_sequence     <= 16'd0;
            frame_length       <= 5'd0;
            payload_index      <= 5'd0;
            frame_crc          <= 16'hFFFF;
            received_crc_lo    <= 8'd0;
            candidate_flags    <= 8'd0;
            candidate_waveform <= 8'd0;
            candidate_ramp_p   <= 8'd1;
            candidate_phase_step <= 32'd0;
            candidate_phase    <= 32'd0;
            candidate_amplitude <= 16'd0;

            ctrl_enabled       <= 1'b0;
            ctrl_waveform      <= 2'd0;
            ctrl_ramp_p        <= 8'd1;
            ctrl_phase_step    <= 32'd0;
            ctrl_phase         <= 32'd0;
            ctrl_phase_apply   <= 1'b0;
            ctrl_amplitude     <= 10'd0;
            ctrl_commit_toggle <= 1'b0;

            response_active    <= 1'b0;
            response_index     <= 4'd0;
            response_sequence  <= 16'd0;
            response_opcode    <= 8'd0;
            response_status    <= STATUS_OK;
            response_live      <= 1'b0;
        end else begin
            if (response_fire) begin
                if (response_index == 4'd10)
                    response_active <= 1'b0;
                else
                    response_index <= response_index + 1'b1;
            end

            if (rx_valid) begin
                case (parser_state)
                    S_WAIT_A5: begin
                        if (rx_data == 8'hA5)
                            parser_state <= S_WAIT_5A;
                    end
                    S_WAIT_5A: begin
                        if (rx_data == 8'h5A)
                            parser_state <= S_VERSION;
                        else if (rx_data != 8'hA5)
                            parser_state <= S_WAIT_A5;
                    end
                    S_VERSION: begin
                        frame_version <= rx_data;
                        frame_crc <= crc16_byte(16'hFFFF, rx_data);
                        parser_state <= S_OPCODE;
                    end
                    S_OPCODE: begin
                        frame_opcode <= rx_data;
                        frame_crc <= crc16_byte(frame_crc, rx_data);
                        parser_state <= S_SEQ_LO;
                    end
                    S_SEQ_LO: begin
                        frame_sequence[7:0] <= rx_data;
                        frame_crc <= crc16_byte(frame_crc, rx_data);
                        parser_state <= S_SEQ_HI;
                    end
                    S_SEQ_HI: begin
                        frame_sequence[15:8] <= rx_data;
                        frame_crc <= crc16_byte(frame_crc, rx_data);
                        parser_state <= S_LENGTH;
                    end
                    S_LENGTH: begin
                        frame_length <= rx_data[4:0];
                        frame_crc <= crc16_byte(frame_crc, rx_data);
                        payload_index <= 5'd0;
                        if (rx_data > 8'd16)
                            parser_state <= S_WAIT_A5;
                        else if (rx_data == 8'd0)
                            parser_state <= S_CRC_LO;
                        else
                            parser_state <= S_PAYLOAD;
                    end
                    S_PAYLOAD: begin
                        frame_crc <= crc16_byte(frame_crc, rx_data);
                        case (payload_index)
                            5'd0:  candidate_flags <= rx_data;
                            5'd1:  candidate_waveform <= rx_data;
                            5'd2:  candidate_ramp_p <= rx_data;
                            5'd3:  candidate_phase_step[7:0] <= rx_data;
                            5'd4:  candidate_phase_step[15:8] <= rx_data;
                            5'd5:  candidate_phase_step[23:16] <= rx_data;
                            5'd6:  candidate_phase_step[31:24] <= rx_data;
                            5'd7:  candidate_phase[7:0] <= rx_data;
                            5'd8:  candidate_phase[15:8] <= rx_data;
                            5'd9:  candidate_phase[23:16] <= rx_data;
                            5'd10: candidate_phase[31:24] <= rx_data;
                            5'd11: candidate_amplitude[7:0] <= rx_data;
                            5'd12: candidate_amplitude[15:8] <= rx_data;
                            default: begin end
                        endcase
                        if (payload_index == frame_length - 1'b1)
                            parser_state <= S_CRC_LO;
                        else
                            payload_index <= payload_index + 1'b1;
                    end
                    S_CRC_LO: begin
                        received_crc_lo <= rx_data;
                        parser_state <= S_CRC_HI;
                    end
                    S_CRC_HI: begin
                        parser_state <= S_WAIT_A5;
                        if ((frame_crc == {rx_data, received_crc_lo}) && !response_active) begin
                            response_active   <= 1'b1;
                            response_index    <= 4'd0;
                            response_sequence <= frame_sequence;
                            response_opcode   <= frame_opcode;
                            response_status   <= STATUS_OK;
                            response_live     <= ctrl_live;

                            if (frame_version != PROTOCOL_VERSION) begin
                                response_status <= STATUS_UNSUPPORTED;
                            end else if (frame_opcode == CMD_SET_DDS) begin
                                if (frame_length != 5'd13) begin
                                    response_status <= STATUS_BAD_LENGTH;
                                end else if ((candidate_waveform > 8'd1) ||
                                             (candidate_ramp_p < 8'd1) || (candidate_ramp_p > 8'd125) ||
                                             (candidate_amplitude > 16'd1023)) begin
                                    response_status <= STATUS_BAD_PAYLOAD;
                                end else begin
                                    ctrl_enabled       <= candidate_flags[0];
                                    ctrl_waveform      <= candidate_waveform[1:0];
                                    ctrl_ramp_p        <= candidate_ramp_p;
                                    ctrl_phase_step    <= candidate_phase_step;
                                    ctrl_phase         <= candidate_phase;
                                    ctrl_phase_apply   <= candidate_flags[1];
                                    ctrl_amplitude     <= candidate_amplitude[9:0];
                                    ctrl_commit_toggle <= ~ctrl_commit_toggle;
                                    response_live      <= candidate_flags[0] && visual_mode_enabled;
                                end
                            end else if (frame_opcode == CMD_DISARM) begin
                                if (frame_length != 5'd0) begin
                                    response_status <= STATUS_BAD_LENGTH;
                                end else begin
                                    ctrl_enabled       <= 1'b0;
                                    ctrl_commit_toggle <= ~ctrl_commit_toggle;
                                    response_live      <= 1'b0;
                                end
                            end else if (frame_opcode == CMD_PING) begin
                                if (frame_length != 5'd0)
                                    response_status <= STATUS_BAD_LENGTH;
                            end else begin
                                response_status <= STATUS_UNSUPPORTED;
                            end
                        end
                    end
                    default: parser_state <= S_WAIT_A5;
                endcase
            end
        end
    end
endmodule
