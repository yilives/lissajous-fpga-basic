// Block-I/Q phase tracker for an ADC-only low-bandwidth DDS loop.
//
// The ADC bus is captured elsewhere after its conversion edge.  This block
// receives that stable sample one ADC cycle later, correlates it with the
// continuous DDS base phase, and publishes one complex estimate per complete
// input cycle (with a 512-sample minimum).  No individual zero crossing can
// directly move the DDS phase.
//
// The loop is a PLL + FLL: the P path pulls the phase toward zero, and the
// FLL path integrates the per-block rotation of the measured angle (the
// 32-bit signed difference is wrap-correct) directly into the frequency
// word.  The FLL is required because a mod-2pi phase error alone is a
// zero-mean sawtooth while a frequency offset persists, so a phase-only
// loop can never acquire the residual frequency error left by the integer
// period tracker.
module adc_block_iq_pll (
    input  wire              adc_sample_clk,
    input  wire              dac_sample_clk,
    input  wire              reset_n,
    input  wire [9:0]        adc_sample,
    input  wire [9:0]        adc_center,
    input  wire [15:0]       estimated_period_samples,
    input  wire [31:0]       base_phase_dac,
    output wire signed [31:0] phase_rate_dac,
    output reg  signed [31:0] phase_error_dac,
    output reg               phase_valid_dac
);

    localparam [15:0] MIN_WINDOW_SAMPLES = 16'd512;
    localparam [17:0] INTEGRAL_TICKS = 18'd249999; // 10 ms at 25 MHz.
    // P term: one radian creates about 485 Hz of temporary frequency pull.
    // The 1/64 smoothing keeps the closed-loop bandwidth low; the 2^13
    // scaling gives an ~20 ms phase acquisition at 100 kHz (versus ~80 ms
    // at 2^15) without waking up the sawtooth capture problem.
    localparam integer PHASE_P_SHIFT = 13;
    localparam integer LOOP_FILTER_SHIFT = 6;
    localparam integer INTEGRAL_SHIFT = 23;
    localparam signed [39:0] MIN_CORRELATION_MAGNITUDE = 40'sd1000000;

    // The ADC bus, centre and period are updated in the 100 MHz capture
    // domain (clk[2]).  The correlation runs in the 25 MHz ADC domain
    // (clk[0]), whose worst-case capture-to-launch relationship to clk[2]
    // is only 5 ns.  Registering these inputs on the ADC clock moves the
    // subtract/multiply/sum path entirely into the 40 ns ADC clock period.
    reg [9:0]  adc_sample_c0;
    reg [9:0]  adc_center_c0;
    reg [15:0] period_c0;

    reg [31:0] base_phase_meta_adc;
    reg [31:0] base_phase_sync_adc;
    reg signed [39:0] iq_sum_i_adc;
    reg signed [39:0] iq_sum_q_adc;
    reg [15:0] iq_sample_count_adc;
    reg signed [39:0] iq_block_i_adc;
    reg signed [39:0] iq_block_q_adc;
    reg               iq_block_toggle_adc;

    wire [15:0] correlation_window_samples =
        (period_c0 < MIN_WINDOW_SAMPLES) ? MIN_WINDOW_SAMPLES : period_c0;

    // These synchronous ROMs are intentionally clocked by the 25 MHz ADC
    // sample clock. At an ADC edge their q values correspond to the phase
    // presented at the previous edge, exactly matching adc_sample, which is
    // the bus captured after that previous conversion edge.
    wire [9:0] iq_sine_address = base_phase_sync_adc[31:22];
    wire [9:0] iq_cosine_address = iq_sine_address + 10'd256;
    wire [13:0] iq_sine_data;
    wire [13:0] iq_cosine_data;
    wire signed [12:0] iq_sine_code =
        ($signed({1'b0, iq_sine_data}) - 15'sd8191) >>> 2;
    wire signed [12:0] iq_cosine_code =
        ($signed({1'b0, iq_cosine_data}) - 15'sd8191) >>> 2;
    wire signed [10:0] iq_centered_sample =
        $signed({1'b0, adc_sample_c0}) - $signed({1'b0, adc_center_c0});
    wire signed [23:0] iq_product_i = iq_centered_sample * iq_sine_code;
    wire signed [23:0] iq_product_q = iq_centered_sample * iq_cosine_code;
    wire signed [39:0] iq_sum_i_next = iq_sum_i_adc + iq_product_i;
    wire signed [39:0] iq_sum_q_next = iq_sum_q_adc + iq_product_q;

    reference_sin_rom1024 u_iq_sine_rom (
        .address(iq_sine_address),
        .clock  (adc_sample_clk),
        .q      (iq_sine_data)
    );
    reference_sin_rom1024 u_iq_cosine_rom (
        .address(iq_cosine_address),
        .clock  (adc_sample_clk),
        .q      (iq_cosine_data)
    );

    always @(posedge adc_sample_clk or negedge reset_n) begin
        if (!reset_n) begin
            adc_sample_c0        <= 10'd0;
            adc_center_c0        <= 10'd512;
            period_c0            <= 16'd512;
            base_phase_meta_adc  <= 32'd0;
            base_phase_sync_adc  <= 32'd0;
            iq_sum_i_adc         <= 40'sd0;
            iq_sum_q_adc         <= 40'sd0;
            iq_sample_count_adc  <= 16'd0;
            iq_block_i_adc       <= 40'sd0;
            iq_block_q_adc       <= 40'sd0;
            iq_block_toggle_adc  <= 1'b0;
        end else begin
            adc_sample_c0        <= adc_sample;
            adc_center_c0        <= adc_center;
            period_c0            <= estimated_period_samples;
            base_phase_meta_adc <= base_phase_dac;
            base_phase_sync_adc <= base_phase_meta_adc;

            if (iq_sample_count_adc == correlation_window_samples - 1'b1) begin
                iq_block_i_adc      <= iq_sum_i_next;
                iq_block_q_adc      <= iq_sum_q_next;
                iq_block_toggle_adc <= ~iq_block_toggle_adc;
                iq_sum_i_adc        <= 40'sd0;
                iq_sum_q_adc        <= 40'sd0;
                iq_sample_count_adc <= 16'd0;
            end else begin
                iq_sum_i_adc        <= iq_sum_i_next;
                iq_sum_q_adc        <= iq_sum_q_next;
                iq_sample_count_adc <= iq_sample_count_adc + 1'b1;
            end
        end
    end

    // The block result is held until the following block completes. A toggle
    // therefore transfers it safely into the DAC controller domain.
    reg signed [39:0] iq_block_i_meta;
    reg signed [39:0] iq_block_i_sync;
    reg signed [39:0] iq_block_q_meta;
    reg signed [39:0] iq_block_q_sync;
    reg               iq_toggle_meta;
    reg               iq_toggle_sync;
    reg               iq_toggle_seen;

    reg               cordic_busy;
    reg [3:0]         cordic_iteration;
    reg signed [39:0] cordic_x;
    reg signed [39:0] cordic_y;
    reg signed [31:0] cordic_angle;
    reg signed [31:0] phase_rate_p_dac;
    reg signed [31:0] phase_integral_dac;
    reg [17:0]        integral_tick_count;
    // FLL: previous block angle and the integrated frequency correction.
    // fll_rotation is the per-block phase rotation, unwrapped by the 32-bit
    // signed subtraction (valid for any drift below pi per block).
    reg signed [31:0] fll_last_angle_dac;
    reg signed [31:0] fll_rate_dac;
    reg               fll_valid;
    wire signed [31:0] fll_rotation_dac =
        $signed(cordic_angle_next - fll_last_angle_dac);
    // A fixed integration gain diverges: a frequency error rotates the angle
    // by N codes per block where N is the block length, so a gain that is
    // stable at 100 kHz (N=512, shift 8) is unstable below ~49 kHz (N>=1024).
    // Grade the shift by the correlation window instead: the loop coefficient
    // is 1 - N/2^(shift+1), kept at or above 0.5 in every band.  A window of
    // N samples at 100 kHz (N=512, shift 9) acquires in ~2 blocks.
    reg [15:0] window_meta_dac;
    reg [15:0] window_sync_dac;
    wire [3:0] fll_rate_shift = (window_sync_dac < 16'd1024) ? 4'd9 :
                                (window_sync_dac < 16'd4096) ? 4'd11 :
                                (window_sync_dac < 16'd16384) ? 4'd13 : 4'd15;
    // Residue-carrying integrator: the shift truncates the per-block rotation
    // and a fixed small frequency error would otherwise sit in the dead zone
    // and rotate the Lissajous figure forever.  Carry the truncation residue
    // into the next block so any steady rotation, however small, is eventually
    // corrected (the P path cannot absorb it: it contributes only
    // rotation/2^P_SHIFT per block).
    reg signed [31:0] fll_rem;
    wire signed [32:0] fll_combined =
        $signed(fll_rotation_dac) + $signed(fll_rem);
    wire signed [32:0] fll_rate_delta_ext = fll_combined >>> fll_rate_shift;
    wire signed [31:0] fll_rate_delta = fll_rate_delta_ext[31:0];
    wire signed [32:0] fll_rem_full = fll_combined - (fll_rate_delta_ext << fll_rate_shift);
    wire signed [31:0] fll_rem_next = fll_rem_full[31:0];
    // Bound the integrated correction to about +-2.9 kHz of the 2^32 scale.
    wire signed [31:0] fll_rate_limited = fll_rate_dac + fll_rate_delta;
    wire signed [31:0] fll_rate_next =
        (fll_rate_limited > 32'sd33554432) ? 32'sd33554432 :
        (fll_rate_limited < -32'sd33554432) ? -32'sd33554432 :
        fll_rate_limited;

    function signed [31:0] cordic_atan;
        input [3:0] index;
        begin
            case (index)
                4'd0:  cordic_atan = 32'sh20000000;
                4'd1:  cordic_atan = 32'sh12E4051D;
                4'd2:  cordic_atan = 32'sh09FB385B;
                4'd3:  cordic_atan = 32'sh051111D4;
                4'd4:  cordic_atan = 32'sh028B0D43;
                4'd5:  cordic_atan = 32'sh0145D7E1;
                4'd6:  cordic_atan = 32'sh00A2F61E;
                4'd7:  cordic_atan = 32'sh00517C55;
                4'd8:  cordic_atan = 32'sh0028BE53;
                4'd9:  cordic_atan = 32'sh00145F2E;
                4'd10: cordic_atan = 32'sh000A2F98;
                4'd11: cordic_atan = 32'sh000517CC;
                4'd12: cordic_atan = 32'sh00028BE6;
                4'd13: cordic_atan = 32'sh000145F3;
                4'd14: cordic_atan = 32'sh0000A2F9;
                default: cordic_atan = 32'sh0000517C;
            endcase
        end
    endfunction

    wire cordic_y_positive = !cordic_y[39];
    wire signed [39:0] cordic_x_next = cordic_y_positive ?
        (cordic_x + (cordic_y >>> cordic_iteration)) :
        (cordic_x - (cordic_y >>> cordic_iteration));
    wire signed [39:0] cordic_y_next = cordic_y_positive ?
        (cordic_y - (cordic_x >>> cordic_iteration)) :
        (cordic_y + (cordic_x >>> cordic_iteration));
    wire signed [31:0] cordic_angle_next = cordic_y_positive ?
        (cordic_angle + cordic_atan(cordic_iteration)) :
        (cordic_angle - cordic_atan(cordic_iteration));
    wire signed [31:0] phase_rate_target = cordic_angle_next >>> PHASE_P_SHIFT;
    wire signed [31:0] phase_rate_p_delta = phase_rate_target - phase_rate_p_dac;
    wire signed [31:0] phase_rate_p_next = phase_rate_p_dac +
        (phase_rate_p_delta >>> LOOP_FILTER_SHIFT);
    wire signed [31:0] phase_error_delta = cordic_angle_next - phase_error_dac;
    wire signed [31:0] phase_error_filtered_next = phase_error_dac +
        (phase_error_delta >>> LOOP_FILTER_SHIFT);
    wire signed [31:0] phase_integral_delta = phase_error_dac >>> INTEGRAL_SHIFT;

    // [AI Mod: FLL-only 2026-08-01] Phase is anchored by the zero-crossing
    // anchor in the top level; this block contributes frequency correction
    // only, so the P and integral paths are bypassed.
    assign phase_rate_dac = fll_rate_dac;

    always @(posedge dac_sample_clk or negedge reset_n) begin
        if (!reset_n) begin
            iq_block_i_meta    <= 40'sd0;
            iq_block_i_sync    <= 40'sd0;
            iq_block_q_meta    <= 40'sd0;
            iq_block_q_sync    <= 40'sd0;
            iq_toggle_meta     <= 1'b0;
            iq_toggle_sync     <= 1'b0;
            iq_toggle_seen     <= 1'b0;
            cordic_busy        <= 1'b0;
            cordic_iteration   <= 4'd0;
            cordic_x           <= 40'sd0;
            cordic_y           <= 40'sd0;
            cordic_angle       <= 32'sd0;
            phase_rate_p_dac   <= 32'sd0;
            phase_integral_dac <= 32'sd0;
            phase_error_dac    <= 32'sd0;
            phase_valid_dac    <= 1'b0;
            integral_tick_count <= 18'd0;
            fll_last_angle_dac <= 32'sd0;
            fll_rate_dac       <= 32'sd0;
            fll_valid          <= 1'b0;
            fll_rem            <= 32'sd0;
            window_meta_dac    <= 16'd512;
            window_sync_dac    <= 16'd512;
        end else begin
            window_meta_dac <= correlation_window_samples;
            window_sync_dac <= window_meta_dac;
            iq_block_i_meta <= iq_block_i_adc;
            iq_block_i_sync <= iq_block_i_meta;
            iq_block_q_meta <= iq_block_q_adc;
            iq_block_q_sync <= iq_block_q_meta;
            iq_toggle_meta  <= iq_block_toggle_adc;
            iq_toggle_sync  <= iq_toggle_meta;

            // [AI Mod: integral path disabled - anchor owns phase]
            // if (integral_tick_count == INTEGRAL_TICKS) begin
            //     integral_tick_count <= 18'd0;
            //     if (phase_valid_dac)
            //         phase_integral_dac <= phase_integral_dac + phase_integral_delta;
            // end else begin
            //     integral_tick_count <= integral_tick_count + 1'b1;
            // end

            if (cordic_busy) begin
                if (cordic_iteration == 4'd15) begin
                    cordic_busy <= 1'b0;
                    if (cordic_x_next >= MIN_CORRELATION_MAGNITUDE) begin
                        // [AI Mod: P path disabled - anchor owns phase]
                        // phase_error_dac  <= phase_error_filtered_next;
                        // phase_rate_p_dac <= phase_rate_p_next;
                        phase_valid_dac  <= 1'b1;
                        // The first valid block only arms the reference
                        // angle; integrating a rotation computed against the
                        // power-on zero would inject a huge spurious kick.
                        fll_last_angle_dac <= cordic_angle_next;
                        if (fll_valid) begin
                            fll_rem     <= fll_rem_next;
                            fll_rate_dac <= fll_rate_next;
                        end
                        fll_valid <= 1'b1;
                    end else begin
                        phase_valid_dac  <= 1'b0;
                    end
                end else begin
                    cordic_x         <= cordic_x_next;
                    cordic_y         <= cordic_y_next;
                    cordic_angle     <= cordic_angle_next;
                    cordic_iteration <= cordic_iteration + 1'b1;
                end
            end else if (iq_toggle_sync != iq_toggle_seen) begin
                iq_toggle_seen <= iq_toggle_sync;
                cordic_busy    <= 1'b1;
                cordic_iteration <= 4'd0;
                if (iq_block_i_sync[39]) begin
                    cordic_x     <= -iq_block_i_sync;
                    cordic_y     <= -iq_block_q_sync;
                    cordic_angle <= 32'sh80000000;
                end else begin
                    cordic_x     <= iq_block_i_sync;
                    cordic_y     <= iq_block_q_sync;
                    cordic_angle <= 32'sd0;
                end
            end
        end
    end
endmodule
