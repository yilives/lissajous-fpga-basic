# Board oscillator is 50 MHz on FPGA package pin 24.
create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
derive_pll_clocks
derive_clock_uncertainty

# ADC-to-DAC tracker values are held between updates and pass through two
# explicit synchronizer stages.  Their first stages are intentional CDC paths.
set_false_path -from [get_registers {phase_step_dac_adc*}] -to [get_registers {phase_step_dac_meta*}]
set_false_path -from [get_registers {phase_anchor_adc*}] -to [get_registers {phase_anchor_meta*}]
set_false_path -from [get_registers {input_path_phase_adc*}] -to [get_registers {input_path_phase_meta*}]
set_false_path -from [get_registers {input_pp_code*}] -to [get_registers {input_pp_meta*}]
set_false_path -from [get_registers {anchor_toggle_adc*}] -to [get_registers {anchor_toggle_meta*}]
set_false_path -from [get_registers {mode_select_adc*}] -to [get_registers {mode_select_meta*}]

# 3PA1030 capture timing: clk[0] is the physical 25 MHz ADC clock.  clk[2]
# is only a 100 MHz FPGA base clock.  adc_raw_sample is clock-enabled exactly
# 35 ns after a clk[0] rising edge; it is not sampled on the earlier base edges.
# Conventional edge-to-edge constraints would incorrectly analyse those ignored
# base edges, so this controlled capture path is excluded from generic STA and
# must be verified with the raw ADC-to-DAC diagnostic waveform before use.
set_false_path -from [get_ports {adc_data[*]}] -to [get_registers {adc_raw_sample[*]}]
set_false_path -from [get_ports {adc_otr}] -to [get_registers {adc_raw_otr}]

# clk[0] is intentionally re-timed through a two-register synchronizer in c2.
set_false_path -from [get_clocks {u_conv_clk_10m_pll|altpll_component|auto_generated|pll1|clk[0]}] -to [get_registers {adc_clk_meta_raw}]
# ADC tracking state is clocked by the 100 MHz capture clock but updates only
# while adc_sample_valid_raw is asserted, once per physical 25 MHz ADC
# conversion.  These are the complete registers in that enabled state machine;
# UART2, DDS, DAC output and raw ADC-capture registers are deliberately absent.
set adc_tracker_registers [get_registers {period_count* period_sum* period_average_count* divider_busy* divider_bit* divider_denominator* divider_remainder* divider_quotient* dac_ratio_busy* dac_ratio_bit* dac_ratio_dividend* dac_ratio_remainder* dac_ratio_quotient* amplitude_counter* amplitude_max* amplitude_min* input_pp_code* input_center_code* adc_sample* adc_otr_sample* adc_previous* adc_filter_z1* adc_filter_z2* adc_filter_z3* adc_phase_filtered_previous* phase_cross_fraction_adc* input_high* positive_crossing_reg* negative_crossing_reg* anchor_toggle_adc* phase_step_dac_adc* phase_anchor_adc* input_path_phase_adc* mode_key_meta* mode_key_sync* mode_key_stable* mode_key_count* mode_select_adc*}]
set_multicycle_path 4 -setup -from $adc_tracker_registers -to $adc_tracker_registers
set_multicycle_path 3 -hold  -from $adc_tracker_registers -to $adc_tracker_registers