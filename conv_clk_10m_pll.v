// 50 MHz input -> configurable PLL VCO -> configurable output clock, 50 % duty cycle.
module conv_clk_10m_pll #(
    parameter integer CLK0_MULTIPLY = 12,
    parameter integer CLK0_DIVIDE = 40,
    parameter integer CLK1_MULTIPLY = 12,
    parameter integer CLK1_DIVIDE = 80,
    // c2 is the ADC-data capture clock.  It has the same rate as c0 but
    // occurs after the ADC sampling edge, so the ADC parallel output has a
    // defined time to settle before it enters FPGA registers.
    parameter integer CLK2_MULTIPLY = 12,
    parameter integer CLK2_DIVIDE = 40,
    parameter integer CLK2_PHASE_SHIFT_PS = 0
) (
    input  wire inclk0,
    output wire c0,
    output wire c1,
    output wire c2,
    output wire locked
);
    wire [1:0] inclk = {1'b0, inclk0};
    wire [4:0] clk;

    assign c0 = clk[0];
    assign c1 = clk[1];
    assign c2 = clk[2];

    altpll altpll_component (
        .inclk              (inclk),
        .clk                (clk),
        .activeclock        (),
        .areset             (1'b0),
        .clkbad             (),
        .clkena             ({6{1'b1}}),
        .clkloss            (),
        .clkswitch          (1'b0),
        .configupdate       (1'b0),
        .enable0            (),
        .enable1            (),
        .extclk             (),
        .extclkena          ({4{1'b1}}),
        .fbin               (1'b1),
        .fbmimicbidir       (),
        .fbout              (),
        .fref               (),
        .icdrclk            (),
        .locked             (locked),
        .pfdena             (1'b1),
        .phasecounterselect ({4{1'b1}}),
        .phasedone          (),
        .phasestep          (1'b1),
        .phaseupdown        (1'b1),
        .pllena             (1'b1),
        .scanaclr           (1'b0),
        .scanclk            (1'b0),
        .scanclkena         (1'b1),
        .scandata           (1'b0),
        .scandataout        (),
        .scandone           (),
        .scanread           (1'b0),
        .scanwrite          (1'b0),
        .sclkout0           (),
        .sclkout1           (),
        .vcooverrange       (),
        .vcounderrange      ()
    );

    defparam
        altpll_component.bandwidth_type         = "AUTO",
        altpll_component.clk0_divide_by         = CLK0_DIVIDE,
        altpll_component.clk0_duty_cycle        = 50,
        altpll_component.clk0_multiply_by       = CLK0_MULTIPLY,
        altpll_component.clk0_phase_shift       = "0",
        altpll_component.clk1_divide_by         = CLK1_DIVIDE,
        altpll_component.clk1_duty_cycle        = 50,
        altpll_component.clk1_multiply_by       = CLK1_MULTIPLY,
        altpll_component.clk1_phase_shift       = "0",
        altpll_component.clk2_divide_by         = CLK2_DIVIDE,
        altpll_component.clk2_duty_cycle        = 50,
        altpll_component.clk2_multiply_by       = CLK2_MULTIPLY,
        altpll_component.clk2_phase_shift       = CLK2_PHASE_SHIFT_PS,
        altpll_component.compensate_clock       = "CLK0",
        altpll_component.inclk0_input_frequency = 20000,
        altpll_component.intended_device_family = "Cyclone IV E",
        altpll_component.lpm_hint               = "CBX_MODULE_PREFIX=conv_clk_10m_pll",
        altpll_component.lpm_type               = "altpll",
        altpll_component.operation_mode         = "NORMAL",
        altpll_component.pll_type               = "AUTO",
        altpll_component.port_inclk0            = "PORT_USED",
        altpll_component.port_locked            = "PORT_USED",
        altpll_component.port_clk0              = "PORT_USED",
        altpll_component.port_clk1              = "PORT_USED",
        altpll_component.port_clk2              = "PORT_USED",
        altpll_component.width_clock            = 5;
endmodule