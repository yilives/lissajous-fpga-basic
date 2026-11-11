# PC to FPGA UART2 DDS Protocol

This is the control link for the visual closed loop:

```text
CAM2 image -> MJPEG web stream -> PC vision -> USB-TTL -> FPGA UART2 -> DDS -> AD9764
```

CAM2 never sends a control byte to the FPGA on this path. The PC is the only
UART2 controller.

## Electrical and UART Settings

- UART: `115200-8-N-1`, 3.3 V LVTTL only.
- FPGA UART2 TX: `PIN_133`; connect to USB-TTL RX.
- FPGA UART2 RX: `PIN_136`; connect to USB-TTL TX.
- Connect a common GND. Do not connect a 5 V USB-TTL adapter directly.

## Frame Format

All multi-byte fields are little endian.

```text
A5 5A | version:u8 | opcode:u8 | sequence:u16 | payload_length:u8 |
payload[0..16] | CRC16-CCITT-FALSE:u16
```

- `version` is `0x04`. Versions 1--3 are intentionally rejected: v1 used profile codes, v2 used the wrong 100 MHz phase-step basis, and v3 had a one-second ownership watchdog.
- CRC starts at `0xFFFF`, uses polynomial `0x1021`, and covers `version`
  through the final payload byte. The SOF and CRC bytes are excluded.
- The PC keeps exactly one command in flight. It waits for the ACK, then
  retries the identical sequence at most twice after a timeout.

## Commands

| Opcode | Name | Payload |
|---:|---|---|
| `0x10` | `SET_DDS` | 13-byte DDS command below |
| `0x11` | `DISARM` | empty; return to original FPGA tracker |
| `0x12` | `PING` | empty; checks wiring and protocol |

`SET_DDS` payload:

```text
flags:u8 | waveform:u8 | ramp_p:u8 | phase_step:u32 |
phase_word:u32 | amplitude_code:u16
```

- `flags.bit0`: enable PC DDS ownership.
- `flags.bit1`: apply `phase_word` immediately. Clear this for a live
  frequency/amplitude update that must keep the NCO phase-continuous.
- `waveform`: `0=sine`, `1=ramp`.
- `ramp_p`: direct integer `P=1..125`, not an enumerated profile. The PC and FPGA apply the ramp with inverted DAC-code polarity so the physical CH2 voltage rises from 0 V to the positive peak. At the
  fixed 100 Hz frame rate, its rising scan is exactly
  `T_scan=1/(100*P)=10 ms/P`; the remainder of the frame stays at the 0 V DAC
  centre.  This makes `P=2`, `P=10`, `P=40`, and any other integer in range
  valid commands.
- `phase_step = round(f_hz * 2^32 / 25000000)`. The NCO update clock is
  25 MHz.
- `phase_word = round((phase_degrees mod 360) * 2^32 / 360)`.
- `amplitude_code` range is `0..1023`. The PC default is `409/4` code per
  requested Vpp and may be adjusted with `--fpga-code-per-vpp` after a scope
  calibration.

## P Selection for 1--100 kHz

The probe ramp is a uniform time scale.  In X-Y mode, let `M` be the count of
CH1 crossings with the centre Y axis.  Then `N=M/2` is the count of measured
periods in the scan window and:

```text
f_x = M / (2 * T_scan) = M * 100 * P / 2
Delta_t = 1 / (2 * f_x)
```

Start with `P=1`.  After a usable coarse estimate, the PC chooses an integer
`P=round(f_x/(100*25))`, clamped to `1..125`, so `N` is about 25 (and `M` about
50), within the intended 5--50-period image range.  At 100 kHz this selects
`P=40`, giving `T_scan=0.25 ms` and `N=25`; `P=125` is the supported shortest
window, `80 us`, still giving `N=8` at 100 kHz.  Values outside `1..125` are
rejected because they add no useful 1--100 kHz measurement range.

## ACK

The FPGA sends no unframed text on UART2. Every valid command receives:

```text
A5 5A | 90 | 05 | sequence:u16 | echoed_opcode:u8 | status:u8 | active:u8 | CRC16
```

`status`: `0=OK`, `1=bad length`, `2=bad payload`, `3=unsupported`.
`active=1` means the PC DDS path is currently selected.

A valid `SET_DDS` remains in force indefinitely while S3 has selected visual
mode. Each later `SET_DDS` replaces the frequency, phase, amplitude and P in
real time. Only `DISARM` or another S3 press returns DDS ownership to the
original ADC/tracker path; there is no serial keepalive timeout.

## PC Entry Point

```powershell
cd B:\intelFPGA\works\CAM2code\code
python pc_closed_loop.py --mode line --output fpga --serial-port COM9 --armed
```

This project uses `COM9`. First omit `--armed`; the program
prints exact UART2 frames without changing the FPGA output. Then use `PING`, a
static sine, then a probe at `P=1` before enabling the complete
visual loop.
## S3 Visual/Normal Selector

`S3` uses FPGA `IO85` and is active low. It is a debounced toggle, not a
momentary hold:

- Power-up starts in **normal mode**. The original FPGA tracker/DDS path is
  active and UART2 commands are stored but cannot drive the output.
- Press S3 once to enter **visual mode**. A live, CRC-valid `SET_DDS` command
  from the PC can then own the DDS.
- Press S3 again to return immediately to **normal mode**. The UART2 visual
  path is gated off and the existing FPGA tracker resumes.

The ACK `active` field is `1` only when both a valid PC command is live and S3
has selected visual mode. S4 (`IO87`) remains the existing local waveform/
amplitude-mode selector and is independent of S3.
## Requirement 3 and 4 Modes

The PC loop treats requirement 4 as an extension of requirement 3:
`f_out = 2 * f_in` is retained in both cases.

```powershell
# Requirement 3: symmetric figure eight, CH2 peak-to-peak amplitude tracks CH1.
python pc_closed_loop.py --mode eight --output fpga --serial-port COM9 --armed

# Requirement 4: retain the figure-eight two-times frequency, set CH2 to 2/4/6/8 div p-p.
python pc_closed_loop.py --mode eight --output fpga --serial-port COM9 --output-span-div 2 --armed
```

Replace `2` with `4`, `6`, or `8` for the other three requirement-4 settings.
The visual loop uses the rectified scope image to keep the selected Y height
within 0.20 div. A normal program exit does not send `DISARM`; the last valid
UART2 setting stays active until S3 is pressed again or `DISARM` is sent.
