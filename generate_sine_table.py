"""Generate the 1024-entry, 14-bit offset-binary sine lookup table.

Run from any directory with Python 3. The HEX file is consumed by the RTL;
the MIF contains the same values for inspection in Quartus.
"""
from math import floor, pi, sin
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEPTH = 1024
CENTER = 8192
AMPLITUDE = 8191


def main():
    # Half-up rounding, followed by clamping, yields codes 1 through 16383.
    samples = [
        max(0, min(16383, floor(CENTER + AMPLITUDE * sin(2 * pi * i / DEPTH) + 0.5)))
        for i in range(DEPTH)
    ]
    (ROOT / "sin_1024x14.hex").write_text(
        "".join(f"{value:04X}\n" for value in samples), encoding="ascii", newline="\n"
    )
    mif = ["WIDTH=14;", "DEPTH=1024;", "", "ADDRESS_RADIX=UNS;", "DATA_RADIX=UNS;", "", "CONTENT BEGIN"]
    mif.extend(f"    {address} : {value};" for address, value in enumerate(samples))
    mif.append("END;")
    (ROOT / "sin_1024x14.mif").write_text("\n".join(mif) + "\n", encoding="ascii", newline="\n")


if __name__ == "__main__":
    main()
