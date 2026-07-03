"""Test and visualize an Arduino Nano joystick over USB serial.

Pairs with scripts/arduino/joystick_test/joystick_test.ino, which streams one
CSV line per sample at 115200 baud:

    <x>,<y>,<sw>\\n

where x,y are 0..1023 raw ADC counts and sw is 1 (released) / 0 (pressed).

Usage:
  # auto-detect port — axis remap is baked in for the mounted joystick
  python3 scripts/test_joystick.py --port /dev/ttyUSB0

  # raw serial lines
  python3 scripts/test_joystick.py --port /dev/ttyUSB0 --raw

Move the stick to test. Ctrl+C to exit. Normalization uses fixed ADC center/span
(no startup calibration wiggle required).

WSL note: USB serial devices must be attached to WSL with `usbipd` on Windows,
e.g. `usbipd attach --wsl --busid <BUSID>`, before the port appears here.
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import serial
from serial.tools import list_ports

# USB vendor IDs commonly used by Arduino Nano boards and clones.
KNOWN_VID_PID = {
    (0x2341, None),  # Arduino
    (0x2A03, None),  # Arduino (arduino.org)
    (0x1A86, None),  # CH340/CH341 (very common Nano clone)
    (0x0403, None),  # FTDI
    (0x10C4, None),  # CP210x
}

ADC_MAX = 1023


@dataclass
class Args:
    port: Optional[str] = None
    """Serial port of the Nano. Auto-detected if omitted."""

    baud: int = 115200
    """Serial baud rate (must match the Arduino sketch)."""

    raw: bool = False
    """Print raw serial lines instead of the live dashboard."""

    deadzone: float = 0.08
    """Normalized magnitude below which an axis is treated as centered."""

    bar_width: int = 21
    """Character width of each axis bar (odd numbers center nicely)."""

    grid_size: int = 15
    """Side length (chars) of the 2D position grid."""

    swap_xy: bool = True
    """Map VRY to screen left/right and VRX to screen up/down."""

    invert_x: bool = False
    """Flip screen left/right."""

    invert_y: bool = False
    """Flip screen up/down."""

    center_x: int = 512
    """Resting ADC value for VRX (A0)."""

    center_y: int = 512
    """Resting ADC value for VRY (A1)."""

    half_span: int = 512
    """ADC counts from center to full deflection (1023-scale)."""


def apply_axis_map(nx: float, ny: float, args: Args) -> Tuple[float, float]:
    """Remap normalized VRX/VRY to screen left/right and up/down."""
    if args.swap_xy:
        nx, ny = ny, nx
    if args.invert_x:
        nx = -nx
    if args.invert_y:
        ny = -ny
    return nx, ny


def parse_args() -> "Args":
    parser = argparse.ArgumentParser(
        description="Test Arduino Nano joystick over USB serial (see joystick_test.ino)."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port of the Nano. Auto-detected if omitted.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (must match the Arduino sketch).",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw serial lines instead of the live dashboard.",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.08,
        help="Normalized magnitude below which an axis is treated as centered.",
    )
    parser.add_argument(
        "--bar-width",
        type=int,
        default=21,
        help="Character width of each axis bar.",
    )
    parser.add_argument(
        "--grid-size",
        type=int,
        default=15,
        help="Side length (chars) of the 2D position grid.",
    )
    parser.add_argument(
        "--no-swap-xy",
        action="store_true",
        help="Disable default axis swap (VRX->up/down, VRY->left/right).",
    )
    parser.add_argument(
        "--invert-x",
        action="store_true",
        help="Flip screen left/right.",
    )
    parser.add_argument(
        "--invert-y",
        action="store_true",
        help="Flip screen up/down.",
    )
    parser.add_argument(
        "--center-x",
        type=int,
        default=512,
        help="Resting ADC value for VRX/A0 (default 512).",
    )
    parser.add_argument(
        "--center-y",
        type=int,
        default=512,
        help="Resting ADC value for VRY/A1 (default 512).",
    )
    parser.add_argument(
        "--half-span",
        type=int,
        default=512,
        help="ADC counts from center to full deflection (default 512).",
    )
    ns = parser.parse_args()
    return Args(
        port=ns.port,
        baud=ns.baud,
        raw=ns.raw,
        deadzone=ns.deadzone,
        bar_width=ns.bar_width,
        grid_size=ns.grid_size,
        swap_xy=not ns.no_swap_xy,
        invert_x=ns.invert_x,
        invert_y=ns.invert_y,
        center_x=ns.center_x,
        center_y=ns.center_y,
        half_span=ns.half_span,
    )


def find_serial_ports() -> List[str]:
    """Return candidate serial ports, preferring known Arduino/clone chips."""
    preferred: List[str] = []
    others: List[str] = []
    for p in list_ports.comports():
        vid = getattr(p, "vid", None)
        if vid is not None and any(vid == known_vid for known_vid, _ in KNOWN_VID_PID):
            preferred.append(p.device)
        else:
            others.append(p.device)

    # Fall back to raw globs in case list_ports misses WSL/udev devices.
    globbed = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    for dev in globbed:
        if dev not in preferred and dev not in others:
            others.append(dev)

    return preferred + others


def resolve_port(requested: Optional[str]) -> str:
    if requested:
        return requested
    candidates = find_serial_ports()
    if not candidates:
        raise SystemExit(
            "No serial ports found. Connect the Nano (and on WSL, attach it with "
            "`usbipd attach --wsl --busid <BUSID>`), then retry. You can also pass "
            "--port explicitly."
        )
    if len(candidates) > 1:
        print("Multiple serial ports found:")
        for i, dev in enumerate(candidates):
            print(f"  [{i}] {dev}")
        print(f"Using {candidates[0]} (override with --port).")
    return candidates[0]


def parse_line(line: str) -> Optional[Tuple[int, int, int]]:
    parts = line.strip().split(",")
    if len(parts) != 3:
        return None
    try:
        x = int(parts[0])
        y = int(parts[1])
        sw = int(parts[2])
    except ValueError:
        return None
    if not (0 <= x <= ADC_MAX and 0 <= y <= ADC_MAX):
        return None
    return x, y, sw


def normalize_axis(raw: int, center: int, half_span: int, deadzone: float) -> float:
    """Map raw ADC to [-1, 1] using fixed center and span (no runtime learning)."""
    value = (raw - center) / max(half_span, 1)
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadzone:
        return 0.0
    return value


def render_bar(value: float, width: int) -> str:
    """Horizontal bar for a value in [-1, 1] with a center tick."""
    half = (width - 1) // 2
    filled = int(round(value * half))
    cells = []
    for i in range(-half, half + 1):
        if i == 0:
            cells.append("|")
        elif 0 < i <= filled:
            cells.append("=")
        elif filled <= i < 0:
            cells.append("=")
        else:
            cells.append(" ")
    return "[" + "".join(cells) + "]"


def render_grid(nx: float, ny: float, size: int) -> List[str]:
    """2D box with a marker showing the stick position; y-up."""
    col = int(round((nx + 1) / 2 * (size - 1)))
    row = int(round((1 - ny) / 2 * (size - 1)))  # invert so up = top
    col = max(0, min(size - 1, col))
    row = max(0, min(size - 1, row))
    lines = ["+" + "-" * size + "+"]
    for r in range(size):
        cells = []
        for c in range(size):
            if r == row and c == col:
                cells.append("o")
            elif r == size // 2 and c == size // 2:
                cells.append("+")
            elif r == size // 2:
                cells.append("-")
            elif c == size // 2:
                cells.append("|")
            else:
                cells.append(" ")
        lines.append("|" + "".join(cells) + "|")
    lines.append("+" + "-" * size + "+")
    return lines


def run_raw(ser: serial.Serial) -> None:
    print("Raw mode — printing serial lines. Ctrl+C to exit.")
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(line)


def run_dashboard(ser: serial.Serial, args: Args) -> None:
    samples = 0
    last_rate_t = time.time()
    last_rate_samples = 0
    rate_hz = 0.0
    button_latched = False

    sys.stdout.write("\033[2J")  # clear screen once
    while True:
        line = ser.readline().decode(errors="replace")
        parsed = parse_line(line)
        if parsed is None:
            continue
        x, y, sw = parsed
        nx_raw = normalize_axis(x, args.center_x, args.half_span, args.deadzone)
        ny_raw = normalize_axis(y, args.center_y, args.half_span, args.deadzone)
        nx, ny = apply_axis_map(nx_raw, ny_raw, args)
        pressed = sw == 0
        if pressed:
            button_latched = True

        samples += 1
        now = time.time()
        if now - last_rate_t >= 0.5:
            rate_hz = (samples - last_rate_samples) / (now - last_rate_t)
            last_rate_t = now
            last_rate_samples = samples

        grid = render_grid(nx, ny, args.grid_size)

        out = []
        map_note = []
        if args.swap_xy:
            map_note.append("swap_xy")
        if args.invert_x:
            map_note.append("invert_x")
        if args.invert_y:
            map_note.append("invert_y")
        map_str = ", ".join(map_note) if map_note else "none (VRX=right, VRY=up)"

        out.append("GELLO joystick test — {} @ {} baud".format(ser.port, args.baud))
        out.append("  axis map: {}".format(map_str))
        out.append("")
        out.append(
            "  VRX A0 {:4d}  raw {:+.2f}  {}".format(
                x, nx_raw, render_bar(nx_raw, args.bar_width)
            )
        )
        out.append(
            "  VRY A1 {:4d}  raw {:+.2f}  {}".format(
                y, ny_raw, render_bar(ny_raw, args.bar_width)
            )
        )
        out.append("")
        out.append(
            "  screen L/R {:+.2f}  {}".format(nx, render_bar(nx, args.bar_width))
        )
        out.append(
            "  screen U/D {:+.2f}  {}".format(ny, render_bar(ny, args.bar_width))
        )
        out.append("")
        out.append(
            "  Button (D2): {}   (ever pressed: {})".format(
                "PRESSED " if pressed else "released",
                "yes" if button_latched else "no",
            )
        )
        out.append("")
        for g in grid:
            out.append("   " + g)
        out.append("")
        out.append(
            "  center X={} Y={} span {}   rate {:.1f} Hz   Ctrl+C to exit".format(
                args.center_x, args.center_y, args.half_span, rate_hz
            )
        )

        sys.stdout.write("\033[H")  # cursor home; overwrite in place
        sys.stdout.write("\n".join(line.ljust(72) for line in out) + "\n")
        sys.stdout.flush()


def main(args: Args) -> None:
    port = resolve_port(args.port)
    print(f"Opening {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        raise SystemExit(f"Failed to open {port}: {e}")

    # Many Nano boards reset when the port opens; give the sketch time to boot.
    time.sleep(2.0)
    ser.reset_input_buffer()

    try:
        if args.raw:
            run_raw(ser)
        else:
            run_dashboard(ser, args)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main(parse_args())
