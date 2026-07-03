#!/usr/bin/env python3
"""Test Arduino serial joystick mapped to FlowBase / LinearBot commands.

Reads CSV lines from scripts/arduino/joystick_test/joystick_test.ino (or the
GELLO copy at gello_software/scripts/arduino/joystick_test/) at 115200 baud:

    <vrx>,<vry>,<sw>\\n

Applies the same axis remap as gello_software/scripts/test_joystick.py
(swap_xy by default), then converts to the normalised user_cmd that
flow_base_controller expects from a USB gamepad left stick:

    user_cmd = [forward, strafe_right, yaw]   each in [-1, 1]

By default this is **dashboard-only** (no robot motion). Pass ``--host`` to
stream commands to a ``flow_base_controller`` running with ``--no-gamepad``.

Usage:
    cd ~/lerobot/i2rt
    python3 scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0

    # Or from repo root:
    python3 i2rt/scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import glob
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import serial
from serial.tools import list_ports

# Allow: cd ~/lerobot/i2rt && python3 scripts/test_serial_joystick_flowbase.py ...
_I2RT_ROOT = Path(__file__).resolve().parents[1]
if str(_I2RT_ROOT) not in sys.path:
    sys.path.insert(0, str(_I2RT_ROOT))

BASE_DEFAULT_PORT = 11323  # same as flow_base_controller.py


def apply_axis_dominance(x: float, y: float, cone_ratio: float) -> tuple[float, float]:
    """Same filter as i2rt.utils.gamepad_utils (left-stick cross-talk suppression)."""
    if cone_ratio <= 0.0:
        return x, y
    ax, ay = abs(x), abs(y)
    if ax >= ay:
        if ay < cone_ratio * ax:
            y = 0.0
    elif ax < cone_ratio * ay:
        x = 0.0
    return x, y

ADC_MAX = 1023
ADC_CENTER = 512
ADC_HALF_SPAN = 512

# Match flow_base_controller.py defaults.
BASE_DEADZONE = 0.05
CROSS_AXIS_CONE_DEG = 25.0
DEFAULT_MAX_VEL = np.array([0.5, 0.5, np.pi / 2])  # m/s, m/s, rad/s
DEFAULT_SEND_HZ = 50.0


@dataclass
class Args:
    port: Optional[str] = None
    baud: int = 115200
    host: Optional[str] = None
    rpc_port: int = BASE_DEFAULT_PORT
    send_hz: float = DEFAULT_SEND_HZ
    deadzone: float = BASE_DEADZONE
    cross_axis_cone_deg: float = CROSS_AXIS_CONE_DEG
    swap_xy: bool = True
    invert_x: bool = False
    invert_y: bool = False
    bar_width: int = 21
    grid_size: int = 15
    center_x: int = ADC_CENTER
    center_y: int = ADC_CENTER
    half_span: int = ADC_HALF_SPAN


def parse_args() -> Args:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default=None, help="Serial port of the Arduino Nano.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--host",
        default=None,
        help="If set, stream normalised commands to flow_base_controller at this host.",
    )
    parser.add_argument(
        "--rpc-port",
        type=int,
        default=BASE_DEFAULT_PORT,
        help=f"RPC port (default {BASE_DEFAULT_PORT}).",
    )
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ)
    parser.add_argument(
        "--deadzone",
        type=float,
        default=BASE_DEADZONE,
        help="Normalised deadzone (flowbase uses 0.05).",
    )
    parser.add_argument(
        "--cross-axis-cone-deg",
        type=float,
        default=CROSS_AXIS_CONE_DEG,
        help="Left-stick cross-axis cone filter (same as gamepad, default 25 deg).",
    )
    parser.add_argument(
        "--no-swap-xy",
        action="store_true",
        help="Disable default axis swap (VRX->up/down, VRY->left/right).",
    )
    parser.add_argument("--invert-x", action="store_true", help="Flip screen left/right.")
    parser.add_argument("--invert-y", action="store_true", help="Flip screen up/down.")
    parser.add_argument("--bar-width", type=int, default=21)
    parser.add_argument("--grid-size", type=int, default=15)
    parser.add_argument("--center-x", type=int, default=ADC_CENTER)
    parser.add_argument("--center-y", type=int, default=ADC_CENTER)
    parser.add_argument("--half-span", type=int, default=ADC_HALF_SPAN)
    ns = parser.parse_args()
    return Args(
        port=ns.port,
        baud=ns.baud,
        host=ns.host,
        rpc_port=ns.rpc_port,
        send_hz=ns.send_hz,
        deadzone=ns.deadzone,
        cross_axis_cone_deg=ns.cross_axis_cone_deg,
        swap_xy=not ns.no_swap_xy,
        invert_x=ns.invert_x,
        invert_y=ns.invert_y,
        bar_width=ns.bar_width,
        grid_size=ns.grid_size,
        center_x=ns.center_x,
        center_y=ns.center_y,
        half_span=ns.half_span,
    )


def find_serial_ports() -> List[str]:
    preferred: List[str] = []
    others: List[str] = []
    for p in list_ports.comports():
        vid = getattr(p, "vid", None)
        if vid in (0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4):
            preferred.append(p.device)
        else:
            others.append(p.device)
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if dev not in preferred and dev not in others:
            others.append(dev)
    return preferred + others


def resolve_port(requested: Optional[str]) -> str:
    if requested:
        return requested
    candidates = find_serial_ports()
    if not candidates:
        raise SystemExit("No serial ports found. Pass --port explicitly.")
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
        x, y, sw = int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError:
        return None
    if not (0 <= x <= ADC_MAX and 0 <= y <= ADC_MAX):
        return None
    return x, y, sw


def normalize_axis(raw: int, center: int, half_span: int, deadzone: float) -> float:
    value = (raw - center) / max(half_span, 1)
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadzone:
        return 0.0
    return value


def apply_axis_map(nx: float, ny: float, args: Args) -> Tuple[float, float]:
    if args.swap_xy:
        nx, ny = ny, nx
    if args.invert_x:
        nx = -nx
    if args.invert_y:
        ny = -ny
    return nx, ny


def screen_to_flowbase(
    screen_lr: float,
    screen_ud: float,
    cone_ratio: float,
    deadzone: float,
) -> np.ndarray:
    """Map screen-left/right + screen-up/down to flowbase [forward, strafe, yaw].

    Matches gamepad_utils.Gamepad.get_user_cmd() for the LEFT stick only:
      forward  = screen up/down  (like -SDL left-Y)
      strafe   = screen left/right (like SDL left-X)
      yaw      = 0 (single-stick Arduino has no right stick)
    """
    forward, strafe = screen_ud, screen_lr
    forward, strafe = apply_axis_dominance(forward, strafe, cone_ratio)
    cmd = np.array([forward, strafe, 0.0])
    cmd[np.abs(cmd) < deadzone] = 0.0
    return cmd


def render_bar(value: float, width: int) -> str:
    half = (width - 1) // 2
    filled = int(round(value * half))
    cells = []
    for i in range(-half, half + 1):
        if i == 0:
            cells.append("|")
        elif 0 < i <= filled or filled <= i < 0:
            cells.append("=")
        else:
            cells.append(" ")
    return "[" + "".join(cells) + "]"


def render_robot_grid(fwd: float, strafe: float, size: int) -> List[str]:
    """Top-down robot view: up = forward, right = strafe right."""
    col = int(round((strafe + 1) / 2 * (size - 1)))
    row = int(round((1 - fwd) / 2 * (size - 1)))
    col = max(0, min(size - 1, col))
    row = max(0, min(size - 1, row))
    lines = ["+" + "-" * size + "+"]
    for r in range(size):
        cells = []
        for c in range(size):
            if r == row and c == col:
                cells.append("R")
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


def main() -> None:
    args = parse_args()
    port = resolve_port(args.port)
    cone_ratio = np.tan(np.radians(args.cross_axis_cone_deg))

    rpc_client = None
    if args.host:
        import portal

        rpc_client = portal.Client(f"{args.host}:{args.rpc_port}")
        print(f"RPC -> flow_base_controller at {args.host}:{args.rpc_port}")
    else:
        print("Dashboard-only mode (pass --host to drive the base).")

    print(f"Opening {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        raise SystemExit(f"Failed to open {port}: {e}")

    time.sleep(2.0)
    ser.reset_input_buffer()

    frame = "local"
    last_button = False
    samples = 0
    last_rate_t = time.time()
    last_rate_samples = 0
    rate_hz = 0.0
    period = 1.0 / args.send_hz
    last_send = 0.0

    sys.stdout.write("\033[2J")
    try:
        while True:
            line = ser.readline().decode(errors="replace")
            parsed = parse_line(line)
            if parsed is None:
                continue
            x, y, sw = parsed

            nx_raw = normalize_axis(x, args.center_x, args.half_span, args.deadzone)
            ny_raw = normalize_axis(y, args.center_y, args.half_span, args.deadzone)
            screen_lr, screen_ud = apply_axis_map(nx_raw, ny_raw, args)
            user_cmd = screen_to_flowbase(screen_lr, screen_ud, cone_ratio, BASE_DEADZONE)
            scaled_vel = user_cmd * DEFAULT_MAX_VEL

            pressed = sw == 0
            if pressed and not last_button:
                frame = "global" if frame == "local" else "local"
            last_button = pressed

            now = time.time()
            samples += 1
            if now - last_rate_t >= 0.5:
                rate_hz = (samples - last_rate_samples) / (now - last_rate_t)
                last_rate_t = now
                last_rate_samples = samples

            if rpc_client is not None and now - last_send >= period:
                rpc_client.set_target_velocity(
                    {"target_velocity": user_cmd, "frame": frame}
                ).result()
                last_send = now

            grid = render_robot_grid(user_cmd[0], user_cmd[1], args.grid_size)
            mode = "LIVE" if rpc_client else "preview"

            out = [
                f"FlowBase serial joystick test [{mode}]  {port} @ {args.baud}",
                f"  frame: {frame}   button toggles local/global on press",
                "",
                f"  VRX A0 {x:4d}  norm {nx_raw:+.2f}  {render_bar(nx_raw, args.bar_width)}",
                f"  VRY A1 {y:4d}  norm {ny_raw:+.2f}  {render_bar(ny_raw, args.bar_width)}",
                "",
                f"  screen L/R {screen_lr:+.2f}   U/D {screen_ud:+.2f}",
                "",
                "  flowbase user_cmd (normalised, like gamepad left stick):",
                f"    forward {user_cmd[0]:+.2f}  strafe {user_cmd[1]:+.2f}  yaw {user_cmd[2]:+.2f}",
                "  scaled physical (controller max_vel):",
                f"    vx {scaled_vel[0]:+.2f} m/s   vy {scaled_vel[1]:+.2f} m/s   "
                f"vtheta {scaled_vel[2]:+.2f} rad/s",
                "",
                "  robot top-down (up=forward, right=strafe right):",
            ]
            out.extend("   " + g for g in grid)
            out.append("")
            out.append(
                f"  center X={args.center_x} Y={args.center_y} span {args.half_span}   "
                f"rate {rate_hz:.1f} Hz   Ctrl+C to exit"
            )

            sys.stdout.write("\033[H")
            sys.stdout.write("\n".join(line.ljust(78) for line in out) + "\n")
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if rpc_client is not None:
            try:
                rpc_client.set_target_velocity(
                    {"target_velocity": np.zeros(3), "frame": "local"}
                ).result()
            except Exception:
                pass
            try:
                rpc_client.close(timeout=2.0)
            except Exception:
                pass
        ser.close()


if __name__ == "__main__":
    main()
