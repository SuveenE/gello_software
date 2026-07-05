#!/usr/bin/env python3
"""Preview Arduino serial joystick mapping for FlowBase (dashboard only).

Uses the same mapping as flow_base_serial_joystick_client.py. Pass ``--host``
to stream commands to a running flow_base_controller.

Usage:
    cd ~/lerobot/gello_software
    python3 scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0

    python3 scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0 --host 172.6.2.20
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List

import numpy as np

# Allow running directly (python3 scripts/...) without `pip install -e .` by
# adding the gello_software repo root (parent of scripts/) to the import path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.utils.serial_joystick import (  # noqa: E402
    FLOWBASE_DEADZONE,
    SerialJoystick,
    SerialJoystickConfig,
)

BASE_DEFAULT_PORT = 11323
DEFAULT_MAX_VEL = np.array([0.5, 0.5, np.pi / 2])
DEFAULT_SEND_HZ = 50.0


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default=None)
    parser.add_argument("--rpc-port", type=int, default=BASE_DEFAULT_PORT)
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ)
    parser.add_argument("--no-swap-xy", action="store_true")
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--center-x", type=int, default=512)
    parser.add_argument("--center-y", type=int, default=512)
    parser.add_argument("--half-span", type=int, default=512)
    parser.add_argument("--cross-axis-cone-deg", type=float, default=25.0)
    parser.add_argument("--bar-width", type=int, default=21)
    parser.add_argument("--grid-size", type=int, default=15)
    args = parser.parse_args()

    rpc_client = None
    if args.host:
        import portal

        rpc_client = portal.Client(f"{args.host}:{args.rpc_port}")
        print(f"RPC -> flow_base_controller at {args.host}:{args.rpc_port}")
    else:
        print("Dashboard-only (pass --host to drive the base).")

    cfg = SerialJoystickConfig(
        port=args.port,
        baud=args.baud,
        cross_axis_cone_deg=args.cross_axis_cone_deg,
        swap_xy=not args.no_swap_xy,
        invert_x=args.invert_x,
        invert_y=args.invert_y,
        center_x=args.center_x,
        center_y=args.center_y,
        half_span=args.half_span,
    )
    joystick = SerialJoystick(cfg)
    print(f"Opened {joystick.port} @ {args.baud} baud")

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
            sample = joystick.read_sample_blocking()
            user_cmd = sample.user_cmd
            scaled_vel = user_cmd * DEFAULT_MAX_VEL

            if sample.button_pressed and not last_button:
                frame = "global" if frame == "local" else "local"
            last_button = sample.button_pressed

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

            nx_raw = (sample.raw_x - args.center_x) / max(args.half_span, 1)
            ny_raw = (sample.raw_y - args.center_y) / max(args.half_span, 1)
            nx_raw = max(-1.0, min(1.0, nx_raw))
            ny_raw = max(-1.0, min(1.0, ny_raw))
            grid = render_robot_grid(user_cmd[0], user_cmd[1], args.grid_size)
            mode = "LIVE" if rpc_client else "preview"

            out = [
                f"FlowBase serial joystick [{mode}]  {joystick.port} @ {args.baud}",
                f"  frame: {frame}   button toggles local/global",
                "",
                f"  VRX A0 {sample.raw_x:4d}  {render_bar(nx_raw, args.bar_width)}",
                f"  VRY A1 {sample.raw_y:4d}  {render_bar(ny_raw, args.bar_width)}",
                "",
                f"  screen L/R {sample.screen_lr:+.2f}   U/D {sample.screen_ud:+.2f}",
                "",
                "  flowbase user_cmd:",
                f"    forward {user_cmd[0]:+.2f}  strafe {user_cmd[1]:+.2f}  yaw {user_cmd[2]:+.2f}",
                "  scaled (max_vel):",
                f"    vx {scaled_vel[0]:+.2f} m/s  vy {scaled_vel[1]:+.2f} m/s  "
                f"vtheta {scaled_vel[2]:+.2f} rad/s",
                "",
                "  robot top-down (up=forward):",
            ]
            out.extend("   " + g for g in grid)
            out.append("")
            out.append(
                f"  center {args.center_x}/{args.center_y} span {args.half_span}  "
                f"dz {FLOWBASE_DEADZONE}  rate {rate_hz:.1f} Hz  Ctrl+C exit"
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
        joystick.close()


if __name__ == "__main__":
    main()
