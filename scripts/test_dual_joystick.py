#!/usr/bin/env python3
"""Test TWO Arduino Nano joysticks (LEFT + RIGHT) at the same time.

Reads both sticks concurrently and renders a side-by-side live dashboard so you
can verify each board, its axis wiring, and its button in one view. Pairs with
scripts/arduino/joystick_test/joystick_test.ino, which streams CSV
``<x>,<y>,<sw>\\n`` at 115200 baud and answers a ``?`` byte with a
``# ID:<LEFT|RIGHT>`` banner.

Telling the two sticks apart
----------------------------
Flash each Nano with ``#define JOYSTICK_ID "LEFT"`` / ``"RIGHT"`` (see the top of
joystick_test.ino). When ``--left-port`` / ``--right-port`` are not set, roles
are assigned by firmware id automatically::

    cd ~/lerobot/gello_software
    python3 scripts/test_dual_joystick.py

Or pin each role to a device path explicitly (robust to identical USB serials)::

    python3 scripts/test_dual_joystick.py --list            # ports + firmware ids
    python3 scripts/test_dual_joystick.py \\
        --left-port  /dev/serial/by-path/...-port0 \\
        --right-port /dev/serial/by-path/...-port0

Env fallbacks: ``JOYSTICK_LEFT_PORT``, ``JOYSTICK_RIGHT_PORT``,
``JOYSTICK_LEFT_ID`` (default LEFT), ``JOYSTICK_RIGHT_ID`` (default RIGHT).

Axis orientation
----------------
The sticks are assumed to be mounted upside down, so ``--reverse`` is ON by
default and negates both axes on both sticks (a 180 deg in-plane rotation),
matching flow_base_serial_joystick_client.py. Pass ``--no-reverse`` for a
normal mount, or override a single axis with ``--no-left-invert-x`` etc.

Move each stick to test. Ctrl+C to exit.

WSL note: USB serial devices must be attached to WSL with `usbipd` on Windows,
e.g. `usbipd attach --wsl --busid <BUSID>`, before the ports appear here.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Optional

# Allow running directly (python3 scripts/...) without `pip install -e .` by
# adding the gello_software repo root (parent of scripts/) to the import path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.utils.serial_joystick import (  # noqa: E402
    SerialJoystick,
    SerialJoystickConfig,
    SerialJoystickSample,
    find_serial_ports,
    format_port_table,
    resolve_ports_by_firmware_id,
)


def render_bar(value: float, width: int) -> str:
    """Horizontal bar for a value in [-1, 1] with a center tick."""
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


def render_grid(nx: float, ny: float, size: int) -> List[str]:
    """2D box with a marker showing the stick position; y-up."""
    col = int(round((nx + 1) / 2 * (size - 1)))
    row = int(round((1 - ny) / 2 * (size - 1)))
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


def render_stick_panel(
    title: str,
    port: str,
    sample: Optional[SerialJoystickSample],
    latched: bool,
    rate_hz: float,
    args: "Args",
) -> List[str]:
    """One stick's column of dashboard text (fixed width for side-by-side)."""
    lines = [f"{title}  {port}"]
    if sample is None:
        lines.append("  (waiting for data...)")
        # Pad to the same height as a populated panel so columns stay aligned.
        while len(lines) < 8 + args.grid_size:
            lines.append("")
        return lines

    nx = sample.screen_lr
    ny = sample.screen_ud
    pressed = sample.button_pressed
    lines.append(
        f"  VRX A0 {sample.raw_x:4d}   VRY A1 {sample.raw_y:4d}   {rate_hz:5.1f} Hz"
    )
    lines.append("")
    lines.append(f"  L/R {nx:+.2f} {render_bar(nx, args.bar_width)}")
    lines.append(f"  U/D {ny:+.2f} {render_bar(ny, args.bar_width)}")
    lines.append("")
    lines.append(
        "  Button: {}   (ever: {})".format(
            "PRESSED " if pressed else "released",
            "yes" if latched else "no ",
        )
    )
    lines.append("")
    lines.extend("   " + g for g in render_grid(nx, ny, args.grid_size))
    return lines


def join_columns(left: List[str], right: List[str], gap: int = 4) -> List[str]:
    """Merge two text blocks into side-by-side columns of equal height."""
    height = max(len(left), len(right))
    left = left + [""] * (height - len(left))
    right = right + [""] * (height - len(right))
    left_w = max((len(s) for s in left), default=0)
    sep = " " * gap
    return [f"{l.ljust(left_w)}{sep}{r}" for l, r in zip(left, right)]


def build_config(port: Optional[str], args: "Args", *, invert_x: bool,
                 invert_y: bool, swap_xy: bool) -> SerialJoystickConfig:
    return SerialJoystickConfig(
        port=port,
        baud=args.baud,
        swap_xy=swap_xy,
        invert_x=invert_x,
        invert_y=invert_y,
        center_x=args.center_x,
        center_y=args.center_y,
        half_span=args.half_span,
    )


class Args(argparse.Namespace):
    pass


def describe_reverse(args: "Args") -> str:
    """``on``/``off``, or the effective per-stick inverts when they diverge."""
    inverts = (
        args.left_invert_x, args.left_invert_y,
        args.right_invert_x, args.right_invert_y,
    )
    if all(inverts):
        return "on"
    if not any(inverts):
        return "off"
    return "L{}{} R{}{}".format(
        "x" if args.left_invert_x else "-",
        "y" if args.left_invert_y else "-",
        "x" if args.right_invert_x else "-",
        "y" if args.right_invert_y else "-",
    )


def parse_args() -> Args:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--left-port",
        default=os.environ.get("JOYSTICK_LEFT_PORT"),
        help="Serial port of the LEFT stick. Default: JOYSTICK_LEFT_PORT env.",
    )
    parser.add_argument(
        "--right-port",
        default=os.environ.get("JOYSTICK_RIGHT_PORT"),
        help="Serial port of the RIGHT stick. Default: JOYSTICK_RIGHT_PORT env.",
    )
    parser.add_argument(
        "--auto-id",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Assign left/right ports by querying each board's firmware "
            "JOYSTICK_ID. Default: on when --left-port/--right-port are not "
            "both set (USB enumeration order is not reliable)."
        ),
    )
    parser.add_argument(
        "--left-id",
        default=os.environ.get("JOYSTICK_LEFT_ID", "LEFT"),
        help="Firmware id of the LEFT stick for --auto-id (default: LEFT).",
    )
    parser.add_argument(
        "--right-id",
        default=os.environ.get("JOYSTICK_RIGHT_ID", "RIGHT"),
        help="Firmware id of the RIGHT stick for --auto-id (default: RIGHT).",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected serial ports (with by-id/by-path + firmware id) and exit.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--refresh-hz", type=float, default=30.0, help="Dashboard refresh rate."
    )
    parser.add_argument("--bar-width", type=int, default=21)
    parser.add_argument("--grid-size", type=int, default=13)
    # Shared normalisation.
    parser.add_argument("--center-x", type=int, default=512)
    parser.add_argument("--center-y", type=int, default=512)
    parser.add_argument("--half-span", type=int, default=512)
    # Axis orientation. --reverse is the baseline: the sticks are mounted upside
    # down, so both axes are negated (a 180 deg in-plane rotation). The per-stick
    # flags default to following --reverse and can override it one axis at a time.
    parser.add_argument(
        "--reverse", action=argparse.BooleanOptionalAction, default=True,
        help=(
            "Invert both axes on both sticks (default: on for upside-down "
            "mount; --no-reverse disables)."
        ),
    )
    parser.add_argument("--left-no-swap-xy", action="store_true")
    parser.add_argument(
        "--left-invert-x", action=argparse.BooleanOptionalAction, default=None,
        help="Invert left-stick X (default: follows --reverse).",
    )
    parser.add_argument(
        "--left-invert-y", action=argparse.BooleanOptionalAction, default=None,
        help="Invert left-stick Y (default: follows --reverse).",
    )
    parser.add_argument("--right-no-swap-xy", action="store_true")
    parser.add_argument(
        "--right-invert-x", action=argparse.BooleanOptionalAction, default=None,
        help="Invert right-stick X (default: follows --reverse).",
    )
    parser.add_argument(
        "--right-invert-y", action=argparse.BooleanOptionalAction, default=None,
        help="Invert right-stick Y (default: follows --reverse).",
    )
    args = parser.parse_args(namespace=Args())
    for name in ("left_invert_x", "left_invert_y", "right_invert_x", "right_invert_y"):
        if getattr(args, name) is None:
            setattr(args, name, args.reverse)
    return args


def resolve_ports(args: Args) -> None:
    # USB enumeration order is not LEFT/RIGHT — prefer firmware ids whenever
    # both ports are not pinned. Explicit --auto-id / --no-auto-id wins.
    use_auto_id = (
        args.auto_id
        if args.auto_id is not None
        else not (args.left_port and args.right_port)
    )
    if use_auto_id:
        print(
            f"Resolving ports by firmware id ({args.left_id!r}, {args.right_id!r})..."
        )
        found = resolve_ports_by_firmware_id(
            [args.left_id, args.right_id], baud=args.baud, fallback_single=True
        )
        args.left_port = found.get(args.left_id, args.left_port)
        args.right_port = found.get(args.right_id, args.right_port)
        print(f"  {args.left_id} -> {args.left_port}")
        print(f"  {args.right_id} -> {args.right_port}")

    # Last resort only: first two candidates (order is not role-correct).
    if not args.left_port or not args.right_port:
        candidates = [
            c
            for c in find_serial_ports()
            if c not in (args.left_port, args.right_port)
        ]
        if not args.left_port and candidates:
            args.left_port = candidates.pop(0)
        if not args.right_port and candidates:
            args.right_port = candidates.pop(0)
        print(
            "Warning: assigned remaining stick(s) by USB order, not firmware id. "
            "Prefer flashing JOYSTICK_ID LEFT/RIGHT or pass --left-port/--right-port.",
            file=sys.stderr,
        )

    if not args.left_port or not args.right_port:
        raise SystemExit(
            "Could not resolve two joystick ports. Connect both boards "
            "(firmware ids LEFT/RIGHT), or pass --left-port and --right-port "
            "explicitly (run --list to see options)."
        )


def make_rate_tracker():
    state = {"samples": 0, "t": time.time(), "n": 0, "hz": 0.0}

    def update(count: int) -> float:
        now = time.time()
        if now - state["t"] >= 0.5:
            state["hz"] = (count - state["n"]) / (now - state["t"])
            state["t"] = now
            state["n"] = count
        return state["hz"]

    return update


def main() -> None:
    args = parse_args()

    if args.list:
        print("Probing firmware ids (this resets each board briefly)...")
        print(format_port_table(probe_ids=True))
        return

    resolve_ports(args)

    left = SerialJoystick(
        build_config(
            args.left_port, args, invert_x=args.left_invert_x,
            invert_y=args.left_invert_y, swap_xy=not args.left_no_swap_xy,
        ),
        timeout=0.05,
    ).start_background()
    right = SerialJoystick(
        build_config(
            args.right_port, args, invert_x=args.right_invert_x,
            invert_y=args.right_invert_y, swap_xy=not args.right_no_swap_xy,
        ),
        timeout=0.05,
    ).start_background()

    print(f"LEFT  on {left.port}")
    print(f"RIGHT on {right.port}")
    print("Move each stick to test. Ctrl+C to exit.")
    time.sleep(0.5)

    left_latched = False
    right_latched = False
    left_count = 0
    right_count = 0
    left_rate = make_rate_tracker()
    right_rate = make_rate_tracker()
    period = 1.0 / max(args.refresh_hz, 1.0)

    sys.stdout.write("\033[2J")  # clear screen once
    try:
        while True:
            ls = left.get_latest()
            rs = right.get_latest()
            if ls is not None:
                left_count += 1
                if ls.button_pressed:
                    left_latched = True
            if rs is not None:
                right_count += 1
                if rs.button_pressed:
                    right_latched = True

            left_panel = render_stick_panel(
                "LEFT ", left.port, ls, left_latched, left_rate(left_count), args
            )
            right_panel = render_stick_panel(
                "RIGHT", right.port, rs, right_latched, right_rate(right_count), args
            )

            out = ["GELLO dual joystick test — LEFT | RIGHT @ {} baud".format(args.baud), ""]
            out.extend(join_columns(left_panel, right_panel))
            out.append("")
            out.append(
                "  center X={} Y={} span {}   reverse {}   Ctrl+C to exit".format(
                    args.center_x, args.center_y, args.half_span,
                    describe_reverse(args),
                )
            )

            sys.stdout.write("\033[H")  # cursor home; overwrite in place
            width = max((len(line) for line in out), default=0)
            sys.stdout.write("\n".join(line.ljust(width) for line in out) + "\n")
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        left.close()
        right.close()


if __name__ == "__main__":
    main()
