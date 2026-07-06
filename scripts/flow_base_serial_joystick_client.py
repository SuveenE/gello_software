#!/usr/bin/env python3
"""FlowBase / LinearBot client for Arduino Nano serial joystick(s).

Streams normalised base velocity commands over the network to
``flow_base_controller`` running with ``--gamepad false``, using the same RPC
path as ``i2rt/.../flow_base_joystick_client.py``.

Two control modes
-----------------
* **Single stick** (``--port`` / ``--left-port`` only): translation only.
      left up/down    -> forward/back   (user_cmd[0])
      left left/right -> strafe          (user_cmd[1])
      button          -> toggle local/global frame

* **Dual stick** (both ``--left-port`` and ``--right-port``): full 4-DOF,
  matching the USB gamepad.
      left  up/down    -> forward/back
      left  left/right -> strafe
      right left/right -> yaw / rotation (user_cmd[2])
      right up/down    -> linear rail (m/s, up = raise)
      left button      -> toggle local/global frame
      right button     -> reset odometry

Arduino sketch: scripts/arduino/joystick_test/joystick_test.ino

Telling the two sticks apart
----------------------------
If both USB adapters report the SAME serial number (common with clone FTDI /
CH340 chips), device paths cannot tell them apart. The robust fix is a firmware
id: flash each Nano with ``#define JOYSTICK_ID "LEFT"`` / ``"RIGHT"`` and let
the client assign roles by asking each board who it is::

    python3 scripts/flow_base_serial_joystick_client.py --host 192.168.50.91 --auto-id

Otherwise pin each role to a stable path (by-id is unique per chip; by-path is
unique per physical USB port and survives identical serials on Linux)::

    python3 scripts/flow_base_serial_joystick_client.py --list        # ports + ids
    python3 scripts/flow_base_serial_joystick_client.py \\
        --host 192.168.50.91 \\
        --left-port  /dev/serial/by-path/...-port0 \\
        --right-port /dev/serial/by-path/...-port0

Env fallbacks: ``LINEARBOT_HOST``, ``JOYSTICK_LEFT_PORT``, ``JOYSTICK_RIGHT_PORT``,
``JOYSTICK_LEFT_ID``, ``JOYSTICK_RIGHT_ID`` (``JOYSTICK_SERIAL_PORT`` is a legacy
alias for the single-stick port).

Preview mapping without driving the base:
    python3 scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import contextlib
import math
import os
import sys
import time

import numpy as np

# Allow running directly (python3 scripts/...) without `pip install -e .` by
# adding the gello_software repo root (parent of scripts/) to the import path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gello.utils.serial_joystick import (  # noqa: E402
    DEFAULT_LIFT_MAX_VEL_MS,
    DEFAULT_RIGHT_STICK_CONE_DEG,
    SerialJoystick,
    SerialJoystickConfig,
    format_port_table,
    resolve_ports_by_firmware_id,
    screen_to_yaw_rail,
)

BASE_DEFAULT_PORT = 11323
DEFAULT_SEND_HZ = 50.0
DEFAULT_LINEARBOT_HOST = os.environ.get("LINEARBOT_HOST", "192.168.50.91")


def _build_config(port, args, *, invert_x, invert_y, swap_xy) -> SerialJoystickConfig:
    return SerialJoystickConfig(
        port=port,
        baud=args.baud,
        cross_axis_cone_deg=args.cross_axis_cone_deg,
        swap_xy=swap_xy,
        invert_x=invert_x,
        invert_y=invert_y,
        center_x=args.center_x,
        center_y=args.center_y,
        half_span=args.half_span,
    )


def _run_single(args, client) -> None:
    port = args.left_port or args.port
    cfg = _build_config(
        port, args, invert_x=args.left_invert_x, invert_y=args.left_invert_y,
        swap_xy=not args.left_no_swap_xy,
    )
    joystick = SerialJoystick(cfg)
    print(f"Single stick on {joystick.port} @ {args.baud} baud (translation only)")
    print("Streaming to base. Button toggles local/global. Ctrl+C to stop.")

    frame = "local"
    last_button = False
    period = 1.0 / args.send_hz
    last_send = 0.0
    count = 0

    try:
        while True:
            sample = joystick.read_sample_blocking()
            user_cmd = sample.user_cmd

            if sample.button_pressed and not last_button:
                frame = "global" if frame == "local" else "local"
                print(f"\nFrame -> {frame}")
            last_button = sample.button_pressed

            now = time.time()
            if now - last_send >= period:
                client.set_target_velocity(
                    {"target_velocity": user_cmd, "frame": frame}
                ).result()
                last_send = now

            if count % 10 == 0:
                sys.stdout.write(
                    f"\r[{frame}] fwd:{user_cmd[0]:+.2f} strafe:{user_cmd[1]:+.2f} "
                    f"yaw:{user_cmd[2]:+.2f}    "
                )
                sys.stdout.flush()
            count += 1
    except KeyboardInterrupt:
        print("\nStopping, sending zero velocity...")
    finally:
        _stop(client)
        _close(joystick)


def _run_dual(args, client) -> None:
    right_cone_ratio = math.tan(math.radians(args.right_stick_cone_deg))

    left_cfg = _build_config(
        args.left_port, args, invert_x=args.left_invert_x,
        invert_y=args.left_invert_y, swap_xy=not args.left_no_swap_xy,
    )
    right_cfg = _build_config(
        args.right_port, args, invert_x=args.right_invert_x,
        invert_y=args.right_invert_y, swap_xy=not args.right_no_swap_xy,
    )
    left = SerialJoystick(left_cfg, timeout=0.05).start_background()
    right = SerialJoystick(right_cfg, timeout=0.05).start_background()
    print(f"Left  stick (translation)     on {left.port}")
    print(f"Right stick (yaw + rail m/s)  on {right.port}")
    print("Left button toggles frame, right button resets odometry. Ctrl+C to stop.")

    frame = "local"
    last_left_button = False
    last_right_button = False
    period = 1.0 / args.send_hz
    count = 0

    try:
        while True:
            time.sleep(period)
            ls = left.get_latest()
            rs = right.get_latest()

            fwd = ls.user_cmd[0] if ls is not None else 0.0
            strafe = ls.user_cmd[1] if ls is not None else 0.0

            yaw, rail_mps = 0.0, 0.0
            if rs is not None:
                yaw, rail_mps = screen_to_yaw_rail(
                    rs.screen_lr, rs.screen_ud,
                    cone_ratio=right_cone_ratio,
                    lift_max_vel_ms=args.lift_max_vel_ms,
                )

            if ls is not None and ls.button_pressed and not last_left_button:
                frame = "global" if frame == "local" else "local"
                print(f"\nFrame -> {frame}")
            if ls is not None:
                last_left_button = ls.button_pressed

            if rs is not None and rs.button_pressed and not last_right_button:
                try:
                    client.reset_odometry({}).result()
                    print("\nOdometry reset")
                except Exception as e:
                    print(f"\nreset_odometry failed: {e}")
            if rs is not None:
                last_right_button = rs.button_pressed

            target = np.array([fwd, strafe, yaw, rail_mps])
            client.set_target_velocity(
                {"target_velocity": target, "frame": frame}
            ).result()

            if count % 10 == 0:
                sys.stdout.write(
                    f"\r[{frame}] fwd:{fwd:+.2f} strafe:{strafe:+.2f} "
                    f"yaw:{yaw:+.2f} rail:{rail_mps:+.2f}m/s    "
                )
                sys.stdout.flush()
            count += 1
    except KeyboardInterrupt:
        print("\nStopping, sending zero velocity...")
    finally:
        _stop(client, num_dofs=4)
        _close(left)
        _close(right)


def _stop(client, num_dofs: int = 3) -> None:
    with contextlib.suppress(Exception):
        client.set_target_velocity(
            {"target_velocity": np.zeros(num_dofs), "frame": "local"}
        ).result()
    with contextlib.suppress(Exception):
        client.close(timeout=2.0)


def _close(joystick) -> None:
    with contextlib.suppress(Exception):
        joystick.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--host",
        type=str,
        default=DEFAULT_LINEARBOT_HOST,
        help=(
            "Host of the flow_base_controller RPC server "
            f"(default: LINEARBOT_HOST or {DEFAULT_LINEARBOT_HOST!r})."
        ),
    )
    parser.add_argument(
        "--rpc-port",
        type=int,
        default=BASE_DEFAULT_PORT,
        help=f"RPC port (default {BASE_DEFAULT_PORT}).",
    )
    parser.add_argument(
        "--port",
        default=os.environ.get("JOYSTICK_SERIAL_PORT"),
        help="Legacy single-stick serial port (alias for --left-port).",
    )
    parser.add_argument(
        "--left-port",
        default=os.environ.get("JOYSTICK_LEFT_PORT"),
        help="Serial port of the LEFT (translation) stick. Default: JOYSTICK_LEFT_PORT.",
    )
    parser.add_argument(
        "--right-port",
        default=os.environ.get("JOYSTICK_RIGHT_PORT"),
        help="Serial port of the RIGHT (yaw + rail) stick. Default: JOYSTICK_RIGHT_PORT.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List detected serial ports (with by-id / by-path + firmware id) and exit.",
    )
    parser.add_argument(
        "--auto-id",
        action="store_true",
        help=(
            "Assign left/right ports by querying each board's firmware JOYSTICK_ID "
            "(flash the sketch with a unique id per board first). Robust to "
            "identical USB serial numbers."
        ),
    )
    parser.add_argument(
        "--exclude-port",
        action="append",
        default=None,
        dest="exclude_ports",
        help=(
            "Serial device path(s) the discovery/--auto-id probe must never open "
            "(repeatable; matched by realpath so by-id/by-path aliases work). Use "
            "this to protect the GELLO arm FTDI adapters from being probed while "
            "their leader servers are running (otherwise you get 'comm failed -3001'). "
            "Also settable via JOYSTICK_EXCLUDE_PORTS (path- or comma-separated)."
        ),
    )
    parser.add_argument(
        "--no-auto-id-fallback",
        action="store_true",
        dest="no_auto_id_fallback",
        help=(
            "Disable the --auto-id fallback. By default, if exactly one stick is "
            "firmware-identified and exactly one other joystick port is left over, "
            "that leftover is assigned to the missing role (handles a board with a "
            "blank/old id). Pass this to require both ids to be reported."
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
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ)
    parser.add_argument(
        "--cross-axis-cone-deg",
        type=float,
        default=25.0,
        help="Left-stick cross-axis cone filter (same as USB gamepad path).",
    )
    parser.add_argument(
        "--right-stick-cone-deg",
        type=float,
        default=DEFAULT_RIGHT_STICK_CONE_DEG,
        help="Right-stick cardinal gate: rotation vs rail separation cone.",
    )
    parser.add_argument(
        "--lift-max-vel-ms",
        type=float,
        default=DEFAULT_LIFT_MAX_VEL_MS,
        help="Right-stick up/down full deflection -> linear rail speed (m/s).",
    )
    # Shared normalisation.
    parser.add_argument("--center-x", type=int, default=512)
    parser.add_argument("--center-y", type=int, default=512)
    parser.add_argument("--half-span", type=int, default=512)
    # Per-stick axis orientation.
    parser.add_argument("--left-no-swap-xy", action="store_true")
    parser.add_argument("--left-invert-x", action="store_true")
    parser.add_argument("--left-invert-y", action="store_true")
    parser.add_argument("--right-no-swap-xy", action="store_true")
    parser.add_argument("--right-invert-x", action="store_true")
    parser.add_argument("--right-invert-y", action="store_true")
    # Back-compat single-stick aliases (apply to the left/only stick).
    parser.add_argument(
        "--no-swap-xy", action="store_true", help="Alias for --left-no-swap-xy."
    )
    parser.add_argument("--invert-x", action="store_true", help="Alias for --left-invert-x.")
    parser.add_argument("--invert-y", action="store_true", help="Alias for --left-invert-y.")
    args = parser.parse_args()

    args.left_no_swap_xy = args.left_no_swap_xy or args.no_swap_xy
    args.left_invert_x = args.left_invert_x or args.invert_x
    args.left_invert_y = args.left_invert_y or args.invert_y

    exclude_ports = list(args.exclude_ports or [])
    env_exclude = os.environ.get("JOYSTICK_EXCLUDE_PORTS", "")
    for chunk in env_exclude.replace(os.pathsep, ",").split(","):
        chunk = chunk.strip()
        if chunk:
            exclude_ports.append(chunk)
    if exclude_ports:
        print(f"Excluding serial ports from discovery/probe: {exclude_ports}")

    if args.list:
        print("Probing firmware ids (this resets each board briefly)...")
        print(format_port_table(probe_ids=True, exclude=exclude_ports))
        return

    if args.auto_id:
        print(f"Resolving ports by firmware id ({args.left_id!r}, {args.right_id!r})...")
        found = resolve_ports_by_firmware_id(
            [args.left_id, args.right_id],
            baud=args.baud,
            exclude=exclude_ports,
            fallback_single=not args.no_auto_id_fallback,
        )
        args.left_port = found.get(args.left_id, args.left_port)
        args.right_port = found.get(args.right_id, args.right_port)
        if not args.left_port:
            raise SystemExit(
                f"Could not find a board reporting id {args.left_id!r}. "
                f"Flash it with #define JOYSTICK_ID \"{args.left_id}\" and retry, "
                f"or run --list to see reported ids."
            )
        print(f"  {args.left_id} -> {args.left_port}")
        if args.right_port:
            print(f"  {args.right_id} -> {args.right_port}")

    try:
        import portal
    except ImportError as e:
        raise SystemExit("portal is required: pip install portal") from e

    client = portal.Client(f"{args.host}:{args.rpc_port}")
    print(f"Connected to flow_base_controller at {args.host}:{args.rpc_port}")

    dual = bool(args.left_port and args.right_port)
    if dual:
        _run_dual(args, client)
    else:
        _run_single(args, client)


if __name__ == "__main__":
    main()
