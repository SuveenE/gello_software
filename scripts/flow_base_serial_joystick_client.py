#!/usr/bin/env python3
"""FlowBase / LinearBot client for an Arduino Nano serial joystick.

Streams normalised base velocity commands over the network to
``flow_base_controller`` running with ``--no-gamepad``, using the same RPC
path as ``i2rt/.../flow_base_joystick_client.py``.

Axis mapping matches ``scripts/test_joystick.py`` (swap_xy baked in):
  screen up/down   -> forward/back
  screen left/right -> strafe
  stick button press -> toggle local/global frame

Arduino sketch: scripts/arduino/joystick_test/joystick_test.ino

Example
-------
On the base PC:
    python3 i2rt/i2rt/flow_base/flow_base_controller.py --no-gamepad ...

On the operator PC with the Nano plugged in:
    python3 scripts/flow_base_serial_joystick_client.py \\
        --host 172.6.2.20 \\
        --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0

Preview mapping without driving the base:
    python3 scripts/test_serial_joystick_flowbase.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from gello.utils.serial_joystick import SerialJoystick, SerialJoystickConfig

BASE_DEFAULT_PORT = 11323
DEFAULT_SEND_HZ = 50.0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--host",
        type=str,
        default="localhost",
        help="Host of the flow_base_controller RPC server.",
    )
    parser.add_argument(
        "--rpc-port",
        type=int,
        default=BASE_DEFAULT_PORT,
        help=f"RPC port (default {BASE_DEFAULT_PORT}).",
    )
    parser.add_argument("--port", default=None, help="Serial port of the Arduino Nano.")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--send-hz", type=float, default=DEFAULT_SEND_HZ)
    parser.add_argument(
        "--no-swap-xy",
        action="store_true",
        help="Disable default axis swap (VRX->up/down, VRY->left/right).",
    )
    parser.add_argument("--invert-x", action="store_true")
    parser.add_argument("--invert-y", action="store_true")
    parser.add_argument("--center-x", type=int, default=512)
    parser.add_argument("--center-y", type=int, default=512)
    parser.add_argument("--half-span", type=int, default=512)
    parser.add_argument(
        "--cross-axis-cone-deg",
        type=float,
        default=25.0,
        help="Left-stick cross-axis cone filter (same as USB gamepad path).",
    )
    args = parser.parse_args()

    try:
        import portal
    except ImportError as e:
        raise SystemExit("portal is required: pip install portal") from e

    client = portal.Client(f"{args.host}:{args.rpc_port}")
    print(f"Connected to flow_base_controller at {args.host}:{args.rpc_port}")

    joy_cfg = SerialJoystickConfig(
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
    joystick = SerialJoystick(joy_cfg)
    print(f"Serial joystick on {joystick.port} @ {args.baud} baud")
    print("Streaming to base. Stick button toggles local/global. Ctrl+C to stop.")

    frame = "local"
    last_button = False
    period = 1.0 / args.send_hz
    last_send = 0.0
    count = 0
    user_cmd = np.zeros(3)

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
        try:
            client.set_target_velocity(
                {"target_velocity": np.zeros(3), "frame": "local"}
            ).result()
        except Exception:
            pass
        try:
            joystick.close()
        except Exception:
            pass
        try:
            client.close(timeout=2.0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
