"""Read GELLO trigger (joint 7) open/close positions and print gripper_config.

Usage (left arm example):
  python scripts/calibrate_gripper.py \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WA0-if00-port0

With the trigger released, press Enter to capture the open position. Then fully
squeeze the trigger and press Enter again to capture the closed position.
"""

import os
import sys
from dataclasses import dataclass

import numpy as np
import tyro

from gello.dynamixel.driver import DynamixelDriver

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

GRIPPER_JOINT_ID = 7


@dataclass
class Args:
    port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WA0-if00-port0"
    """U2D2 serial port for the GELLO leader."""

    gripper_joint_id: int = GRIPPER_JOINT_ID
    """Dynamixel ID for the trigger/gripper servo (default: 7)."""


def read_gripper_deg(driver: DynamixelDriver) -> float:
    for _ in range(10):
        joints = driver.get_joints()
    return float(np.rad2deg(joints[-1]))


def main(args: Args) -> None:
    joint_ids = list(range(1, args.gripper_joint_id + 1))
    print(f"Connecting to {args.port} (joints {joint_ids})...")
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    input("Release the trigger, then press Enter to capture OPEN position...")
    open_deg = read_gripper_deg(driver)
    print(f"  open  = {open_deg:.5f}°")

    input("Fully squeeze the trigger, then press Enter to capture CLOSED position...")
    close_deg = read_gripper_deg(driver)
    print(f"  close = {close_deg:.5f}°")

    gripper_config = [args.gripper_joint_id, open_deg, close_deg]
    print()
    print("gripper_config:")
    print(f"  {gripper_config}")
    print()
    print("Paste into your YAML under agent.dynamixel_config.gripper_config")
    driver.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
