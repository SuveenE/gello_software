"""Calibrate GELLO trigger (joint 7) open/close for gripper_config.

The Dynamixel driver maps raw trigger degrees linearly to [0, 1]:
  0 = open, 1 = fully closed

If your squeeze range shrank after moving the servo, the old close angle is often
too extreme and normalized gripper never reaches 1.0 at full squeeze. This script
measures your actual open/close angles and can pad close slightly toward open so
full squeeze reliably hits 1.0 in sim/hardware.

Usage (right arm example):
  python scripts/calibrate_gripper.py \\
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WI5-if00-port0

  python scripts/calibrate_gripper.py \\
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WI5-if00-port0 \\
    --write-config configs/yam_auto_generated_sim_right.yaml

Live monitor only (no capture):
  python scripts/calibrate_gripper.py --port ... --monitor-only
"""

from __future__ import annotations

import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

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

    close_padding_deg: float = 3.0
    """Move configured close toward open by this many degrees so full squeeze reaches g=1.0."""

    monitor_only: bool = False
    """Stream live raw degrees and preview normalized g; Ctrl+C to exit."""

    write_config: Optional[str] = None
    """Optional YAML config path to update agent.dynamixel_config.gripper_config."""


def read_gripper_deg(driver: DynamixelDriver, samples: int = 10) -> float:
    for _ in range(samples):
        joints = driver.get_joints()
    return float(np.rad2deg(joints[-1]))


def normalized_g(raw_deg: float, open_deg: float, close_deg: float) -> float:
    denom = close_deg - open_deg
    if abs(denom) < 1e-6:
        return 0.0
    return float(np.clip((raw_deg - open_deg) / denom, 0.0, 1.0))


def apply_close_padding(open_deg: float, close_measured: float, padding_deg: float) -> float:
    """Shift close toward open so the physical squeeze limit maps to g >= 1.0."""
    if padding_deg <= 0:
        return close_measured
    if close_measured < open_deg:
        return close_measured + padding_deg
    return close_measured - padding_deg


def format_gripper_config(joint_id: int, open_deg: float, close_deg: float) -> list:
    return [joint_id, float(open_deg), float(close_deg)]


def update_yaml_gripper_config(config_path: Path, gripper_config: list) -> None:
    text = config_path.read_text()
    new_line = (
        f"    gripper_config: [{int(gripper_config[0])}, "
        f"{gripper_config[1]}, {gripper_config[2]}]"
    )
    updated, count = re.subn(
        r"^\s*gripper_config:\s*\[[^\]]+\]\s*$",
        new_line,
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError(
            f"Could not find gripper_config line in {config_path}. Update the YAML manually."
        )
    config_path.write_text(updated)


def run_monitor(driver: DynamixelDriver, open_deg: Optional[float], close_deg: Optional[float]) -> None:
    print("Live monitor — move the trigger (Ctrl+C to stop).")
    if open_deg is not None and close_deg is not None:
        print(f"Preview using open={open_deg:.2f}° close={close_deg:.2f}°")
    print(f"{'raw_deg':>10}  {'g_preview':>10}  {'range':>10}")
    min_deg = float("inf")
    max_deg = float("-inf")
    try:
        while True:
            raw = read_gripper_deg(driver, samples=3)
            min_deg = min(min_deg, raw)
            max_deg = max(max_deg, raw)
            span = max_deg - min_deg
            if open_deg is not None and close_deg is not None:
                g = normalized_g(raw, open_deg, close_deg)
                print(f"{raw:10.2f}  {g:10.3f}  {span:10.2f}°", end="\r", flush=True)
            else:
                print(f"{raw:10.2f}  {'—':>10}  {span:10.2f}°", end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
        print(f"Observed range: {min_deg:.2f}° .. {max_deg:.2f}°  (span {max_deg - min_deg:.2f}°)")


def main(args: Args) -> None:
    joint_ids = list(range(1, args.gripper_joint_id + 1))
    print(f"Connecting to {args.port} (joints {joint_ids})...")
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    if args.monitor_only:
        run_monitor(driver, open_deg=None, close_deg=None)
        driver.close()
        return

    print()
    print("Step 1/3: OPEN — release the trigger completely.")
    input("Press Enter to capture OPEN...")
    open_deg = read_gripper_deg(driver)
    print(f"  measured open  = {open_deg:.5f}°")
    print()
    print("Step 2/3: CLOSE — squeeze the trigger as hard as you normally would.")
    input("Press Enter to capture CLOSED...")
    close_measured = read_gripper_deg(driver)
    print(f"  measured close = {close_measured:.5f}°")

    span = abs(close_measured - open_deg)
    print(f"  squeeze span   = {span:.2f}°")
    if span < 10.0:
        print("  WARNING: squeeze span is very small — check servo mount and trigger travel.")

    close_deg = apply_close_padding(open_deg, close_measured, args.close_padding_deg)
    g_at_measured = normalized_g(close_measured, open_deg, close_deg)
    g_at_open = normalized_g(open_deg, open_deg, close_deg)

    print()
    print("Step 3/3: verify (squeeze again; g should reach ~1.0 at full squeeze).")
    if args.close_padding_deg > 0:
        print(
            f"  close padded {args.close_padding_deg:.1f}° toward open: "
            f"{close_measured:.2f}° -> {close_deg:.2f}°"
        )
    print(f"  g at open      = {g_at_open:.3f}")
    print(f"  g at measured  = {g_at_measured:.3f}")
    run_monitor(driver, open_deg=open_deg, close_deg=close_deg)

    gripper_config = format_gripper_config(args.gripper_joint_id, open_deg, close_deg)
    print()
    print("gripper_config (paste into agent.dynamixel_config.gripper_config):")
    print(f"  {gripper_config}")

    if args.write_config:
        config_path = Path(args.write_config)
        if not config_path.is_file():
            raise FileNotFoundError(config_path)
        update_yaml_gripper_config(config_path, gripper_config)
        print(f"Updated {config_path}")

    driver.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
