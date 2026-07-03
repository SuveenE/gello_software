"""Calibrate GELLO trigger (joint 7) open/close for gripper_config.

Normalized gripper convention in gello/robots/dynamixel.py:
  0.0 = open (trigger released)
  1.0 = fully closed (trigger squeezed)

If you changed the Dynamixel Max/Min Position Limit on joint 7, the trigger travel
shrinks and old gripper_config values stop working. At rest you may also see a
partially closed sim gripper if the configured open angle no longer matches the
released trigger position.

This script:
  1. Reads firmware position limits on joint 7
  2. Auto-sweeps the trigger to capture min/max travel
  3. Re-measures the released rest position as open (avoids stale pre-sweep readings)
  4. Writes gripper_config using measured endpoints (+ optional close padding)
  5. Can update servo limits to match the measured sweep

Usage (right arm):
  python scripts/calibrate_gripper.py \\
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WI5-if00-port0 \\
    --write-config configs/yam_auto_generated_sim_right.yaml

If limits are too tight after a servo change:
  python scripts/calibrate_gripper.py --port ... --set-limits-from-sweep

Restore full extended range on joint 7 only:
  python scripts/calibrate_gripper.py --port ... --restore-full-range
"""

from __future__ import annotations

import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from gello.dynamixel.driver import (
    EXTENDED_POSITION_MAX_TICKS,
    EXTENDED_POSITION_MIN_TICKS,
    DynamixelDriver,
)

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

GRIPPER_JOINT_ID = 7


@dataclass
class Args:
    port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WA0-if00-port0"
    """U2D2 serial port for the GELLO leader."""

    gripper_joint_id: int = GRIPPER_JOINT_ID
    """Dynamixel ID for the trigger/gripper servo (default: 7)."""

    sweep_seconds: float = 8.0
    """Seconds to sweep the trigger open -> closed -> open while tracking min/max."""

    close_padding_deg: float = 2.0
    """Move configured close toward open so full squeeze reaches g=1.0 in software."""

    limit_margin_deg: float = 5.0
    """Margin added when writing firmware limits from the measured sweep."""

    monitor_only: bool = False
    """Stream live raw degrees; Ctrl+C to exit."""

    show_limits: bool = False
    """Only print joint-7 firmware limits and exit."""

    restore_full_range: bool = False
    """Restore joint-7 Min/Max Position Limit to full extended range."""

    set_limits_from_sweep: bool = False
    """After sweep, write firmware limits to measured open/close (+ margin)."""

    write_config: Optional[str] = None
    """Optional YAML config path to update agent.dynamixel_config.gripper_config."""


def read_gripper_deg(driver: DynamixelDriver, samples: int = 5) -> float:
    for _ in range(samples):
        joints = driver.get_joints()
    return float(np.rad2deg(joints[-1]))


def normalized_g(raw_deg: float, open_deg: float, close_deg: float) -> float:
    denom = close_deg - open_deg
    if abs(denom) < 1e-6:
        return 0.0
    return float(np.clip((raw_deg - open_deg) / denom, 0.0, 1.0))


def apply_close_padding(open_deg: float, close_measured: float, padding_deg: float) -> float:
    if padding_deg <= 0:
        return close_measured
    if close_measured < open_deg:
        return close_measured + padding_deg
    return close_measured - padding_deg


def pick_close_deg(open_deg: float, min_deg: float, max_deg: float) -> float:
    """Return the sweep endpoint farthest from the released (open) position."""
    if abs(min_deg - open_deg) >= abs(max_deg - open_deg):
        return min_deg
    return max_deg


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


def update_yaml_start_joints_gripper(config_path: Path, gripper_value: float = 0.0) -> None:
    text = config_path.read_text()
    updated, count = re.subn(
        r"(start_joints:\s*\[[^\]]*,\s*)([0-9.]+)(\s*\])",
        rf"\g<1>{gripper_value}\3",
        text,
        count=1,
    )
    if count == 1:
        config_path.write_text(updated)


def print_limits(driver: DynamixelDriver, joint_id: int) -> Tuple[int, int]:
    min_ticks, max_ticks = driver.read_position_limits(joint_id)
    print(f"Joint {joint_id} firmware limits:")
    print(f"  min = {min_ticks} ticks ({driver.ticks_to_deg(min_ticks):.2f}°)")
    print(f"  max = {max_ticks} ticks ({driver.ticks_to_deg(max_ticks):.2f}°)")
    return min_ticks, max_ticks


def sweep_trigger_range(
    driver: DynamixelDriver, duration_s: float
) -> Tuple[float, float, float]:
    print()
    print(
        f"Sweep the trigger for {duration_s:.0f}s: fully open -> fully closed -> open again."
    )
    print("Tracking min/max raw degrees...")
    deadline = time.time() + duration_s
    min_deg = float("inf")
    max_deg = float("-inf")
    last_print = 0.0
    while time.time() < deadline:
        raw = read_gripper_deg(driver, samples=2)
        min_deg = min(min_deg, raw)
        max_deg = max(max_deg, raw)
        now = time.time()
        if now - last_print > 0.1:
            print(
                f"  raw={raw:8.2f}°   min={min_deg:8.2f}°   max={max_deg:8.2f}°",
                end="\r",
                flush=True,
            )
            last_print = now
        time.sleep(0.02)
    print()
    span = max_deg - min_deg
    print(f"  observed min = {min_deg:.5f}°")
    print(f"  observed max = {max_deg:.5f}°")
    print(f"  span         = {span:.2f}°")
    return min_deg, max_deg, span


def run_monitor(
    driver: DynamixelDriver,
    open_deg: Optional[float],
    close_deg: Optional[float],
) -> Tuple[float, float]:
    print("Live monitor — move the trigger (Ctrl+C to stop).")
    if open_deg is not None and close_deg is not None:
        print(f"Using open={open_deg:.2f}° close={close_deg:.2f}°")
    print(f"{'raw_deg':>10}  {'g':>8}  {'span':>10}")
    min_deg = float("inf")
    max_deg = float("-inf")
    try:
        while True:
            raw = read_gripper_deg(driver, samples=2)
            min_deg = min(min_deg, raw)
            max_deg = max(max_deg, raw)
            span = max_deg - min_deg
            if open_deg is not None and close_deg is not None:
                g = normalized_g(raw, open_deg, close_deg)
                print(f"{raw:10.2f}  {g:8.3f}  {span:10.2f}°", end="\r", flush=True)
            else:
                print(f"{raw:10.2f}  {'—':>8}  {span:10.2f}°", end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
        print(f"Observed range: {min_deg:.2f}° .. {max_deg:.2f}°  (span {span:.2f}°)")
    return min_deg, max_deg


def deg_to_ticks(driver: DynamixelDriver, deg: float) -> int:
    return driver._rad_to_ticks(np.deg2rad(deg))


def write_limits_for_sweep(
    driver: DynamixelDriver,
    joint_id: int,
    open_deg: float,
    close_deg: float,
    margin_deg: float,
) -> None:
    lo_deg = min(open_deg, close_deg) - margin_deg
    hi_deg = max(open_deg, close_deg) + margin_deg
    min_ticks = deg_to_ticks(driver, lo_deg)
    max_ticks = deg_to_ticks(driver, hi_deg)
    driver.write_position_limits(joint_id, min_ticks, max_ticks)
    print(
        f"Wrote joint {joint_id} limits: "
        f"{lo_deg:.2f}° .. {hi_deg:.2f}° ({min_ticks} .. {max_ticks} ticks)"
    )


def main(args: Args) -> None:
    joint_ids = list(range(1, args.gripper_joint_id + 1))
    print(f"Connecting to {args.port} (joints {joint_ids})...")
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    if args.show_limits or args.restore_full_range:
        print_limits(driver, args.gripper_joint_id)
        if args.restore_full_range:
            driver.write_position_limits(
                args.gripper_joint_id,
                EXTENDED_POSITION_MIN_TICKS,
                EXTENDED_POSITION_MAX_TICKS,
            )
            print("Restored full extended position range on joint 7.")
            print_limits(driver, args.gripper_joint_id)
        driver.close()
        return

    if args.monitor_only:
        run_monitor(driver, open_deg=None, close_deg=None)
        driver.close()
        return

    print_limits(driver, args.gripper_joint_id)

    print()
    print("Step 1: Sweep the trigger through its full range.")
    min_deg, max_deg, span = sweep_trigger_range(driver, args.sweep_seconds)
    if span < 5.0:
        print(
            "WARNING: squeeze span is very small. The Dynamixel position limits may be "
            "too tight — try --restore-full-range or --set-limits-from-sweep."
        )

    print()
    print("Step 2: Release the trigger to its natural rest position (fully open).")
    input("Press Enter when ready...")
    open_deg = read_gripper_deg(driver)
    close_deg = pick_close_deg(open_deg, min_deg, max_deg)
    print(f"  open  (released rest) = {open_deg:.5f}°")
    print(f"  close (from sweep)    = {close_deg:.5f}°")
    if abs(close_deg - open_deg) < 1.0:
        raise RuntimeError("Open and close are too close. Check joint 7 and servo limits.")

    close_config = apply_close_padding(open_deg, close_deg, args.close_padding_deg)
    g_at_open = normalized_g(open_deg, open_deg, close_config)
    g_at_close = normalized_g(close_deg, open_deg, close_config)
    g_at_rest = normalized_g(read_gripper_deg(driver), open_deg, close_config)

    print()
    print("Calibration summary:")
    print(f"  open  (released)          = {open_deg:.5f}°  -> g={g_at_open:.3f}")
    print(f"  close (fully squeezed)    = {close_deg:.5f}°  -> g={g_at_close:.3f}")
    print(f"  close in config           = {close_config:.5f}°  (padding {args.close_padding_deg:.1f}°)")
    print(f"  current trigger at rest   = g={g_at_rest:.3f}  (should be ~0.0 when released)")
    if g_at_rest > 0.05:
        print(
            "WARNING: g is not near 0 at rest. Re-check that the trigger is fully "
            "released, or run with --restore-full-range if the Dynamixel limits are tight."
        )

    if args.set_limits_from_sweep:
        write_limits_for_sweep(
            driver,
            args.gripper_joint_id,
            open_deg,
            close_deg,
            args.limit_margin_deg,
        )
        print_limits(driver, args.gripper_joint_id)

    print()
    print("Verify: release trigger (g~0), squeeze fully (g~1). Ctrl+C when done.")
    run_monitor(driver, open_deg=open_deg, close_deg=close_config)

    gripper_config = format_gripper_config(
        args.gripper_joint_id, open_deg, close_config
    )
    print()
    print("gripper_config:")
    print(f"  {gripper_config}")
    print()
    print("Also set start_joints gripper to 0.0 (open) in your YAML:")
    print("  start_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")

    if args.write_config:
        config_path = Path(args.write_config)
        if not config_path.is_file():
            raise FileNotFoundError(config_path)
        update_yaml_gripper_config(config_path, gripper_config)
        update_yaml_start_joints_gripper(config_path, gripper_value=0.0)
        print(f"Updated {config_path} (gripper_config + start_joints[6]=0.0)")

    driver.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
