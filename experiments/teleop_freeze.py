"""Bimanual GELLO->YAM teleop with a trigger-toggled "freeze" (self-hold) mode.

Behaviour:
  * Teleops one or both arms normally (GELLO leader -> YAM follower), exactly
    like ``experiments/launch_yaml.py``.
  * The 7th GELLO joint (the gripper/trigger) is NOT used to drive the gripper
    here. Instead it acts as a toggle:
      - Squeeze the trigger past the half-way point -> the GELLO arm joints
        (1-6) "freeze": torque is enabled and they hold their current pose using
        current-based position control, capped at ~80% of each servo's max
        torque, so the leader holds itself at any pose.
      - Squeeze it past half-way again -> the freeze releases and the arm is
        passive (back-drivable) again.
  * The trigger motor itself is deliberately left un-braked so you can always
    press it again to release the freeze.

Usage:
  conda activate lerobot
  # single arm
  python experiments/teleop_freeze.py --left-config-path configs/yam_left_hw.yaml
  # both arms
  python experiments/teleop_freeze.py \
      --left-config-path configs/yam_left_hw.yaml \
      --right-config-path configs/yam_right_hw.yaml

Exit with Ctrl+C (clean) so torque is disabled on shutdown.
"""

import argparse
import signal
import sys
import time
from typing import List, Optional

import numpy as np
from omegaconf import OmegaConf

from gello.dynamixel.driver import (
    ADDR_GOAL_POSITION,
    ADDR_OPERATING_MODE,
    ADDR_TORQUE_ENABLE,
    COMM_SUCCESS,
    TORQUE_DISABLE,
    TORQUE_ENABLE,
)
from gello.env import Rate, RobotEnv
from gello.utils.launch_utils import instantiate_from_dict, move_to_start_position

# Dynamixel operating mode used to hold the arm: Extended Position Control is
# multi-turn and holds at full torque (capped only by the servo's Current
# Limit). On release we restore each joint's *original* mode. Both the GELLO
# default (Current Control, 0) and Extended Position (4) are multi-turn, so
# switching between them does NOT re-wrap the reported position -> the follower
# does not jerk (e.g. wrist yaw snapping ~90 deg) on release.
EXTENDED_POSITION_CONTROL_MODE = 4

# Tuning -----------------------------------------------------------------
PRESS_THRESHOLD = 0.5  # trigger (0=open .. 1=fully squeezed) past half = press
RELEASE_THRESHOLD = 0.3  # must drop below this before another press counts


def _conv_rad_to_ticks(rad: float) -> int:
    """Match DynamixelDriver.set_joints position encoding."""
    return int(rad * 2048 / np.pi)


class LeaderBrake:
    """Freezes/releases the GELLO *arm* joints (gripper joint left untouched).

    Works directly on the existing DynamixelDriver held by the GelloAgent, so
    it shares the same serial port and the driver's internal lock.

    The freeze switches each arm joint to Extended Position Control (multi-turn,
    full torque) and holds it at its captured pose. On release each joint is
    restored to its original operating mode. Both modes are multi-turn, so the
    reported position never re-wraps and the follower does not jerk.
    """

    def __init__(self, label: str, gello_agent):
        self.label = label
        dxl_robot = gello_agent._robot
        self.driver = dxl_robot._driver
        all_ids = list(dxl_robot._joint_ids)
        # last id is the gripper/trigger -> exclude it from the brake
        self.arm_ids: List[int] = all_ids[:-1]
        self.gripper_id: int = all_ids[-1]
        self.frozen = False
        self._armed = False  # becomes True once trigger is released
        self._orig_modes = {}  # id -> operating mode to restore on release

        self._real = hasattr(self.driver, "_packetHandler")
        if not self._real:
            print(f"[{label}] WARNING: fake driver, freeze disabled.")
            return
        self._cache_modes()

    # -- low level helpers (must hold driver lock) -----------------------
    def _w1(self, dxl_id, addr, val):
        r, e = self.driver._packetHandler.write1ByteTxRx(
            self.driver._portHandler, dxl_id, addr, val
        )
        if r != COMM_SUCCESS or e != 0:
            print(f"[{self.label}] id {dxl_id} write1@{addr} failed (r={r}, e={e})")

    def _w4(self, dxl_id, addr, val):
        r, e = self.driver._packetHandler.write4ByteTxRx(
            self.driver._portHandler, dxl_id, addr, int(val) & 0xFFFFFFFF
        )
        if r != COMM_SUCCESS or e != 0:
            print(f"[{self.label}] id {dxl_id} write4@{addr} failed (r={r}, e={e})")

    def _cache_modes(self):
        """Record each arm joint's current operating mode (restored on release)."""
        with self.driver._lock:
            for dxl_id in self.arm_ids:
                mode, r, e = self.driver._packetHandler.read1ByteTxRx(
                    self.driver._portHandler, dxl_id, ADDR_OPERATING_MODE
                )
                self._orig_modes[dxl_id] = (
                    mode if (r == COMM_SUCCESS and e == 0) else EXTENDED_POSITION_CONTROL_MODE
                )
        print(
            f"[{self.label}] original operating modes (restored on release): "
            + ", ".join(f"id{i}={self._orig_modes[i]}" for i in self.arm_ids)
        )

    # -- public API ------------------------------------------------------
    def freeze(self):
        if not self._real or self.frozen:
            return
        # capture current pose BEFORE grabbing the lock (no port access needed)
        joints_rad = self.driver.get_joints()[: len(self.arm_ids)]
        ticks = [_conv_rad_to_ticks(float(a)) for a in joints_rad]
        with self.driver._lock:
            # Torque must be off to change operating mode.
            for dxl_id in self.arm_ids:
                self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            # Switch to multi-turn position hold and target the captured pose
            # BEFORE enabling torque, so it locks exactly where it is.
            for dxl_id in self.arm_ids:
                self._w1(dxl_id, ADDR_OPERATING_MODE, EXTENDED_POSITION_CONTROL_MODE)
            for dxl_id, tick in zip(self.arm_ids, ticks):
                self._w4(dxl_id, ADDR_GOAL_POSITION, tick)
            for dxl_id in self.arm_ids:
                self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            self.driver._torque_enabled = True
        self.frozen = True
        print(f"\n[{self.label}] FROZEN - arm holding pose at full torque.")

    def release(self):
        if not self._real or not self.frozen:
            return
        with self.driver._lock:
            # Torque off, then restore each joint's original operating mode.
            for dxl_id in self.arm_ids:
                self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            for dxl_id in self.arm_ids:
                self._w1(dxl_id, ADDR_OPERATING_MODE, self._orig_modes[dxl_id])
            self.driver._torque_enabled = False
        self.frozen = False
        print(f"\n[{self.label}] RELEASED - arm is back-drivable again.")

    def update_from_trigger(self, trigger_value: float):
        """Toggle freeze on a rising edge past the press threshold (hysteresis)."""
        if not self._real:
            return
        if self._armed and trigger_value >= PRESS_THRESHOLD:
            self._armed = False
            if self.frozen:
                self.release()
            else:
                self.freeze()
        elif (not self._armed) and trigger_value <= RELEASE_THRESHOLD:
            self._armed = True

    def shutdown(self):
        if not self._real:
            return
        try:
            if self.frozen:
                self.release()
            self.driver.set_torque_mode(False)
        except Exception as e:
            print(f"[{self.label}] shutdown warning: {e}")


class Arm:
    """One GELLO leader + YAM follower pair."""

    def __init__(self, label: str, config_path: str):
        self.label = label
        cfg = OmegaConf.to_container(OmegaConf.load(config_path), resolve=True)
        self.cfg = cfg
        self.agent = instantiate_from_dict(cfg["agent"])

        robot_cfg = cfg["robot"]
        if isinstance(robot_cfg.get("config"), str):
            robot_cfg["config"] = OmegaConf.to_container(
                OmegaConf.load(robot_cfg["config"]), resolve=True
            )
        self.robot = instantiate_from_dict(robot_cfg)
        self.env = RobotEnv(self.robot, control_rate_hz=cfg.get("hz", 30))
        self.brake = LeaderBrake(label, self.agent)
        self.gripper_hold: Optional[float] = None

    def align(self):
        print(f"[{self.label}] aligning YAM to GELLO...")
        move_to_start_position(
            self.env, bimanual=False, left_cfg=self.cfg, agent=self.agent
        )
        # hold the gripper wherever it currently is (trigger is repurposed)
        action = np.array(self.agent.act(self.env.get_obs()), dtype=float)
        self.gripper_hold = float(action[-1])

    def step(self):
        action = np.array(self.agent.act(self.env.get_obs()), dtype=float)
        trigger = float(action[-1])
        self.brake.update_from_trigger(trigger)
        # follower command: arm joints from leader, gripper held constant
        command = action.copy()
        command[-1] = self.gripper_hold if self.gripper_hold is not None else trigger
        self.robot.command_joint_state(command)

    def shutdown(self):
        self.brake.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-config-path", required=True)
    parser.add_argument("--right-config-path", default=None)
    args = parser.parse_args()

    arms: List[Arm] = [Arm("LEFT", args.left_config_path)]
    if args.right_config_path:
        arms.append(Arm("RIGHT", args.right_config_path))

    hz = arms[0].cfg.get("hz", 30)
    rate = Rate(hz)

    stop = {"flag": False}

    def _handle_sigint(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_sigint)
    signal.signal(signal.SIGTERM, _handle_sigint)

    try:
        for arm in arms:
            arm.align()

        print("\n=== Teleop running ===")
        print("Squeeze the trigger past half-way to FREEZE the arm in place.")
        print("Squeeze again to RELEASE. Ctrl+C to quit.\n")

        start = time.time()
        while not stop["flag"]:
            for arm in arms:
                arm.step()
            states = " | ".join(
                f"{a.label}:{'FROZEN' if a.brake.frozen else 'live'}" for a in arms
            )
            print(f"\rt={time.time()-start:7.1f}s  {states}        ", end="", flush=True)
            rate.sleep()
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nShutting down, disabling leader torque...")
        for arm in arms:
            arm.shutdown()
        print("Done.")
        sys.exit(0)


if __name__ == "__main__":
    main()
