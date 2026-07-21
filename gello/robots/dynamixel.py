import threading
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.robots.robot import Robot

# Poll interval (s) for the assisted-motor thermal safety check. Low frequency so
# it barely competes with the high-rate joint read thread on the same bus.
_ASSIST_TEMP_POLL_S = 2.0


class DynamixelRobot(Robot):
    """A class representing a UR robot."""

    def __init__(
        self,
        joint_ids: Sequence[int],
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        real: bool = False,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        from gello.dynamixel.driver import (
            DynamixelDriver,
            DynamixelDriverProtocol,
            FakeDynamixelDriver,
        )

        print(f"attempting to connect to port: {port}")
        self.gripper_open_close: Optional[Tuple[float, float]]
        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            # joint_ids.append(gripper_config[0])
            # joint_offsets.append(0.0)
            # joint_signs.append(1)
            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        self._joint_ids = joint_ids
        self._driver: DynamixelDriverProtocol

        if joint_offsets is None:
            self._joint_offsets = np.zeros(len(joint_ids))
        else:
            self._joint_offsets = np.array(joint_offsets)

        if joint_signs is None:
            self._joint_signs = np.ones(len(joint_ids))
        else:
            self._joint_signs = np.array(joint_signs)

        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        if real:
            self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
            self._driver.set_torque_mode(False)
        else:
            self._driver = FakeDynamixelDriver(joint_ids)
        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        # Snapshot the calibrated start pose before the wrap-around logic below
        # consumes it; the return-assist uses it as the per-joint target.
        self._start_joints = (
            None if start_joints is None else np.asarray(start_joints, dtype=float)
        )
        # Return-assist state (opt-in via enable_joint_return_assist).
        self._assist_ids: list = []
        self._assist_max_temp_c = 0.0
        self._assist_enabled = False
        self._assist_thread: Optional[threading.Thread] = None
        self._assist_stop: Optional[threading.Event] = None

        if start_joints is not None:
            # loop through all joints and add +- 2pi to the joint offsets to get the closest to start joints
            new_joint_offsets = []
            current_joints = self.get_joint_state()
            assert current_joints.shape == start_joints.shape
            if gripper_config is not None:
                current_joints = current_joints[:-1]
                start_joints = start_joints[:-1]
            for idx, (c_joint, s_joint, joint_offset) in enumerate(
                zip(current_joints, start_joints, self._joint_offsets)
            ):
                new_joint_offsets.append(
                    np.pi
                    * 2
                    * np.round((-s_joint + c_joint) / (2 * np.pi))
                    * self._joint_signs[idx]
                    + joint_offset
                )
            if gripper_config is not None:
                new_joint_offsets.append(self._joint_offsets[-1])
            self._joint_offsets = np.array(new_joint_offsets)

    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs()

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        self._driver.set_joints((joint_state + self._joint_offsets).tolist())

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}

    def enable_joint_return_assist(
        self,
        assist_ids: Sequence[int],
        current_ma: float,
        max_temperature_c: float = 50.0,
        watchdog_timeout_s: float = 1.0,
    ) -> None:
        """Give a gentle, current-limited pull back to the start pose on some joints.

        The listed motor IDs are switched into current-based position control mode
        with their Goal Position set to the calibrated start pose and their torque
        capped at ``current_ma``. Because the cap is well below stall current the
        joint stays easily backdrivable, but it will drift back toward the start
        pose when released (useful when a stretched joint is hard to bring back
        one-handed). All other motors are left untouched (passive/torque-off).

        Safety: a bus watchdog relaxes the motors if this process or the USB link
        dies while torque is on, and a background thread disables the assist if any
        assisted motor gets too hot.
        """
        from gello.dynamixel.driver import CURRENT_BASED_POSITION_CONTROL_MODE

        if self._assist_enabled:
            return
        assist_ids = [int(m) for m in assist_ids]
        if not assist_ids:
            return
        if current_ma <= 0:
            raise ValueError(f"current_ma must be positive, got {current_ma}")
        if max_temperature_c <= 0:
            raise ValueError(
                f"max_temperature_c must be positive, got {max_temperature_c}"
            )

        id_to_index = {int(jid): idx for idx, jid in enumerate(self._joint_ids)}
        targets_rad: Dict[int, float] = {}
        for motor_id in assist_ids:
            if motor_id not in id_to_index:
                raise ValueError(
                    f"assist motor id {motor_id} is not among joint_ids {tuple(self._joint_ids)}"
                )
            idx = id_to_index[motor_id]
            if self._start_joints is not None and idx < len(self._start_joints):
                calibrated_target = float(self._start_joints[idx])
            else:
                calibrated_target = 0.0
            # Invert get_joint_state's mapping pos = (raw - offset) * sign to get
            # the raw motor angle for the desired calibrated target (sign = +-1).
            raw_target = calibrated_target * self._joint_signs[idx] + self._joint_offsets[idx]
            targets_rad[motor_id] = float(raw_target)

        # Torque must be off to change the operating mode; the driver comes up with
        # torque disabled, but assert it explicitly for these IDs to be safe.
        self._driver.set_torque_mode_for_ids(assist_ids, False)
        self._driver.set_operating_mode_for_ids(
            assist_ids, CURRENT_BASED_POSITION_CONTROL_MODE
        )
        self._driver.verify_operating_mode_for_ids(
            assist_ids, CURRENT_BASED_POSITION_CONTROL_MODE
        )
        # Cap the holding torque (Goal Current) before enabling torque.
        self._driver.set_goal_currents_for_ids({m: current_ma for m in assist_ids})
        self._driver.set_torque_mode_for_ids(assist_ids, True)
        self._driver.set_goal_positions_for_ids(targets_rad)
        self._driver.set_bus_watchdog_for_ids(assist_ids, watchdog_timeout_s)

        self._assist_ids = assist_ids
        self._assist_max_temp_c = float(max_temperature_c)
        self._assist_stop = threading.Event()
        self._assist_enabled = True
        self._assist_thread = threading.Thread(
            target=self._monitor_assist_temperature,
            name="gello-return-assist-thermal",
            daemon=True,
        )
        self._assist_thread.start()
        print(
            f"[return-assist] enabled on motor ids {assist_ids} at {current_ma:.0f} mA "
            f"(cutoff {max_temperature_c:.0f}C, watchdog {watchdog_timeout_s:.2f}s)"
        )

    def _monitor_assist_temperature(self) -> None:
        assert self._assist_stop is not None
        while not self._assist_stop.wait(_ASSIST_TEMP_POLL_S):
            try:
                temps = self._driver.read_temperatures(self._assist_ids)
            except Exception as e:  # noqa: BLE001 - keep monitoring on transient errors
                print(f"[return-assist] temperature read failed: {e}")
                continue
            hot = {i: t for i, t in temps.items() if t >= self._assist_max_temp_c}
            if hot:
                print(
                    f"[return-assist] over-temperature {hot} >= "
                    f"{self._assist_max_temp_c:.0f}C; disabling assist torque"
                )
                try:
                    self._driver.set_torque_mode_for_ids(self._assist_ids, False)
                except Exception as e:  # noqa: BLE001
                    print(f"[return-assist] failed to disable torque on overtemp: {e}")
                self._assist_enabled = False
                return

    def disable_joint_return_assist(self) -> None:
        """Stop the thermal monitor and relax any assisted motors (idempotent)."""
        if self._assist_stop is not None:
            self._assist_stop.set()
        if self._assist_thread is not None:
            self._assist_thread.join(timeout=_ASSIST_TEMP_POLL_S + 1.0)
            self._assist_thread = None
        if self._assist_ids:
            try:
                self._driver.set_bus_watchdog_for_ids(self._assist_ids, 0.0)
            except Exception as e:  # noqa: BLE001
                print(f"[return-assist] failed to clear bus watchdog: {e}")
            try:
                self._driver.set_torque_mode_for_ids(self._assist_ids, False)
            except Exception as e:  # noqa: BLE001
                print(f"[return-assist] failed to disable torque on shutdown: {e}")
        self._assist_enabled = False
        self._assist_stop = None
        self._assist_ids = []
