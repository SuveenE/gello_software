import time

import numpy as np
import pytest

import gello.robots.dynamixel as dynamixel_mod
from gello.dynamixel.driver import (
    CURRENT_BASED_POSITION_CONTROL_MODE,
    POSITION_CONTROL_MODE,
)
from gello.robots.dynamixel import DynamixelRobot


def _make_robot(start_joints=None):
    # real=False -> FakeDynamixelDriver. Gripper (id 7) is appended internally.
    return DynamixelRobot(
        joint_ids=[1, 2, 3, 4, 5, 6],
        joint_offsets=[1.0, 2.0, 0.0, 0.0, 0.0, 0.0],
        joint_signs=[1, -1, -1, -1, 1, 1],
        real=False,
        gripper_config=(7, 10.0, 20.0),
        start_joints=start_joints,
    )


def test_assist_only_touches_requested_ids():
    robot = _make_robot()
    robot.enable_joint_return_assist([1, 2], current_ma=100.0)
    driver = robot._driver

    # Assisted IDs go to current-based position mode with torque on.
    assert driver._operating_modes[1] == CURRENT_BASED_POSITION_CONTROL_MODE
    assert driver._operating_modes[2] == CURRENT_BASED_POSITION_CONTROL_MODE
    assert driver._per_id_torque[1] is True
    assert driver._per_id_torque[2] is True
    assert driver._goal_currents == {1: 100.0, 2: 100.0}

    # Every other motor (arm joints 3-6 and gripper id 7) stays passive.
    for other_id in (3, 4, 5, 6, 7):
        assert driver._operating_modes[other_id] == POSITION_CONTROL_MODE
        assert driver._per_id_torque[other_id] is False

    robot.disable_joint_return_assist()


def test_assist_target_conversion_respects_sign_and_offset():
    # start pose small enough that the offset wrap-around leaves offsets intact.
    start = np.array([0.5, -0.3, 0.0, 0.0, 0.0, 0.0, 1.0])
    robot = _make_robot(start_joints=start)
    robot.enable_joint_return_assist([1, 2], current_ma=100.0)

    # raw_target = calibrated_target * sign + offset
    # id 1: 0.5 * (+1) + 1.0 = 1.5 ; id 2: -0.3 * (-1) + 2.0 = 2.3
    assert robot._driver._goal_positions[1] == pytest.approx(1.5)
    assert robot._driver._goal_positions[2] == pytest.approx(2.3)

    robot.disable_joint_return_assist()


def test_assist_rejects_unknown_and_invalid_args():
    robot = _make_robot()
    with pytest.raises(ValueError):
        robot.enable_joint_return_assist([99], current_ma=100.0)
    with pytest.raises(ValueError):
        robot.enable_joint_return_assist([1], current_ma=0.0)
    with pytest.raises(ValueError):
        robot.enable_joint_return_assist([1], current_ma=100.0, max_temperature_c=0.0)


def test_assist_sets_bus_watchdog_and_clears_on_disable():
    robot = _make_robot()
    robot.enable_joint_return_assist([1, 2], current_ma=100.0, watchdog_timeout_s=1.0)
    assert robot._driver._bus_watchdog == {1: 1.0, 2: 1.0}

    robot.disable_joint_return_assist()
    assert robot._assist_enabled is False
    assert robot._driver._per_id_torque[1] is False
    assert robot._driver._per_id_torque[2] is False
    # Watchdog cleared (0 disables it) so the motors are not left armed.
    assert robot._driver._bus_watchdog == {1: 0.0, 2: 0.0}


def test_assist_thermal_cutoff_disables_torque(monkeypatch):
    monkeypatch.setattr(dynamixel_mod, "_ASSIST_TEMP_POLL_S", 0.02)
    robot = _make_robot()
    robot.enable_joint_return_assist([1, 2], current_ma=100.0, max_temperature_c=50.0)

    # Simulate motor 1 overheating; the monitor thread should latch the assist off.
    robot._driver._temperatures[1] = 80.0

    deadline = time.time() + 3.0
    while time.time() < deadline and robot._assist_enabled:
        time.sleep(0.02)

    assert robot._assist_enabled is False
    assert robot._driver._per_id_torque[1] is False
    assert robot._driver._per_id_torque[2] is False

    robot.disable_joint_return_assist()
