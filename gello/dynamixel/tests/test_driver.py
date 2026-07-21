import numpy as np
import pytest

from gello.dynamixel.driver import (
    CURRENT_BASED_POSITION_CONTROL_MODE,
    POSITION_CONTROL_MODE,
    FakeDynamixelDriver,
)


@pytest.fixture
def fake_driver():
    return FakeDynamixelDriver(ids=[1, 2])


def test_set_joints(fake_driver):
    fake_driver.set_torque_mode(True)
    fake_driver.set_joints([np.pi / 2, np.pi / 2])
    assert np.allclose(fake_driver.get_joints(), [np.pi / 2, np.pi / 2])


def test_set_joints_wrong_length(fake_driver):
    with pytest.raises(ValueError):
        fake_driver.set_joints([np.pi / 2])


def test_set_joints_torque_disabled(fake_driver):
    with pytest.raises(RuntimeError):
        fake_driver.set_joints([np.pi / 2, np.pi / 2])


def test_torque_enabled(fake_driver):
    assert not fake_driver.torque_enabled()
    fake_driver.set_torque_mode(True)
    assert fake_driver.torque_enabled()


def test_get_joints(fake_driver):
    assert np.allclose(fake_driver.get_joints(), [0, 0])


def test_set_operating_mode_for_ids_is_per_id(fake_driver):
    fake_driver.set_operating_mode_for_ids([1], CURRENT_BASED_POSITION_CONTROL_MODE)
    assert fake_driver._operating_modes[1] == CURRENT_BASED_POSITION_CONTROL_MODE
    # Untouched IDs keep the default mode.
    assert fake_driver._operating_modes[2] == POSITION_CONTROL_MODE


def test_verify_operating_mode_for_ids(fake_driver):
    fake_driver.set_operating_mode_for_ids([1], CURRENT_BASED_POSITION_CONTROL_MODE)
    fake_driver.verify_operating_mode_for_ids(
        [1], CURRENT_BASED_POSITION_CONTROL_MODE
    )
    with pytest.raises(RuntimeError):
        fake_driver.verify_operating_mode_for_ids([2], CURRENT_BASED_POSITION_CONTROL_MODE)


def test_set_torque_mode_for_ids_is_per_id(fake_driver):
    fake_driver.set_torque_mode_for_ids([1], True)
    assert fake_driver._per_id_torque[1] is True
    assert fake_driver._per_id_torque[2] is False


def test_set_goal_currents_and_positions_for_ids(fake_driver):
    fake_driver.set_goal_currents_for_ids({1: 100.0})
    fake_driver.set_goal_positions_for_ids({1: 1.5})
    assert fake_driver._goal_currents == {1: 100.0}
    assert fake_driver._goal_positions == {1: 1.5}


def test_read_temperatures_for_ids(fake_driver):
    fake_driver._temperatures[1] = 42.0
    temps = fake_driver.read_temperatures([1, 2])
    assert temps == {1: 42.0, 2: 25.0}


def test_set_bus_watchdog_for_ids(fake_driver):
    fake_driver.set_bus_watchdog_for_ids([1], 1.0)
    assert fake_driver._bus_watchdog[1] == 1.0
