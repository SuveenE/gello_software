"""Arduino Nano serial joystick reader and FlowBase axis mapping.

Pairs with scripts/arduino/joystick_test/joystick_test.ino (115200 baud CSV:
``<vrx>,<vry>,<sw>\\n``).
"""

from __future__ import annotations

import glob
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import serial
from serial.tools import list_ports

ADC_MAX = 1023
ADC_CENTER = 512
ADC_HALF_SPAN = 512

# FlowBase gamepad path uses 0.05 for base translation/rotation.
FLOWBASE_DEADZONE = 0.05
DEFAULT_CROSS_AXIS_CONE_DEG = 25.0

KNOWN_VID_PID = (0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4)


@dataclass
class SerialJoystickConfig:
    port: Optional[str] = None
    baud: int = 115200
    deadzone: float = FLOWBASE_DEADZONE
    cross_axis_cone_deg: float = DEFAULT_CROSS_AXIS_CONE_DEG
    swap_xy: bool = True
    invert_x: bool = False
    invert_y: bool = False
    center_x: int = ADC_CENTER
    center_y: int = ADC_CENTER
    half_span: int = ADC_HALF_SPAN


def apply_axis_dominance(x: float, y: float, cone_ratio: float) -> tuple[float, float]:
    """Same left-stick cross-talk filter as i2rt.utils.gamepad_utils."""
    if cone_ratio <= 0.0:
        return x, y
    ax, ay = abs(x), abs(y)
    if ax >= ay:
        if ay < cone_ratio * ax:
            y = 0.0
    elif ax < cone_ratio * ay:
        x = 0.0
    return x, y


def normalize_axis(raw: int, center: int, half_span: int, deadzone: float) -> float:
    value = (raw - center) / max(half_span, 1)
    value = max(-1.0, min(1.0, value))
    if abs(value) < deadzone:
        return 0.0
    return value


def apply_axis_map(
    nx: float,
    ny: float,
    *,
    swap_xy: bool,
    invert_x: bool,
    invert_y: bool,
) -> Tuple[float, float]:
    if swap_xy:
        nx, ny = ny, nx
    if invert_x:
        nx = -nx
    if invert_y:
        ny = -ny
    return nx, ny


def screen_to_flowbase(
    screen_lr: float,
    screen_ud: float,
    *,
    cone_ratio: float,
    deadzone: float = FLOWBASE_DEADZONE,
) -> np.ndarray:
    """Map screen axes to normalised FlowBase ``[forward, strafe, yaw]``."""
    forward, strafe = screen_ud, screen_lr
    forward, strafe = apply_axis_dominance(forward, strafe, cone_ratio)
    cmd = np.array([forward, strafe, 0.0])
    cmd[np.abs(cmd) < deadzone] = 0.0
    return cmd


def parse_line(line: str) -> Optional[Tuple[int, int, int]]:
    parts = line.strip().split(",")
    if len(parts) != 3:
        return None
    try:
        x, y, sw = int(parts[0]), int(parts[1]), int(parts[2])
    except ValueError:
        return None
    if not (0 <= x <= ADC_MAX and 0 <= y <= ADC_MAX):
        return None
    return x, y, sw


def find_serial_ports() -> List[str]:
    preferred: List[str] = []
    others: List[str] = []
    for p in list_ports.comports():
        vid = getattr(p, "vid", None)
        if vid in KNOWN_VID_PID:
            preferred.append(p.device)
        else:
            others.append(p.device)
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if dev not in preferred and dev not in others:
            others.append(dev)
    return preferred + others


def resolve_port(requested: Optional[str]) -> str:
    if requested:
        return requested
    candidates = find_serial_ports()
    if not candidates:
        raise RuntimeError("No serial ports found. Pass port explicitly.")
    if len(candidates) > 1:
        print("Multiple serial ports found:")
        for i, dev in enumerate(candidates):
            print(f"  [{i}] {dev}")
        print(f"Using {candidates[0]} (override with --port).")
    return candidates[0]


@dataclass
class SerialJoystickSample:
    raw_x: int
    raw_y: int
    button_pressed: bool
    screen_lr: float
    screen_ud: float
    user_cmd: np.ndarray


class SerialJoystick:
    """Read Arduino joystick serial stream and expose FlowBase user commands."""

    def __init__(self, config: SerialJoystickConfig):
        self.config = config
        self._cone_ratio = math.tan(math.radians(config.cross_axis_cone_deg))
        port = resolve_port(config.port)
        try:
            self._ser = serial.Serial(port, config.baud, timeout=1.0)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open {port}: {e}") from e
        self.port = port
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()

    def read_sample(self) -> Optional[SerialJoystickSample]:
        line = self._ser.readline().decode(errors="replace")
        parsed = parse_line(line)
        if parsed is None:
            return None
        x, y, sw = parsed
        cfg = self.config
        nx_raw = normalize_axis(x, cfg.center_x, cfg.half_span, cfg.deadzone)
        ny_raw = normalize_axis(y, cfg.center_y, cfg.half_span, cfg.deadzone)
        screen_lr, screen_ud = apply_axis_map(
            nx_raw,
            ny_raw,
            swap_xy=cfg.swap_xy,
            invert_x=cfg.invert_x,
            invert_y=cfg.invert_y,
        )
        user_cmd = screen_to_flowbase(
            screen_lr,
            screen_ud,
            cone_ratio=self._cone_ratio,
            deadzone=FLOWBASE_DEADZONE,
        )
        return SerialJoystickSample(
            raw_x=x,
            raw_y=y,
            button_pressed=sw == 0,
            screen_lr=screen_lr,
            screen_ud=screen_ud,
            user_cmd=user_cmd,
        )

    def read_sample_blocking(self) -> SerialJoystickSample:
        while True:
            sample = self.read_sample()
            if sample is not None:
                return sample
