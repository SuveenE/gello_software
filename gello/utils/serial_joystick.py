"""Arduino Nano serial joystick reader and FlowBase axis mapping.

Pairs with scripts/arduino/joystick_test/joystick_test.ino (115200 baud CSV:
``<vrx>,<vry>,<sw>\\n``).
"""

from __future__ import annotations

import glob
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple

import numpy as np
import serial
from serial.tools import list_ports

ADC_MAX = 1023
ADC_CENTER = 512
ADC_HALF_SPAN = 512

# FlowBase gamepad path uses 0.05 for base translation/rotation.
FLOWBASE_DEADZONE = 0.05
DEFAULT_CROSS_AXIS_CONE_DEG = 25.0

# Right-stick (rotation + linear rail) constants. Yaw gets a wider horizontal
# cone than the rail's vertical cone; together they leave a 2 degree diagonal
# dead band in each quadrant (58 + 30 = 88 degrees).
RAIL_DEADZONE = 0.15  # Larger deadzone so a resting stick never drives the rail.
DEFAULT_RIGHT_STICK_HORIZONTAL_CONE_DEG = 58.0
DEFAULT_RIGHT_STICK_VERTICAL_CONE_DEG = 30.0
DEFAULT_LIFT_MAX_VEL_MS = 0.5  # Right-stick Y full deflection -> rail m/s.

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
    # On connect, sample the (assumed resting) stick for ``auto_center_secs`` and
    # use the median as center_x/center_y. Fixes off-center pots that otherwise
    # make one direction respond instantly and the opposite direction lag.
    auto_center: bool = True
    auto_center_secs: float = 1.0
    # Refuse to trust an auto-center this far from ADC_CENTER (means the stick
    # was almost certainly held/deflected during calibration): keep 512 instead.
    auto_center_max_offset: int = 250


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


def gate_to_cardinal(
    x: float,
    y: float,
    x_cone_ratio: float,
    y_cone_ratio: Optional[float] = None,
) -> Tuple[float, float]:
    """Keep each axis only when the push is near its OWN cardinal direction.

    Same as ``i2rt.utils.gamepad_utils.gate_to_cardinal``: used for the right
    stick so rotation (X) and rail (Y) do not cross-talk. ``x`` survives only
    when ``|y| <= x_cone_ratio * |x|`` and ``y`` survives only when
    ``|x| <= y_cone_ratio * |y|``. If ``y_cone_ratio`` is omitted, the
    symmetric legacy behavior is retained. A non-positive ratio disables the
    filter for its corresponding axis.
    """
    if y_cone_ratio is None:
        y_cone_ratio = x_cone_ratio
    ax, ay = abs(x), abs(y)
    x_out = x if x_cone_ratio <= 0.0 or ay <= x_cone_ratio * ax else 0.0
    y_out = y if y_cone_ratio <= 0.0 or ax <= y_cone_ratio * ay else 0.0
    return x_out, y_out


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


def screen_to_yaw_rail(
    screen_lr: float,
    screen_ud: float,
    *,
    cone_ratio: Optional[float] = None,
    yaw_cone_ratio: Optional[float] = None,
    rail_cone_ratio: Optional[float] = None,
    lift_max_vel_ms: float = DEFAULT_LIFT_MAX_VEL_MS,
    yaw_deadzone: float = FLOWBASE_DEADZONE,
    rail_deadzone: float = RAIL_DEADZONE,
) -> Tuple[float, float]:
    """Map a right-stick reading to ``(yaw, rail_mps)``.

    Left/right drives yaw (normalised ``[-1, 1]``, scaled by the controller's
    ``max_vel``); up/down drives the linear rail in physical m/s (up = positive).
    Yaw and rail may use different cardinal half-angles. ``cone_ratio`` remains
    as a backward-compatible symmetric setting when the axis-specific ratios
    are omitted.
    """
    if yaw_cone_ratio is None:
        yaw_cone_ratio = (
            cone_ratio
            if cone_ratio is not None
            else math.tan(math.radians(DEFAULT_RIGHT_STICK_HORIZONTAL_CONE_DEG))
        )
    if rail_cone_ratio is None:
        rail_cone_ratio = (
            cone_ratio
            if cone_ratio is not None
            else math.tan(math.radians(DEFAULT_RIGHT_STICK_VERTICAL_CONE_DEG))
        )
    yaw, rail = gate_to_cardinal(
        screen_lr,
        screen_ud,
        yaw_cone_ratio,
        rail_cone_ratio,
    )
    if abs(yaw) < yaw_deadzone:
        yaw = 0.0
    if abs(rail) < rail_deadzone:
        rail = 0.0
    return yaw, rail * lift_max_vel_ms


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


def _excluded_realpaths(exclude: Optional[Iterable[str]]) -> set:
    """Resolve exclusion entries to canonical realpaths for robust matching.

    Entries may be given as ``/dev/ttyUSB*`` device nodes or as
    ``/dev/serial/by-id`` / ``by-path`` symlinks; resolving each to its realpath
    lets us match regardless of which alias the caller (or udev) used. This is
    how we keep the joystick auto-id probe from ever opening the GELLO arm FTDI
    adapters, which would otherwise disturb the Dynamixel read loop (-3001).
    """
    resolved: set = set()
    if not exclude:
        return resolved
    for entry in exclude:
        if not entry:
            continue
        resolved.add(entry)
        try:
            resolved.add(os.path.realpath(entry))
        except OSError:
            pass
    return resolved


def _is_excluded(device: str, excluded: set) -> bool:
    if not excluded:
        return False
    if device in excluded:
        return True
    try:
        return os.path.realpath(device) in excluded
    except OSError:
        return False


def find_serial_ports(exclude: Optional[Iterable[str]] = None) -> List[str]:
    excluded = _excluded_realpaths(exclude)
    preferred: List[str] = []
    others: List[str] = []
    for p in list_ports.comports():
        if _is_excluded(p.device, excluded):
            continue
        vid = getattr(p, "vid", None)
        if vid in KNOWN_VID_PID:
            preferred.append(p.device)
        else:
            others.append(p.device)
    for dev in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if _is_excluded(dev, excluded):
            continue
        if dev not in preferred and dev not in others:
            others.append(dev)
    return preferred + others


def _symlink_targets(directory: str) -> dict:
    """Map realpath(target) -> [symlink, ...] for a /dev/serial/by-* directory."""
    mapping: dict = {}
    try:
        entries = os.listdir(directory)
    except OSError:
        return mapping
    for name in entries:
        link = os.path.join(directory, name)
        try:
            target = os.path.realpath(link)
        except OSError:
            continue
        mapping.setdefault(target, []).append(link)
    return mapping


@dataclass
class PortInfo:
    """Stable-identity metadata for a serial port."""

    device: str
    by_id: Optional[str] = None
    by_path: Optional[str] = None
    serial_number: Optional[str] = None
    vid: Optional[int] = None
    pid: Optional[int] = None
    description: Optional[str] = None

    def describe(self) -> str:
        vid_pid = ""
        if self.vid is not None and self.pid is not None:
            vid_pid = f" [{self.vid:04x}:{self.pid:04x}]"
        serial = f" serial={self.serial_number}" if self.serial_number else ""
        by_id = f"\n      by-id:   {self.by_id}" if self.by_id else ""
        by_path = f"\n      by-path: {self.by_path}" if self.by_path else ""
        return f"{self.device}{vid_pid}{serial}{by_id}{by_path}"


def list_joystick_ports(exclude: Optional[Iterable[str]] = None) -> List[PortInfo]:
    """Return rich, stable-identity info for candidate joystick serial ports."""
    by_id = _symlink_targets("/dev/serial/by-id")
    by_path = _symlink_targets("/dev/serial/by-path")
    infos: List[PortInfo] = []
    for dev in find_serial_ports(exclude=exclude):
        real = os.path.realpath(dev)
        meta = next(
            (p for p in list_ports.comports() if p.device == dev),
            None,
        )
        infos.append(
            PortInfo(
                device=dev,
                by_id=(by_id.get(real, [None])[0]),
                by_path=(by_path.get(real, [None])[0]),
                serial_number=getattr(meta, "serial_number", None),
                vid=getattr(meta, "vid", None),
                pid=getattr(meta, "pid", None),
                description=getattr(meta, "description", None),
            )
        )
    return infos


ID_BANNER_PREFIX = "# ID:"


def probe_port_id(
    device: str,
    baud: int = 115200,
    boot_wait: float = 2.0,
    read_timeout: float = 2.5,
    query_interval: float = 0.25,
) -> Optional[str]:
    """Return the firmware ``JOYSTICK_ID`` reported by the board on ``device``.

    Opening the port resets most Nano boards, so we wait for boot, then keep
    sending a ``?`` query byte while looking for the ``# ID:<value>`` banner the
    sketch prints. We re-send ``?`` throughout the read window rather than once:
    reset timing differs across OS/USB drivers (notably macOS vs Linux FTDI), so
    a single query can land while the board is still booting and get dropped.
    Returns ``None`` if the board reports no ID (older/blank firmware) or the
    port cannot be read. Consumes the serial port for the duration of the probe.
    """
    try:
        ser = serial.Serial(device, baud, timeout=0.2)
    except (serial.SerialException, OSError):
        return None
    try:
        time.sleep(boot_wait)
        ser.reset_input_buffer()
        deadline = time.time() + read_timeout
        next_query = 0.0
        while time.time() < deadline:
            now = time.time()
            if now >= next_query:
                try:
                    ser.write(b"?")
                except (serial.SerialException, OSError):
                    pass
                next_query = now + query_interval
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith(ID_BANNER_PREFIX):
                return line[len(ID_BANNER_PREFIX):].strip()
        return None
    finally:
        ser.close()


def resolve_ports_by_firmware_id(
    wanted_ids: List[str],
    baud: int = 115200,
    exclude: Optional[Iterable[str]] = None,
    fallback_single: bool = False,
) -> dict:
    """Map each wanted firmware ID to the serial device reporting it.

    Probes every candidate port once. Returns ``{id: device}`` for the ids that
    were found (missing ids are simply absent from the dict). Robust to
    identical USB serial numbers / unstable device names because identity comes
    from the firmware, not the OS device path.

    ``exclude`` lists serial devices the probe must never open (matched by
    realpath, so by-id/by-path aliases resolve too). Pass the GELLO arm ports
    here so the probe's port reset / ``?`` writes don't collide with the running
    Dynamixel leader servers and trigger COMM_RX_TIMEOUT (-3001).

    ``fallback_single``: when exactly one of two wanted ids is firmware-identified
    and exactly one other USB-serial candidate port remains unclaimed (e.g. a
    board flashed with a blank/old id that reports no banner), infer that leftover
    as the missing role. Only fires in the unambiguous 1-found / 1-leftover case,
    so extra serial adapters won't cause a mis-assignment.
    """
    infos = list_joystick_ports(exclude=exclude)
    id_by_device: dict = {}
    found: dict = {}
    for info in infos:
        jid = probe_port_id(info.device, baud=baud)
        id_by_device[info.device] = jid
        if jid in wanted_ids and jid not in found:
            found[jid] = info.device

    if fallback_single and len(wanted_ids) == 2:
        missing = [i for i in wanted_ids if i not in found]
        if len(missing) == 1:
            claimed = set(found.values())
            # Only consider real USB-serial adapters (known VID), never the
            # built-in ttyS* ports, and skip anything already positively
            # identified as the other role.
            leftovers = [
                info.device
                for info in infos
                if info.vid in KNOWN_VID_PID
                and info.device not in claimed
                and id_by_device.get(info.device) not in wanted_ids
            ]
            if len(leftovers) == 1:
                found[missing[0]] = leftovers[0]
                print(
                    f"[auto-id fallback] {missing[0]} not firmware-identified; "
                    f"assigning the only remaining joystick port {leftovers[0]!r} to it."
                )
            elif len(leftovers) > 1:
                print(
                    f"[auto-id fallback] {missing[0]} not firmware-identified and "
                    f"{len(leftovers)} unclaimed joystick ports remain "
                    f"({leftovers}); refusing to guess. Flash a firmware id or pin "
                    f"--{missing[0].lower()}-port explicitly."
                )
    return found


def format_port_table(
    infos: Optional[List[PortInfo]] = None,
    probe_ids: bool = False,
    exclude: Optional[Iterable[str]] = None,
) -> str:
    if infos is None:
        infos = list_joystick_ports(exclude=exclude)
    if not infos:
        return "No serial ports found."
    lines = [f"Found {len(infos)} serial port(s):"]
    for i, info in enumerate(infos):
        suffix = ""
        if probe_ids:
            if not os.access(info.device, os.R_OK | os.W_OK):
                suffix = (
                    "\n      firmware-id: (no access — add your user to the "
                    "'dialout' group: sudo usermod -aG dialout $USER)"
                )
            else:
                jid = probe_port_id(info.device)
                suffix = (
                    f"\n      firmware-id: {jid}" if jid else "\n      firmware-id: (none)"
                )
        lines.append(f"  [{i}] {info.describe()}{suffix}")
    return "\n".join(lines)


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

    def __init__(self, config: SerialJoystickConfig, *, timeout: float = 1.0):
        self.config = config
        self._cone_ratio = math.tan(math.radians(config.cross_axis_cone_deg))
        port = resolve_port(config.port)
        try:
            self._ser = serial.Serial(port, config.baud, timeout=timeout)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open {port}: {e}") from e
        self.port = port
        time.sleep(2.0)
        self._ser.reset_input_buffer()

        if config.auto_center:
            self._auto_center(config.auto_center_secs)

        self._latest: Optional[SerialJoystickSample] = None
        self._latest_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

    def _auto_center(self, duration: float) -> None:
        """Measure the resting stick position and adopt it as the axis center.

        Assumes the stick is untouched during the window. Uses the median (robust
        to the occasional garbled line) and only trusts the result when it lands
        within ``auto_center_max_offset`` of ``ADC_CENTER`` — a wildly off value
        means the stick was held mid-deflection, so we keep the configured center.
        """
        cfg = self.config
        xs: List[int] = []
        ys: List[int] = []
        deadline = time.time() + max(duration, 0.0)
        while time.time() < deadline:
            parsed = parse_line(self._ser.readline().decode(errors="replace"))
            if parsed is None:
                continue
            x, y, _ = parsed
            xs.append(x)
            ys.append(y)
        if len(xs) < 5:
            print(
                f"[auto-center {self.port}] too few samples ({len(xs)}); "
                f"keeping center {cfg.center_x}/{cfg.center_y}."
            )
            return
        cx = int(round(float(np.median(xs))))
        cy = int(round(float(np.median(ys))))
        if (
            abs(cx - ADC_CENTER) > cfg.auto_center_max_offset
            or abs(cy - ADC_CENTER) > cfg.auto_center_max_offset
        ):
            print(
                f"[auto-center {self.port}] measured {cx}/{cy} is far from "
                f"{ADC_CENTER} (stick held during calibration?); keeping "
                f"{cfg.center_x}/{cfg.center_y}."
            )
            return
        cfg.center_x, cfg.center_y = cx, cy
        print(f"[auto-center {self.port}] center set to {cx}/{cy} (VRX/VRY at rest).")
        self._ser.reset_input_buffer()

    def start_background(self) -> SerialJoystick:
        """Start a daemon thread that keeps ``get_latest()`` fresh.

        Use this when reading multiple joysticks concurrently: a blocking
        ``read_sample_blocking()`` on one port would otherwise stall the others.
        """
        if self._reader_thread is not None:
            return self
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name=f"joystick-{self.port}", daemon=True
        )
        self._reader_thread.start()
        return self

    def _reader_loop(self) -> None:
        while self._running:
            try:
                sample = self.read_sample()
            except (serial.SerialException, OSError):
                break
            if sample is not None:
                with self._latest_lock:
                    self._latest = sample

    def get_latest(self) -> Optional[SerialJoystickSample]:
        """Return the most recent sample from the background reader (or None)."""
        with self._latest_lock:
            return self._latest

    def close(self) -> None:
        self._running = False
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.5)
            self._reader_thread = None
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
