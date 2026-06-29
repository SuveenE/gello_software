# GELLO ↔ YAM Bring-up Notes (Ubuntu)

Working notes for running the GELLO leaders against the YAM follower arms on a
native Ubuntu PC (migrated from WSL). Covers CAN/serial setup, simulation and
hardware teleop, calibration, gravity compensation, and the trigger-freeze
teleop tool.

---

## 1. Hardware overview

| Component | Identifier |
| --- | --- |
| Left GELLO leader (U2D2) | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WA0-if00-port0` |
| Right GELLO leader (U2D2) | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WI5-if00-port0` |
| Left YAM CAN dongle | serial `0047004B33354B0332333837` → `can_left` |
| Right YAM CAN dongle | serial `0036004733354B0332333837` → `can_right` |

- GELLO leaders are Dynamixel servos on a U2D2 USB adapter (baudrate `57600`,
  joint IDs `1..6` for the arm + `7` for the gripper/trigger).
- YAM followers are I2RT arms on a `gs_usb` (Geschwister Schneider) CAN adapter,
  1 Mbps bitrate, 7 motors each (6 arm + 1 gripper).

> The U2D2 only carries **data**. The GELLO motors need their own power supply;
> if the port opens but every motor times out (`-3001`), suspect motor power.

---

## 2. CAN setup (native Ubuntu)

The USB-CAN adapters are mapped to stable names (`can_left` / `can_right`) via a
udev rule keyed on each adapter's serial number, and brought up at 1 Mbps.

Files:
- `scripts/90-can.rules.template` — udev rules (serial → name + auto bring-up).
- `scripts/setup_can_ubuntu.sh` — installs rules, reloads udev, brings interfaces up, verifies.

Bring up CAN:

```bash
sudo bash scripts/setup_can_ubuntu.sh
```

Verify the YAM motors respond (expect `[1, 2, 3, 4, 5, 6, 7]`):

```bash
python -m i2rt.motor_config_tool.ping_motors --channel can_left
python -m i2rt.motor_config_tool.ping_motors --channel can_right
```

If motors appear on the wrong arm, swap the two serial lines in
`/etc/udev/rules.d/90-can.rules`.

---

## 3. Environment

Use the `lerobot` conda env (Python 3.12) — it has `dm_control` (for sim) and
`pinocchio` (for gravity comp). The system Python 3.13 cannot build `dm_control`.

```bash
conda activate lerobot
cd ~/gello_software
```

One-time dependency notes:
- DynamixelSDK submodule: `git submodule update --init third_party/DynamixelSDK`
  then `pip install -e third_party/DynamixelSDK/python`.
- Pinocchio (gravity comp): install via conda-forge to get the right shared libs:
  `conda install -c conda-forge pinocchio -y` (the pip build hit a
  `liburdfdom_sensor.so` load error).

---

## 4. Configs

Calibrated configs were generated for each arm (offsets via
`scripts/gello_get_offset.py`, with the physical arms in the YAM default pose).

| Config | Use | Leader port | Follower | Server port |
| --- | --- | --- | --- | --- |
| `configs/yam_left_sim.yaml` | Left, simulation | `FTAO9WA0` | MuJoCo | 6001 |
| `configs/yam_right_sim.yaml` | Right, simulation | `FTAO9WI5` | MuJoCo | 6002 |
| `configs/yam_left_hw.yaml` | Left, real YAM | `FTAO9WA0` | `can_left` | 6001 |
| `configs/yam_right_hw.yaml` | Right, real YAM | `FTAO9WI5` | `can_right` | 6002 |
| `configs/yam_gello_factr_left_hw.yaml` | Left gravity comp + teleop | `FTAO9WA0` | `can_left` | — |

Notes:
- Gripper direction was flipped per side via `gripper_config` (open/closed degrees).
- Bimanual hardware uses distinct `hardware_server_port` (6001 / 6002) to avoid a
  ZMQ "address already in use" clash.
- The hardware configs use a **safe gradual startup** (`move_to_start_target: gello`):
  the YAM slowly tracks the GELLO pose on launch instead of jerking.

---

## 5. Running teleop

### Simulation

```bash
# single arm
python experiments/launch_yaml.py --left-config-path configs/yam_left_sim.yaml
# both arms
python experiments/launch_yaml.py \
  --left-config-path configs/yam_left_sim.yaml \
  --right-config-path configs/yam_right_sim.yaml
```

### Real YAM hardware

```bash
sudo bash scripts/setup_can_ubuntu.sh   # ensure can_left/can_right are UP

# single arm
python experiments/launch_yaml.py --left-config-path configs/yam_left_hw.yaml
# both arms
python experiments/launch_yaml.py \
  --left-config-path configs/yam_left_hw.yaml \
  --right-config-path configs/yam_right_hw.yaml
```

> Bimanual needs **both** CAN dongles plugged in. If only one shows up in
> `setup_can_ubuntu.sh`, the second dongle isn't enumerating — check
> `lsusb | grep 1d50:606f` (should list two adapters).

---

## 6. Gravity compensation (FACTR)

Lets the GELLO leader hold itself against gravity while teleoperating.

```bash
python gello/factr/gravity_compensation.py --config configs/yam_gello_factr_left_hw.yaml
```

Changes made to `gello/factr/gravity_compensation.py`:
- Load the URDF with `pin.buildModelFromUrdf(...)` so it works **without** the
  collision/visual mesh files (the `.stl`s were missing locally).
- Support a **per-joint** `gravity_comp.gain` (list) in addition to a single
  scalar, for tuning each joint's hold strength independently.

Tuning lives in the config under `controller.gravity_comp.gain` (e.g.
`[0.6, 0.6, 0.6, 0.6, 0.6, 0.6]`). Raise a joint if it sags, lower if it floats.

Direction note: the follower net sign = `dynamixel.joint_signs * teleop.mapping.signs`.
Since `dynamixel.joint_signs` already encodes the working direction, keep
`teleop.mapping.signs = [1,1,1,1,1,1]` to avoid double-inverting joints.

---

## 7. Trigger-freeze teleop (`experiments/teleop_freeze.py`)

Normal bimanual teleop, but the 7th GELLO joint (the gripper/trigger) is
repurposed as a **freeze toggle** instead of driving the gripper:

- Squeeze the trigger **past half-way** → GELLO arm joints (1–6) **freeze** in
  place at full torque (self-hold at any pose).
- Squeeze again → **release**, arm is back-drivable again.
- The trigger motor itself is left un-braked so you can always press it again.

```bash
# single arm
python experiments/teleop_freeze.py --left-config-path configs/yam_left_hw.yaml
# both arms
python experiments/teleop_freeze.py \
  --left-config-path configs/yam_left_hw.yaml \
  --right-config-path configs/yam_right_hw.yaml
```

Implementation details:
- Freeze switches joints 1–6 to **Extended Position Control (mode 4, multi-turn)**
  and writes the captured goal position **before** enabling torque (no snap).
- Release **restores each joint's original operating mode** (e.g. Current Control
  `0`, as left by gravity comp).
- Both modes are multi-turn, so the reported position never re-wraps — this fixed
  a bug where the wrist-yaw joint snapped ~90° on release (caused by an earlier
  version switching to single-turn Position Control mode 3).
- Tuning knobs at the top of the file: `PRESS_THRESHOLD`, `RELEASE_THRESHOLD`.

---

## 8. Recovery / troubleshooting

**Always exit with `Ctrl+C`** (clean shutdown disables motor torque). A hard
kill (`Ctrl+\`) leaves the GELLO servos torque-enabled (feels like resistance)
and can leave processes holding the serial port / CAN bus.

Clear stale processes:

```bash
pkill -f "launch_yaml|gravity_compensation|teleop_freeze"
```

Reset the FTDI serial adapter (USB unbind/rebind, no physical unplug) — fixes
stuck `-3001` timeouts after a hard kill:

```bash
sudo bash scripts/reset_serial.sh           # all FTDI adapters
sudo bash scripts/reset_serial.sh ttyUSB0   # one port
```

Reset CAN:

```bash
sh scripts/reset_all_can.sh
# or full re-setup
sudo bash scripts/setup_can_ubuntu.sh
```

Disable GELLO leader torque (if it feels stiff after a crash):

```bash
python -c "
from gello.dynamixel.driver import DynamixelDriver
d = DynamixelDriver([1,2,3,4,5,6,7], port='/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO9WA0-if00-port0', baudrate=57600)
d.set_torque_mode(False); d.close()
"
```

Common gotchas:
- **Partial YAM motor response** (e.g. only `[1,5,6]` online): usually a stale
  process (gravity comp/teleop) is also reading the same CAN bus and stealing
  frames. Kill it, then re-ping.
- **`zmq.error ... Address already in use`**: a previous launch is still bound to
  the port; kill it or rely on the distinct 6001/6002 ports.
- **`-3001 COMM_RX_TIMEOUT` on GELLO**: reset serial; if it persists, check GELLO
  motor power.
