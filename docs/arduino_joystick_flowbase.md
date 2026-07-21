# Arduino Joystick → FlowBase

Custom Arduino Nano joystick for teleoperating the i2rt FlowBase / LinearBot base, as an alternative to the USB gamepad remote.

## Hardware

### Wiring

```
Joystick GND  -> Nano GND
Joystick +5V  -> Nano 5V
Joystick VRX  -> Nano A0
Joystick VRY  -> Nano A1
Joystick SW   -> Nano D2
```

### Arduino sketch

Upload `scripts/arduino/joystick_test/joystick_test.ino` to the Nano.

- Board: **Arduino Nano**
- If upload fails on a clone: **Processor → ATmega328P (Old Bootloader)**
- Baud: **115200**

Serial format (one line per sample, ~50 Hz):

```
<vrx>,<vry>,<sw>
```

| Field | Range | Meaning |
|-------|-------|---------|
| `vrx` | 0–1023 | A0 raw ADC |
| `vry` | 0–1023 | A1 raw ADC |
| `sw` | 0 or 1 | 0 = pressed, 1 = released |

## WSL USB setup

On Windows (PowerShell as Admin):

```powershell
usbipd list
usbipd attach --wsl --busid <BUSID>
```

In WSL, find the port:

```bash
ls /dev/serial/by-id/
# e.g. usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 -> ttyUSB0
```

If you get permission errors:

```bash
sudo usermod -aG dialout $USER
# restart WSL: wsl --shutdown (from PowerShell)
```

## Python dependencies

```bash
pip install pyserial numpy portal
```

Or from the i2rt repo (includes `portal`):

```bash
cd ~/lerobot/i2rt && pip install -e .
```

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/test_joystick.py` | Test stick + button, screen dashboard |
| `scripts/test_serial_joystick_flowbase.py` | Preview FlowBase mapping (safe, no robot motion) |
| `scripts/flow_base_serial_joystick_client.py` | **Drive the base** over RPC (1 or 2 sticks) |
| `gello/utils/serial_joystick.py` | Shared reader + axis mapping + port detection |

## Telling two sticks apart

Two USB serial adapters enumerate as `/dev/ttyUSB0` and `/dev/ttyUSB1`, but the
order can swap between reboots. Use a **stable identity** instead:

- **`/dev/serial/by-id/...`** — unique per chip. FTDI (FT232R) boards each carry
  a unique serial number, so this is the most robust. Many CH340 clones ship
  with **no serial number**, so their by-id paths collide (fall back to by-path
  or a firmware ID below).
- **`/dev/serial/by-path/...`** — unique per physical USB port. Works even for
  identical CH340 clones, as long as each stick stays in the same port.
- **Firmware ID** — set `#define JOYSTICK_ID "LEFT"` / `"RIGHT"` in
  `joystick_test.ino` and flash each board. On boot / on a `?` byte the board
  prints `# ID:LEFT`. This survives cable swaps and clone collisions.

List detected ports (with stable paths and each board's firmware id):

```bash
python3 scripts/flow_base_serial_joystick_client.py --list
```

Handy raw commands:

```bash
# Linux
ls -l /dev/serial/by-id/ /dev/serial/by-path/
udevadm info -q property -n /dev/ttyUSB0 | grep -E 'ID_SERIAL|ID_PATH|ID_VENDOR_ID'

# macOS
ls -1 /dev/cu.usbserial-*
ioreg -r -c IOUSBHostDevice -l | grep -E 'USB Product Name|USB Serial Number'
```

### Both adapters show the SAME serial number

Cheap/clone **FTDI FT232R** (and many CH340) chips are often flashed with an
identical hard-coded serial (e.g. every board reports `A5069RR4`). Then:

- **Linux** still creates two nodes (`ttyUSB0`, `ttyUSB1`), but their
  `by-id` paths collide. Use **`/dev/serial/by-path/...`** (unique per physical
  USB port) — no reflash needed. This is the easy fix on the Leader PC.
- **macOS** derives the node name from the serial, so you get one
  `/dev/cu.usbserial-A5069RR4` and a fallback like `/dev/cu.usbserial-3`, and
  the mapping is not stable across replug.

Two reliable fixes that work everywhere:

1. **Firmware id (recommended, no hardware tools).** Flash each Nano with a
   unique `#define JOYSTICK_ID "LEFT"` / `"RIGHT"` in `joystick_test.ino`, then
   let the client assign roles by asking each board who it is:

   ```bash
   python3 scripts/flow_base_serial_joystick_client.py --host 192.168.50.91 --auto-id
   ```

   `--auto-id` probes every port, reads the `# ID:` banner, and maps
   `--left-id`/`--right-id` (default `LEFT`/`RIGHT`) to the right device
   regardless of the OS device name. `--list` shows each board's reported id.

2. **Reprogram a unique USB serial into the FTDI EEPROM** (permanent). Plug in
   ONE board at a time and write a new serial with `pyftdi`:

   ```bash
   pip install pyftdi
   # with a single FTDI attached:
   ftconf ftdi://ftdi:232 -s LEFTJOY -o /dev/null   # then repeat for RIGHTJOY
   ```

   (On Windows, FTDI's FT_PROG does the same. Avoid FTDI's old bricking driver;
   pyftdi/libftdi are safe.) After this, `by-id` / `usbserial-<serial>` names
   are unique again.

## Axis mapping

### Single stick (translation only)

Defaults match a 90°-mounted stick (no extra flags needed):

| Physical (on screen) | FlowBase command |
|----------------------|------------------|
| Up / down | Forward / back (`user_cmd[0]`) |
| Left / right | Strafe (`user_cmd[1]`) |
| — | Yaw = 0 (single stick, no rotation) |
| Button press | Toggle local ↔ global frame |

### Two sticks (full 4-DOF, matches the USB gamepad)

Pass both `--left-port` and `--right-port` to enable dual mode:

| Stick | Physical | FlowBase command |
|-------|----------|------------------|
| Left | Up / down | Forward / back (`user_cmd[0]`) |
| Left | Left / right | Strafe (`user_cmd[1]`) |
| Left | Button | Toggle local ↔ global frame |
| Right | Left / right | Yaw / rotation (`user_cmd[2]`) |
| Right | Up / down | Linear rail m/s (up = raise) |
| Right | Button | Reset odometry |

The right stick uses asymmetric cardinal gates so rotation and rail don't
cross-talk: the left/right yaw cones have a 58° half-angle and the top/bottom
rail cones have a 30° half-angle, leaving a 2° diagonal dead band. Override
them with `--right-stick-horizontal-cone-deg` and
`--right-stick-vertical-cone-deg`; `--right-stick-cone-deg` remains a legacy
symmetric override. Set rail scaling with `--lift-max-vel-ms`.

Normalization uses fixed ADC center **512** and span **512** (no startup calibration wiggle).

FlowBase applies the same filters as the USB gamepad path:

- Deadzone: **0.05**
- Cross-axis cone: **25°** (reduces diagonal bleed)
- Max velocity: **0.5 m/s** forward/strafe, **π/2 rad/s** yaw (on the controller)

## Usage

### 1. Test the joystick

```bash
cd ~/lerobot/gello_software
python3 scripts/test_joystick.py \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

### 2. Preview FlowBase mapping (no robot motion)

```bash
python3 scripts/test_serial_joystick_flowbase.py \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

### 3. Drive the base

On the **base PC** (Pi), start the controller without a local gamepad:

```bash
python3 i2rt/i2rt/flow_base/flow_base_controller.py \
  --channel can_linearbot \
  --gpio-host <GPIO_HOST> \
  --no-gamepad
```

On the **operator PC** with the Nano(s) plugged in.

Single stick (translation only):

```bash
cd ~/lerobot/gello_software
python3 scripts/flow_base_serial_joystick_client.py \
  --host 172.6.2.20 \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
```

Two sticks (full 4-DOF: left translates, right rotates + lifts the rail):

```bash
cd ~/lerobot/gello_software
python3 scripts/flow_base_serial_joystick_client.py \
  --host 172.6.2.20 \
  --left-port  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0 \
  --right-port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0012XYZ-if00-port0
```

Commands stream at 50 Hz. If the client disconnects, the base stops after **0.2 s**.

## Tuning flags

If an axis is reversed or neutral is off-center:

```bash
--invert-x          # flip screen left/right
--invert-y          # flip screen up/down
--no-swap-xy        # disable default VRX/VRY swap
--center-x 480      # resting ADC for A0
--center-y 510      # resting ADC for A1
--half-span 400     # if full push doesn't reach ±1.0
```

## Branch

All joystick + FlowBase tooling lives on **`weining-joystick`** in `gello_software` and `lerobot`.

## Limitations

- A **single** stick covers **translation only** (no yaw or linear rail lift).
- For full 4-DOF control (translation + rotation + rail), use **two sticks**
  (`--left-port` + `--right-port`) or the original USB gamepad.
