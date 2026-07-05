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
| `scripts/flow_base_serial_joystick_client.py` | **Drive the base** over RPC |
| `gello/utils/serial_joystick.py` | Shared reader + axis mapping |

## Axis mapping

Defaults match a 90°-mounted stick (no extra flags needed):

| Physical (on screen) | FlowBase command |
|----------------------|------------------|
| Up / down | Forward / back (`user_cmd[0]`) |
| Left / right | Strafe (`user_cmd[1]`) |
| — | Yaw = 0 (single stick, no rotation) |
| Button press | Toggle local ↔ global frame |

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

On the **operator PC** with the Nano plugged in:

```bash
cd ~/lerobot/gello_software
python3 scripts/flow_base_serial_joystick_client.py \
  --host 172.6.2.20 \
  --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0
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

- Single stick covers **translation only** (no yaw or linear rail lift).
- For full 4-DOF control (base + rail + rotation), use the original USB gamepad or add a second stick / extra inputs.
