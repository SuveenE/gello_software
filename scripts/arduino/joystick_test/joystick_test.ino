/*
 * joystick_test.ino — Arduino Nano joystick streamer for GELLO tooling.
 *
 * Wiring (matches scripts/test_joystick.py):
 *   Joystick GND  -> Nano GND
 *   Joystick +5V  -> Nano 5V
 *   Joystick VRX  -> Nano A0
 *   Joystick VRY  -> Nano A1
 *   Joystick SW   -> Nano D2
 *
 * Serial protocol: one CSV line per sample at 115200 baud:
 *   <x>,<y>,<sw>\n
 * where x,y are 0..1023 raw ADC counts and sw is 1 (released) or 0 (pressed).
 *
 * Upload with the Arduino IDE / arduino-cli, selecting the "Arduino Nano"
 * board (use the "ATmega328P (Old Bootloader)" processor variant if a normal
 * upload fails on a clone board).
 */

const int PIN_VRX = A0;
const int PIN_VRY = A1;
const int PIN_SW = 2;

const unsigned long SAMPLE_INTERVAL_MS = 20;  // ~50 Hz

unsigned long last_sample_ms = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);  // switch pulls to GND when pressed
}

void loop() {
  const unsigned long now = millis();
  if (now - last_sample_ms < SAMPLE_INTERVAL_MS) {
    return;
  }
  last_sample_ms = now;

  const int x = analogRead(PIN_VRX);
  const int y = analogRead(PIN_VRY);
  const int sw = digitalRead(PIN_SW);  // 1 = released, 0 = pressed

  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.println(sw);
}
