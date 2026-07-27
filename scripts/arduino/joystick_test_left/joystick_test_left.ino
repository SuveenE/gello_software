/*
 * joystick_test_left.ino — Arduino Nano LEFT joystick streamer for GELLO tooling.
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
 * Board identity: this sketch is tagged LEFT (translation: forward/strafe).
 * On boot, and whenever the host sends a '?' byte, the board replies with:
 *   # ID:LEFT
 * Banner lines start with '#' so the CSV parser ignores them. This lets you
 * tell two otherwise-identical clone boards apart (e.g. two FTDI chips sharing
 * the same USB serial) regardless of which /dev/ttyUSB* they enumerate as, and
 * lets the client auto-assign roles with `--auto-id`.
 *
 * Upload with the Arduino IDE / arduino-cli, selecting the "Arduino Nano"
 * board (use the "ATmega328P (Old Bootloader)" processor variant if a normal
 * upload fails on a clone board).
 */

#define JOYSTICK_ID "LEFT"  // left stick -> translation (forward/strafe)

const int PIN_VRX = A0;
const int PIN_VRY = A1;
const int PIN_SW = 2;

const unsigned long SAMPLE_INTERVAL_MS = 20;  // ~50 Hz

unsigned long last_sample_ms = 0;

void print_id_banner() {
  if (sizeof(JOYSTICK_ID) > 1) {  // non-empty string literal
    Serial.print(F("# ID:"));
    Serial.println(F(JOYSTICK_ID));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);  // switch pulls to GND when pressed
  print_id_banner();
}

void loop() {
  // Respond to an identity query at any time without blocking the sample loop.
  while (Serial.available() > 0) {
    if (Serial.read() == '?') {
      print_id_banner();
    }
  }

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
