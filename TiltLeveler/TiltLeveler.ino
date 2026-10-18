/*
  TiltLeveler
  -----------
  Reads tilt from a GY-521 (MPU-6050) accelerometer, saves the current
  tilt as a reference when the button is pressed, then shows how far the
  board has moved from that reference:

    |difference| <= 0.25 deg          -> BLUE  LED (pin 9)
    0.25 < |difference| <= 0.50 deg   -> GREEN LED (pin 8)
    |difference| > 0.50 deg           -> RED   LED (pin 10)

  Until the button has been pressed once, all LEDs stay off.

  Wiring (Arduino Uno R3):
    MPU-6050 VCC -> 5V   (breadboard + rail; the GY-521 has its own regulator)
    MPU-6050 GND -> GND  (breadboard - rail)
    MPU-6050 SDA -> SDA  (same as A4)
    MPU-6050 SCL -> SCL  (same as A5)
    MPU-6050 AD0 -> not connected (I2C address 0x68)

    Button: one side to 5V, other side to pin 3, with a pull-down resistor
            from pin 3's side of the button to GND (pressed = HIGH).

    Blue LED  -> pin 9  (through resistor)
    Green LED -> pin 8  (through resistor)
    Red LED   -> pin 10 (through resistor)

  No extra libraries needed - only Wire, which ships with the Arduino IDE.
  Open the Serial Monitor at 9600 baud to watch the readings.

  At power-up each LED lights for half a second (blue, green, red) so you
  can check the LED wiring, then the sketch reports on the Serial Monitor
  whether it found the MPU-6050.
*/

#include <Wire.h>

// ---------------- Pins ----------------
const int BUTTON_PIN = 3;
const int BLUE_LED   = 9;
const int GREEN_LED  = 8;
const int RED_LED    = 10;

// ---------------- Thresholds (degrees) ----------------
const float BLUE_LIMIT  = 0.25;
const float GREEN_LIMIT = 0.50;

// ---------------- Which axis is "horizontal tilt"? ----------------
// The sensor is lying flat on the popsicle stick. Tilting the stick
// end-to-end (along its length) is usually one axis, side-to-side the other.
// Change to false if the reading doesn't change when you tilt the way you want.
const bool USE_ROLL = true;   // true = rotation about the sensor's X axis
                              // false = rotation about the sensor's Y axis

// ---------------- MPU-6050 registers ----------------
// 0x68 when AD0 is low/unconnected, 0x69 when AD0 is high. Detected at startup.
uint8_t mpuAddr = 0x68;
const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT = 0x6B;
const uint8_t REG_CONFIG   = 0x1A;
const uint8_t REG_ACCEL_CFG = 0x1C;
const uint8_t REG_ACCEL_X  = 0x3B;

// ---------------- Smoothing ----------------
// 0.25 deg is a very fine window, so the raw reading must be steadied.
// Each loop averages SAMPLES readings, then an exponential filter smooths
// further. Smaller ALPHA = steadier but slower to respond.
const int   SAMPLES = 10;
const float ALPHA   = 0.15;

// ---------------- State ----------------
float filteredAngle = 0.0;
float savedAngle    = 0.0;
bool  haveSaved     = false;

int  lastButtonReading = LOW;
int  buttonState       = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 30;

unsigned long lastPrint = 0;

// Write one byte to an MPU-6050 register.
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read raw accelerometer X, Y, Z. Returns false if the sensor didn't answer.
bool readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_ACCEL_X);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  return true;
}

// True if a device acknowledges at this I2C address.
bool devicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Print every address on the I2C bus, to help track down wiring problems.
void scanI2C() {
  Serial.println(F("Scanning I2C bus..."));
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (devicePresent(addr)) {
      Serial.print(F("  Device found at 0x"));
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  No I2C devices found. Check SDA, SCL, VCC and GND wiring."));
  }
}

// Find the MPU-6050 at 0x68 or 0x69. Returns false if it isn't there.
bool findMpu() {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t addr : candidates) {
    if (devicePresent(addr)) {
      mpuAddr = addr;
      return true;
    }
  }
  return false;
}

// Read one byte from an MPU-6050 register (0xFF on failure).
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

// Average several accelerometer readings and convert to a tilt angle in degrees.
// Returns NAN if the sensor can't be read.
float readTiltAngle() {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az;
    if (!readAccel(ax, ay, az)) return NAN;
    sumX += ax;
    sumY += ay;
    sumZ += az;
    delay(2);
  }
  float x = sumX / (float)SAMPLES;
  float y = sumY / (float)SAMPLES;
  float z = sumZ / (float)SAMPLES;

  // Using atan2 with the magnitude of the other two axes keeps the angle
  // accurate even when the board is tilted on both axes at once.
  if (USE_ROLL) {
    return atan2(y, sqrt(x * x + z * z)) * 180.0 / PI;
  } else {
    return atan2(-x, sqrt(y * y + z * z)) * 180.0 / PI;
  }
}

void setLeds(bool blue, bool green, bool red) {
  digitalWrite(BLUE_LED,  blue  ? HIGH : LOW);
  digitalWrite(GREEN_LED, green ? HIGH : LOW);
  digitalWrite(RED_LED,   red   ? HIGH : LOW);
}

// Light each LED in turn so the wiring can be checked by eye.
void ledSelfTest() {
  Serial.println(F("LED test: blue (pin 9)"));
  setLeds(true, false, false);
  delay(500);
  Serial.println(F("LED test: green (pin 8)"));
  setLeds(false, true, false);
  delay(500);
  Serial.println(F("LED test: red (pin 10)"));
  setLeds(false, false, true);
  delay(500);
  setLeds(false, false, false);
}

// Quick flash of all three LEDs to confirm a saved reading.
void flashConfirm() {
  for (int i = 0; i < 2; i++) {
    setLeds(true, true, true);
    delay(100);
    setLeds(false, false, false);
    delay(100);
  }
}

// Blink red forever so the problem is obvious without a computer,
// repeating the hint on the Serial Monitor every few seconds.
void haltWithError() {
  unsigned long lastHint = millis();
  while (true) {
    setLeds(false, false, true);
    delay(250);
    setLeds(false, false, false);
    delay(250);
    if (millis() - lastHint >= 3000) {
      lastHint = millis();
      Serial.println(F("Stopped: sensor problem. Check wiring, then press RESET."));
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("TiltLeveler starting..."));

  pinMode(BUTTON_PIN, INPUT);   // external pull-down resistor
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  setLeds(false, false, false);
  ledSelfTest();

  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  // Without a timeout, a bad I2C connection can freeze the Arduino forever.
  Wire.setWireTimeout(3000, true);
#endif

  if (!findMpu()) {
    Serial.println(F("MPU-6050 not found at 0x68 or 0x69!"));
    scanI2C();
    haltWithError();
  }
  Serial.print(F("MPU-6050 found at 0x"));
  Serial.print(mpuAddr, HEX);
  Serial.print(F(", WHO_AM_I = 0x"));
  Serial.println(readRegister(REG_WHO_AM_I), HEX);

  writeRegister(REG_PWR_MGMT, 0x00);   // wake the MPU-6050 (it boots asleep)
  writeRegister(REG_ACCEL_CFG, 0x00);  // +/-2g range = best resolution
  writeRegister(REG_CONFIG, 0x05);     // built-in low-pass filter ~10 Hz to cut vibration noise
  delay(100);

  float first = readTiltAngle();
  if (isnan(first)) {
    Serial.println(F("MPU-6050 found but could not read accelerometer data."));
    haltWithError();
  }
  filteredAngle = first;

  Serial.println(F("Ready. Press the button to save the current tilt."));
}

void loop() {
  // ---- Read and smooth the tilt angle ----
  float angle = readTiltAngle();
  if (!isnan(angle)) {
    filteredAngle = ALPHA * angle + (1.0 - ALPHA) * filteredAngle;
  }

  // ---- Debounced button: save reading on the press (LOW -> HIGH) ----
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > DEBOUNCE_MS && reading != buttonState) {
    buttonState = reading;
    if (buttonState == HIGH) {
      savedAngle = filteredAngle;
      haveSaved = true;
      Serial.print(F("Saved reference angle: "));
      Serial.print(savedAngle, 2);
      Serial.println(F(" deg"));
      flashConfirm();
    }
  }
  lastButtonReading = reading;

  // ---- Update LEDs ----
  float diff = 0.0;
  if (haveSaved) {
    diff = fabs(filteredAngle - savedAngle);
    if (diff <= BLUE_LIMIT) {
      setLeds(true, false, false);
    } else if (diff <= GREEN_LIMIT) {
      setLeds(false, true, false);
    } else {
      setLeds(false, false, true);
    }
  } else {
    setLeds(false, false, false);
  }

  // ---- Print status a few times per second ----
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    Serial.print(F("Angle: "));
    Serial.print(filteredAngle, 2);
    if (haveSaved) {
      Serial.print(F("  Saved: "));
      Serial.print(savedAngle, 2);
      Serial.print(F("  Diff: "));
      Serial.print(diff, 2);
    }
    Serial.println();
  }
}
