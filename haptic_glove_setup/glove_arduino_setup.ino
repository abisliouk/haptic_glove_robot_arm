// This script is used to read the flex sensors and the MPU6050 accelerometer
// and send the data to the computer via serial.

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const int pins[5] = {A0, A1, A2, A3, A4};

// This value must be calibrated manually (sensor min/max value)
const int minVals[5] = {200, 210, 220, 230, 240}; // Finger fully extended
const int maxVals[5] = {800, 810, 820, 830, 840}; // Finger fully folded value

const int tiltThreshold = 20;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  delay(1000);
}

void loop() {
  String output = "";
  bool anySignal = false;

  // === detect finger ===
  String fingerSignal = "";
  for (int i = 0; i < 5; i++) {
    int val = analogRead(pins[i]);

    // normalize: 0 ~ 100%
    int percent = map(val, minVals[i], maxVals[i], 0, 100);
    percent = constrain(percent, 0, 100);

    fingerSignal += String(i + 1) + ":" + String(percent) + ",";
    anySignal = true;
  }

  if (fingerSignal.length() > 0) {
    fingerSignal.remove(fingerSignal.length() - 1); //
    output += "FINGER:" + fingerSignal;
  }

  // === detect tilt ===
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float ax_f = (float)ax / 16384.0;
  float ay_f = (float)ay / 16384.0;
  float az_f = (float)az / 16384.0;

  static float pitch_filtered = 0;
  static float roll_filtered  = 0;

  float pitch_raw = atan2(ax_f, sqrt(ay_f * ay_f + az_f * az_f)) * 180.0 / PI;
  float roll_raw  = atan2(ay_f, sqrt(ax_f * ax_f + az_f * az_f)) * 180.0 / PI;

  pitch_filtered = pitch_raw * 0.3 + pitch_filtered * 0.7;
  roll_filtered  = roll_raw  * 0.3 + roll_filtered * 0.7;

  if (abs(pitch_filtered) > tiltThreshold || abs(roll_filtered) > tiltThreshold) {
    if (anySignal) output += "|";
    output += "TILT:";
    output += String((int)pitch_filtered) + "," + String((int)roll_filtered);
    anySignal = true;
  }

  // === final output ===
  if (anySignal) {
    Serial.println(output);
  }

  delay(50);
}
