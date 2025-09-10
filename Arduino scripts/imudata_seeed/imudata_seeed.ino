#include "LSM6DS3.h"
#include "Wire.h"

// IMU setup
LSM6DS3 myIMU(I2C_MODE, 0x6A);

// RAM buffer
const int MAX_SAMPLES = 500;
float accelX[MAX_SAMPLES], accelY[MAX_SAMPLES], accelZ[MAX_SAMPLES];
int sampleCount = 0;
bool logging = false;

void setup() {
  // Start USB serial
  Serial.begin(115200);
  while (!Serial) { delay(10); }  // wait for USB

  if (myIMU.begin() != 0) {
    Serial.println("IMU not detected!");
    while (1);
  }
  Serial.println("IMU ready. Type 'r' to start recording.");
}

void loop() {
  // Check for command
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      Serial.println("Recording...");
      sampleCount = 0;
      logging = true;
    }
    if (c == 'p') {
      Serial.println("Printing data...");
      for (int i = 0; i < sampleCount; i++) {
        Serial.print(accelX[i]); Serial.print(",");
        Serial.print(accelY[i]); Serial.print(",");
        Serial.println(accelZ[i]);
      }
      Serial.println("Done!");
    }
  }

  // Record samples
  if (logging && sampleCount < MAX_SAMPLES) {
    accelX[sampleCount] = myIMU.readFloatAccelX();
    accelY[sampleCount] = myIMU.readFloatAccelY();
    accelZ[sampleCount] = myIMU.readFloatAccelZ();
    sampleCount++;
    delay(10); // ~100 Hz logging
  }
  else if (logging && sampleCount >= MAX_SAMPLES) {
    logging = false;
    Serial.println("Buffer full, type 'p' to print.");
  }
}
