#include <Wire.h>
#include <LSM6DS3.h>
#include <ArduinoBLE.h>

// ==== IMU ====
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==== Bluetooth ====
BLEService imuService("180A");
BLECharacteristic imuDataChar("2A57", BLERead | BLENotify, 32);

// ==== Parameters ====
const int buzzerPin = 5;
const float g = 9.81;
const float fs = 52.0;        // Sampling frequency (Hz)
const float dt = 1.0 / fs;

// Low-pass filter (acceleration)
const float fc_acc = 5.0;
float alpha_acc;
float acc_filt_prev = 0;

// High-pass filter (velocity)
const float fc_vel = 50.0;
float alpha_vel;
float velo_filt_prev = 0;
float velo_prev = 0;

// Apex detection
float last_apex_time = -1000.0;
const float min_dt = 0.3;         // seconds between apices

float velocity = 0;
float velocity_filt = 0;

// Buzzer
const float filter_delay = 0.034; // seconds
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 500; // milliseconds
bool buzzerStarted = false;

// Free-fall detection state
bool inFreeFall = false;
bool inFreeFallPrev = false;
float freeFallStartTime = 0.0;
float freeFallEndTime = 0.0;
float apex_since_freefall = 0.0;
const float FREEFALL_DELAY_COMP = 0.0; // seconds


// ==== Setup ====
/*
  Before running this sketch:
  - Replace the LSM6DS3.cpp file in your Arduino library folder
    with the modified file from this project folder.
    This change ensures correct free-fall register behavior.

  This program detects free-fall and apex events using the LSM6DS3 IMU,
  sends the event data via BLE, and triggers a buzzer after the apex.
*/
void setup() {
  // --- Initialize Bluetooth ---
  delay(200);
  BLE.begin();
  BLE.setLocalName("XIAO_IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  BLE.addService(imuService);
  BLE.advertise();

  // --- Hardware setup ---
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();
  imu.begin();
  configFreeFall();

  // --- Compute filter coefficients ---
  alpha_acc = dt / (dt + 1.0 / (2.0 * 3.14159 * fc_acc));
  alpha_vel = 1.0 / (1.0 + 1.0 / (2.0 * 3.14159 * fc_vel * dt));
}


// ==== Configure Free-Fall ====
/*
  Configures the LSM6DS3 sensor for free-fall detection.

  The modified LSM6DS3.cpp ensures correct operation of the
  free-fall interrupt bit in the WAKE_UP_SRC register.
*/
int configFreeFall() {
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  error += imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33); // threshold/duration
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81); // enable detection

  return error;
}


// ==== Main Loop ====
void loop() {
  BLE.poll();

  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  // Maintain constant sample rate
  if (now - lastMicros >= dt * 1e6) {
    lastMicros = now;
    float current_time = millis() / 1000.0;

    // --- Read accelerometer ---
    float accX = imu.readFloatAccelX() * g;
    float accY = imu.readFloatAccelY() * g;
    float accZ = imu.readFloatAccelZ() * g;
    float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ) - g;

    // --- Low-pass filter acceleration ---
    float acc_filt = alpha_acc * acc_norm + (1 - alpha_acc) * acc_filt_prev;
    acc_filt_prev = acc_filt;

    // --- High-pass filter velocity (to remove drift) ---
    velocity_filt = alpha_vel * (velo_filt_prev + acc_filt * dt);

    // --- Read free-fall flag ---
    uint8_t src;
    imu.readRegister(&src, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
    bool inFreeFall = src & 0x20; // bit 5: free-fall flag

    // --- Detect start of free fall ---
    if (inFreeFall && !inFreeFallPrev) {
      freeFallStartTime = current_time - FREEFALL_DELAY_COMP;
    }

    // --- Apex detection (zero-crossing of velocity) ---
    if (inFreeFall) {
      if ((velo_filt_prev > 0) && (velocity_filt <= 0)) {
        float apex_time = current_time - dt / 2;
        apex_since_freefall = apex_time - freeFallStartTime + filter_delay;

        // Avoid duplicates if too close in time
        if ((apex_time - last_apex_time > min_dt)) {
          last_apex_time = apex_time;

          // Schedule buzzer activation
          buzzerStartTime = millis() + filter_delay * 1000;
          buzzerActive = true;
        }
      }
    }

    // --- Detect end of free fall ---
    if (!inFreeFall && inFreeFallPrev) {
      freeFallEndTime = current_time;

      // Send free-fall data via BLE
      char buf[32];
      snprintf(buf, sizeof(buf), "%.3f, %.3f, %.3f",
               freeFallStartTime, apex_since_freefall, freeFallEndTime);
      imuDataChar.setValue(buf);
    }

    // --- Update states ---
    inFreeFallPrev = inFreeFall;
    velo_filt_prev = velocity_filt;

    // --- Buzzer control ---
    if (buzzerActive) {
      unsigned long now_ms = millis();

      // Activate buzzer
      if (!buzzerStarted && now_ms >= buzzerStartTime) {
        tone(buzzerPin, 1000);
        buzzerStarted = true;
      }

      // Stop buzzer after duration
      if (buzzerStarted && now_ms >= buzzerStartTime + buzzerDuration) {
        noTone(buzzerPin);
        buzzerActive = false;
        buzzerStarted = false;
      }
    }
  }
}
