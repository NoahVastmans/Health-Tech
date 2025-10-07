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

// ==== Prediction handling ====
float v0_release = 0.0;
float t_pred_apex = 0.0;
float t_pred_absolute = 0.0;
bool apexPredScheduled = false;
bool apexPredSounded = false;

// ==== Sound parameters ====
const int freq_pred = 1200;  // predicted apex tone (high)
const int freq_real = 800;   // measured apex tone (low)

// ==== Filters ====
const float fc_acc = 50.0;
float alpha_acc;
float acc_filt_prev = 0;

const float fc_vel = 5.0;
float alpha_vel;
float velo_filt_prev = 0;
float velo_prev = 0;

// ==== Apex detection ====
float last_apex_time = -1000.0;
const float min_velocity = 0.02;  // m/s
const float min_dt = 0.3;         // seconds between apices

float velocity = 0;
float velocity_filt = 0;

// ==== Buzzer ====
const float filter_delay = 0.12; // seconds
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 500; // milliseconds
bool buzzerStarted = false;

// ==== Free-fall detection ====
bool inFreeFall = false;
bool inFreeFallPrev = false;
float freeFallStartTime = 0.0;
const float FREEFALL_DELAY_COMP = 0.060; // seconds
const float LAND_IMPACT_THRESH = 1.3 * g; // detect landing

// ==== Setup ====
void setup() {
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(115200);
  Wire.begin();

  if (imu.begin() != 0) {
    Serial.println("IMU error!");
  } else {
    Serial.println("IMU OK!");
  }

  configFreeFall();

  // Compute filter coefficients
  alpha_acc = dt / (dt + 1.0 / (2.0 * 3.14159 * fc_acc));
  alpha_vel = 1.0 / (1.0 + 1.0 / (2.0 * 3.14159 * fc_vel * dt));
}

// ==== Configure Free-Fall ====
int configFreeFall() {
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  error += imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x70); // threshold/duration
  error += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81); // enable detection

  return error;
}

// ==== Loop ====
void loop() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  if (now - lastMicros >= dt * 1e6) {
    lastMicros = now;
    float current_time = millis() / 1000.0;

    // --- Read accelerometer ---
    float accX = imu.readFloatAccelX() * g;
    float accY = imu.readFloatAccelY() * g;
    float accZ = imu.readFloatAccelZ() * g;
    float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ) - g;

    // --- Low-pass filter accel ---
    float acc_filt = alpha_acc * acc_norm + (1 - alpha_acc) * acc_filt_prev;
    acc_filt_prev = acc_filt;

    // --- High-pass filter velocity ---
    velocity_filt = alpha_vel * (velo_filt_prev + acc_filt * dt);

    // --- Read free-fall flag from IMU ---
    uint8_t src;
    imu.readRegister(&src, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
    bool inFreeFall = src & 0x20; // bit 5

    // --- Free-fall detection and apex prediction ---
    if (inFreeFall && !inFreeFallPrev) {
      freeFallStartTime = current_time - FREEFALL_DELAY_COMP;

      // Capture velocity at release (upward positive)
      v0_release = velocity_filt;
      if (v0_release < 0) v0_release = 0;

      // Predict apex time (no drag)
      t_pred_apex = v0_release / g;           // time until apex
      t_pred_absolute = current_time + t_pred_apex;
      apexPredScheduled = true;
      apexPredSounded = false;

      Serial.print("🪂 Free fall detected! v0=");
      Serial.print(v0_release, 2);
      Serial.print(" m/s → predicted apex in ");
      Serial.print(t_pred_apex, 3);
      Serial.println(" s.");
    }

    // --- Predicted apex buzzer ---
    if (apexPredScheduled && !apexPredSounded) {
      if (current_time >= t_pred_absolute) {
        tone(buzzerPin, freq_pred);
        delay(150);
        noTone(buzzerPin);
        apexPredSounded = true;
        Serial.println("🔮 Predicted apex reached (theoretical).");
      }
    }

    // --- Apex detection (measured: velocity crosses + to -) ---
    if (inFreeFall) {
      if ((velo_filt_prev > 0) && (velocity_filt <= 0)) {
        float apex_time = current_time - dt / 2;
        float apex_since_freefall = apex_time - freeFallStartTime;

        if ((apex_time - last_apex_time > min_dt)) {
          last_apex_time = apex_time;
          Serial.print("🎯 Apex detected at t = ");
          Serial.print(apex_since_freefall, 3);
          Serial.println(" s since free-fall start.");

          // Play different tone for measured apex
          tone(buzzerPin, freq_real);
          delay(150);
          noTone(buzzerPin);

          // Log difference between predicted and actual apex
          float error = apex_time - t_pred_absolute;
          Serial.print("⏱ Prediction error: ");
          Serial.print(error * 1000, 1);
          Serial.println(" ms.");
        }
      }
    }

    // --- Landing detection and reset for next throw ---
    if (!inFreeFall && inFreeFallPrev) {
      // Potential landing event
      if (acc_norm > LAND_IMPACT_THRESH) {
        Serial.println("💥 Landing detected — system reset for next throw.");

        // Reset all state variables
        apexPredScheduled = false;
        apexPredSounded   = false;
        last_apex_time    = -1000.0;
        buzzerActive      = false;
        buzzerStarted     = false;
        v0_release        = 0.0;
        t_pred_apex       = 0.0;
        t_pred_absolute   = 0.0;
      }
    }

    // Update previous values
    inFreeFallPrev = inFreeFall;
    velo_filt_prev = velocity_filt;

    // --- Buzzer control (for scheduled events, if used) ---
    if (buzzerActive) {
      unsigned long now_ms = millis();

      if (!buzzerStarted && now_ms >= buzzerStartTime) {
        tone(buzzerPin, 1000);
        buzzerStarted = true;
      }

      if (buzzerStarted && now_ms >= buzzerStartTime + buzzerDuration) {
        noTone(buzzerPin);
        buzzerActive = false;
        buzzerStarted = false;
      }
    }
  }
}
