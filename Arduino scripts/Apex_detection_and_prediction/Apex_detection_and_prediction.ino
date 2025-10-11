#include <Wire.h>
#include <LSM6DS3.h>
#include <ArduinoBLE.h>

//
// ============================================================
//  SmartServe Ball: Apex Detection, Apex Estimation & BLE Output
// ============================================================
//  Features:
//   • Detects free fall and catch events from accelerometer data
//   • Estimates flight apex using filtered velocity zero-crossing
//   • Predicts apex time based on initial velocity
//   • Provides audible buzzer feedback (sync with video)
//   • Transmits timing data via BLE for remote logging
//
//  Hardware:
//   • IMU: LSM6DS3 (I²C address 0x6A)
//   • BLE: ArduinoBLE library (e.g., XIAO BLE)
//   • Buzzer on pin 5
//
// ============================================================
//


// ==== IMU ====
LSM6DS3 imu(I2C_MODE, 0x6A);

// ==== Bluetooth ====
BLEService imuService("180A");                 // Generic IMU service UUID
BLECharacteristic imuDataChar("2A57", BLERead | BLENotify, 32);  // Data payload

// ==== Constants ====
const int buzzerPin = 5;
const float g = 9.81;                          // Gravity constant (m/s²)
const float fs = 52.0;                         // Sampling frequency (Hz)
const float dt = 1.0 / fs;                     // Sampling period (s)

// ------------------------------------------------------------
// FILTER CONFIGURATION
// ------------------------------------------------------------

// Low-pass filter for acceleration (removes high-frequency noise)
const float fc_acc = 5.0;                      // Cutoff frequency (Hz)
float alpha_acc;                               // Filter coefficient
float acc_filt_prev = 0;                       // Previous filtered acceleration
float acc_norm_prev = 0;                       // Previous raw norm

// High-pass filter for velocity (removes drift)
const float fc_vel = 50.0;                     // Cutoff frequency (Hz)
float alpha_vel;
float velo_filt_prev = 0;
float velo_prev = 0;

// ------------------------------------------------------------
// APEX DETECTION
// ------------------------------------------------------------
float last_apex_time = -1000.0;
const float min_velocity = 0.1;                // Threshold to avoid noise-triggered apex (m/s)
const float min_dt = 1.0;                      // Minimum time between apex detections (s)
float velocity = 0;
float velocity_filt = 0;

const float filter_delay = 0.12;               // Compensation delay from filtering (s)

// Buzzer control
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 500;      // Duration of beep (ms)
bool buzzerStarted = false;

// ------------------------------------------------------------
// FREE FALL DETECTION
// ------------------------------------------------------------
const float FREEFALL_THRESH = -7.0;            // Acceleration threshold (≈0.1 g)
const float CATCH_THRESH = 1.0;                // Threshold indicating catch or impact
const unsigned long FREEFALL_MIN = 0.040;      // Must remain below threshold this long (s)

bool inFreeFall = false;                       // Indicates active flight phase
float freeFallStart = 0.0;
float freeFallEnd = 0.0;
float apex_since_freefall = 0.0;

// ------------------------------------------------------------
// APEX PREDICTION
// ------------------------------------------------------------
float v0_release = 0.0;                        // Estimated release velocity (m/s)
float t_pred_apex = 0.0;                       // Predicted time until apex (s)
float t_pred_absolute = 0.0;                   // Absolute predicted apex time (s)


// ============================================================
//                      SETUP
// ============================================================
/*
  Before running this sketch:
  - Replace the LSM6DS3.cpp file in your Arduino library folder
    with the modified file from this project folder.
    This change ensures correct free-fall register behavior.
*/
void setup() {
  // ---- Initialize Bluetooth ----
  delay(200);
  BLE.begin();
  BLE.setLocalName("XIAO_IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  BLE.addService(imuService);
  BLE.advertise();

  // ---- Initialize peripherals ----
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();
  imu.begin();

  // ---- Compute filter coefficients (1st-order) ----
  alpha_acc = dt / (dt + 1.0 / (2.0 * 3.14159 * fc_acc));  // LPF smoothing
  alpha_vel = 1.0 / (1.0 + 1.0 / (2.0 * 3.14159 * fc_vel * dt));  // HPF factor
}


// ============================================================
//                      MAIN LOOP
// ============================================================

void loop() {
  BLE.poll();   // Keep BLE connection alive

  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  // Run loop at fixed sampling frequency
  if (now - lastMicros >= dt * 1e6) {
    lastMicros = now;

    // --- Read raw accelerometer data (m/s²) ---
    float accX = imu.readFloatAccelX() * g;
    float accY = imu.readFloatAccelY() * g;
    float accZ = imu.readFloatAccelZ() * g;

    float current_time = millis() / 1000.0;

    // --- Compute acceleration norm minus gravity ---
    float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ) - g;

    // --- Apply low-pass filter to acceleration ---
    float acc_filt = alpha_acc * acc_norm + (1 - alpha_acc) * acc_filt_prev;

    // --- Integrate to get high-pass filtered velocity ---
    velocity_filt = alpha_vel * (velo_filt_prev + acc_filt * dt);


    // ----------------------------------------------------
    //               FREE FALL START DETECTION
    // ----------------------------------------------------
    if (!inFreeFall && acc_norm < FREEFALL_THRESH) {
      // Potential start of free fall
      if (freeFallStart == 0) {
        freeFallStart = current_time;

        // Approximate initial velocity at release
        v0_release = velo_filt_prev + acc_norm_prev * dt;

        // Predict apex time (neglecting drag)
        t_pred_apex = v0_release / g;
        t_pred_absolute = current_time + t_pred_apex;
      }
      // Confirm free fall after remaining below threshold long enough
      else if (current_time - freeFallStart > FREEFALL_MIN) {
        inFreeFall = true;

        // Short tone to mark start of flight (sync signal)
        tone(buzzerPin, 500, 100);
      }
    }

    // ----------------------------------------------------
    //               FREE FALL END DETECTION
    // ----------------------------------------------------
    else if (inFreeFall && acc_norm > CATCH_THRESH) {
      // Impact or catch detected
      inFreeFall = false;
      freeFallEnd = current_time;

      // Send timing data via BLE
      char buf[32];
      snprintf(buf, sizeof(buf), "%.3f, %.3f, %.3f",
               t_pred_apex, apex_since_freefall, freeFallEnd - freeFallStart);
      imuDataChar.setValue(buf);

      // Reset variables for next throw
      freeFallStart = 0;
      v0_release = 0.0;
      t_pred_apex = 0.0;
      t_pred_absolute = 0.0;
    }

    // ----------------------------------------------------
    //               CANCEL FALSE FREE FALL
    // ----------------------------------------------------
    else if (!inFreeFall && acc_filt >= FREEFALL_THRESH) {
      // Threshold not held long enough — discard
      freeFallStart = 0;
      v0_release = 0;
      t_pred_apex = 0.0;
      t_pred_absolute = 0;
    }


    // ----------------------------------------------------
    //               APEX DETECTION (ZERO-CROSSING)
    // ----------------------------------------------------
    if (inFreeFall) {
      // Detect sign change: upward (positive) → downward (negative)
      if ((velo_filt_prev > 0) && (velocity_filt <= 0)) {
        float apex_time = current_time - dt / 2; // Midpoint between samples

        if ((apex_time - last_apex_time > min_dt)) {
          last_apex_time = apex_time;
          apex_since_freefall = apex_time - freeFallStart + filter_delay;

          // Schedule buzzer for apex mark (after filter delay)
          buzzerStartTime = millis() + filter_delay * 1000;
          buzzerActive = true;

          // Compute error between predicted and actual apex
          float error = apex_time - t_pred_absolute;
          // (could be sent or logged if desired)
        }
      }
    }


    // ----------------------------------------------------
    //               UPDATE FILTER STATES
    // ----------------------------------------------------
    velo_filt_prev = velocity_filt;
    velo_prev = velocity;
    acc_norm_prev = acc_norm;
    acc_filt_prev = acc_filt;


    // ----------------------------------------------------
    //               BUZZER CONTROL
    // ----------------------------------------------------
    if (buzzerActive) {
      unsigned long now_ms = millis();

      // Start buzzer after scheduled delay
      if (!buzzerStarted && now_ms >= buzzerStartTime) {
        tone(buzzerPin, 1000);  // Beep at 1 kHz
        buzzerStarted = true;
      }

      // Stop buzzer after fixed duration
      if (buzzerStarted && now_ms >= buzzerStartTime + buzzerDuration) {
        noTone(buzzerPin);
        buzzerActive = false;
        buzzerStarted = false;
      }
    }
  }
}
