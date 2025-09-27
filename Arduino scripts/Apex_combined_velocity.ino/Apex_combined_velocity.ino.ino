#include <Wire.h>
#include <LSM6DS3.h>

// ==== IMU ====
LSM6DS3 imu(I2C_MODE, 0x6A);   

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
const float fc_vel = 0.9;
float alpha_vel;
float velo_filt_prev = 0;
float velo_prev = 0;

// Apex detection
float last_apex_time = -1000.0;
const float min_velocity = 0.5;  // m/s
const float min_dt = 1.0;        // s
const float t_min = 0.0;         // start detection
const float t_max = 60.0;        // stop detection

float velocity = 0;
float velocity_filt = 0;

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();

  // Initialize IMU
  if (imu.begin() != 0) {
    Serial.println("IMU initialization failed!");
    while (1);
  } else {
    Serial.println("IMU initialized!");
  }

  // Compute filter coefficients (1st-order approximations)
  alpha_acc = dt / (dt + 1.0 / (2.0 * 3.14159 * fc_acc));
  alpha_vel = 1.0 / (1.0 + 1.0 / (2.0 * 3.14159 * fc_vel * dt));
}

void loop() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  // Run loop at fs
  if (now - lastMicros >= dt * 1e6) {
    lastMicros = now;

    // --- Read accelerometer ---
    float accX = imu.readFloatAccelX() * g;
    float accY = imu.readFloatAccelY() * g;
    float accZ = imu.readFloatAccelZ() * g;

    float current_time = millis() / 1000.0;

    // --- Norme Acc - g ---
    float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ) - g;

    // --- Low-pass filter accel ---
    float acc_filt = alpha_acc * acc_norm + (1 - alpha_acc) * acc_filt_prev;
    acc_filt_prev = acc_filt;

    // --- Integrate Acc -> Velocity ---
    velocity += acc_filt * dt;

    // --- High-pass filter velocity ---
    velocity_filt = alpha_vel * (velo_filt_prev + velocity - velo_prev);
    velo_prev = velocity;
    velo_filt_prev = velocity_filt;

    // --- Apex detection (zero-crossing positive→negative) ---
    static float velocity_prev_filt = 0;
    if ((velocity_prev_filt > 0) && (velocity_filt <= 0)) {
      float apex_time = current_time - dt / 2; // interpolation approx.

      if ((velocity_prev_filt > min_velocity) && 
          (apex_time - last_apex_time > min_dt) &&
          (apex_time >= t_min) && (apex_time <= t_max)) {
        
        last_apex_time = apex_time;
        Serial.print("Apex valide détecté à t = ");
        Serial.println(apex_time, 3);

        // Buzzer
        tone(buzzerPin, 1000); // 1 kHz tone
        delay(1000);             // play for 500 ms
        noTone(buzzerPin);      // stop the tone
      }
    }
    velocity_prev_filt = velocity_filt;

    // --- Debug ---
    Serial.print("t=");
    Serial.print(current_time, 2);
    Serial.print("  Velo_filt=");
    Serial.println(velocity_filt, 3);
  }
}
