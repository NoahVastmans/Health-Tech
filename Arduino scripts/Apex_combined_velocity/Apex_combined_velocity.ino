#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// Create BMI270 object
BMI270 imu;

// ==== Parameters ====
const int buzzerPin = 5;
const float g = 9.81;
const float fs = 52.0;        // Sampling frequency (Hz)
const float dt = 1.0 / fs;    

// Low-pass filter (acceleration)
const float fc_acc = 0.5;
float alpha_acc;
float acc_filt_prev = 0;

// High-pass filter (velocity)
const float fc_vel = 0.4;
float alpha_vel;
float velo_filt_prev = 0;
float velo_prev = 0;

// Apex detection
float last_apex_time = -1000.0;
const float min_velocity = 80.0;  // Minimum velocity for valid apex (m/s)
const float min_dt = 3.0;         // Minimum time between apexes (s)
const float t_min = 3.0;          // Minimum time to start detecting apex (s)
const float t_max = 30.0;         // Maximum time for apex detection (s)

float velocity = 0;
float velocity_filt = 0;

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);
  Wire.begin();

  // Initialize IMU over I2C
  if (imu.beginI2C() != 0) { // 0 = success
    Serial.println("IMU initialization failed!");
    while (1);
  }

  // Alpha coefficients for filters
  alpha_acc = dt / (dt + 1.0 / (2 * 3.14159 * fc_acc));
  alpha_vel = 1.0 / (1.0 + 1.0 / (2 * 3.14159 * fc_vel * dt));
}


void loop() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();

  // Ensure loop runs at sampling frequency
  if (now - lastMicros >= dt * 1e6) {
    lastMicros = now;

    // --- Read accelerometer ---
    imu.getSensorData(); // Update imu.data
    float accX = imu.data.accelX * g;
    float accY = imu.data.accelY * g;
    float accZ = imu.data.accelZ * g;

    float current_time = millis() / 1000.0;

    // --- Compute acceleration magnitude minus gravity ---
    float acc_norm = sqrt(accX*accX + accY*accY + accZ*accZ) - g;

    // --- Low-pass filter ---
    float acc_filt = alpha_acc*acc_norm + (1 - alpha_acc)*acc_filt_prev;
    acc_filt_prev = acc_filt;

    // --- Integrate to get velocity ---
    velocity += acc_filt * dt;

    // --- High-pass filter velocity ---
    velocity_filt = alpha_vel*(velo_filt_prev + velocity - velo_prev);
    velo_prev = velocity;
    velo_filt_prev = velocity_filt;

    // --- Apex detection ---
    static float velocity_prev = 0;
    if ((velocity_prev > 0) && (velocity_filt <= 0)) {
      float apex_time = current_time - dt/2;
      if ((velocity_prev > min_velocity) && (apex_time - last_apex_time > min_dt)
          && (apex_time >= t_min) && (apex_time <= t_max)) {
        last_apex_time = apex_time;
        Serial.print("Apex detected at t = ");
        Serial.println(apex_time);

        // Trigger buzzer
        digitalWrite(buzzerPin, HIGH);
        delay(100);
        digitalWrite(buzzerPin, LOW);
      }
    }
    velocity_prev = velocity_filt;
  }
}
