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
const float min_velocity = 0.1;  // m/s
const float min_dt = 1.0;        // s
const float t_min = 0.0;         // start detection
const float t_max = 60.0;        // stop detection

float velocity = 0;
float velocity_filt = 0;

const float filter_delay = 0.12; // seconds
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
const unsigned long buzzerDuration = 500; // milliseconds
bool buzzerStarted = false;


void setup() {
  // Initialize Bluetooth
  delay(200);
  BLE.begin();
  BLE.setLocalName("XIAO_IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  BLE.addService(imuService);
  BLE.advertise();

  pinMode(buzzerPin, OUTPUT);
  Wire.begin();

  // Initialize IMU
  imu.begin();

  // Compute filter coefficients (1st-order approximations)
  alpha_acc = dt / (dt + 1.0 / (2.0 * 3.14159 * fc_acc));
  alpha_vel = 1.0 / (1.0 + 1.0 / (2.0 * 3.14159 * fc_vel * dt));
}

void loop() {
  BLE.poll();

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

    // --- Norm Acc - g ---
    float acc_norm = sqrt(accX * accX + accY * accY + accZ * accZ) - g;

    // --- Low-pass filter accel ---
    float acc_filt = alpha_acc * acc_norm + (1 - alpha_acc) * acc_filt_prev;
    acc_filt_prev = acc_filt;

    // --- Integrate Acc -> Velocity ---
    //velocity += acc_filt * dt;

    // --- High-pass filter velocity ---
    //velocity_filt = alpha_vel * (velo_filt_prev + velocity - velo_prev);
    velocity_filt = alpha_vel * (velo_filt_prev + acc_filt * dt);
    velo_prev = velocity;
    

    // --- Apex detection (zero-crossing positive→negative) ---
    
    if ((velo_filt_prev > 0) && (velocity_filt <= 0)) {
      float apex_time = current_time - dt / 2; // interpolation approx.

      if ((velo_filt_prev - velocity_filt > min_velocity) && 
          (apex_time - last_apex_time > min_dt)) {
        
        last_apex_time = apex_time;

        // Buzzer
        
        buzzerStartTime = millis() + filter_delay*1000;
        buzzerActive = true;
      }
    }
    velo_filt_prev = velocity_filt;

    //--- Debug ---
    char buf[32];  // plenty for 3 floats
    snprintf(buf, sizeof(buf), "%.5f", velocity_filt);
    imuDataChar.setValue(buf);

    if (buzzerActive) {
      unsigned long now_ms = millis();

      // Start buzzer after scheduled delay
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
