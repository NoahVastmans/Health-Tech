#include "ImuFusion.h"
#include <Arduino.h>

// Sensor data variables (in m/s^2 for accel, deg/s for gyro)
float ax, ay, az, gx, gy, gz;

// ImuFusion object
Fusion fusion;

// Time tracking
unsigned long last_update_time = 0;
float delta_time = 0;

// State machine
enum State {
    ON_GROUND,
    IN_FLIGHT,
    LANDING
};
State current_state = ON_GROUND;

// Thresholds (tune these values)
const float LAUNCH_ACCEL_THRESHOLD = 10.0; //from takes around 10 m/s^2
const float HIT_ACCEL_THRESHOLD = 15.0; // hit or landing at least 15 m/s^2
const float APEX_VELOCITY_THRESHOLD = 0.5;

// Velocity tracking
float vz = 0.0;
float vz_previous = 0.0;

void setup() {
    Serial.begin(115200);
    // Initialize your IMU sensor here
    // Example: MPU6050_IMU.begin();
    last_update_time = micros();
}

void loop() {
    // Read raw sensor data
    // Example: MPU6050_IMU.readSensorData(&gx, &gy, &gz, &ax, &ay, &az);

    // Calculate time step
    unsigned long current_time = micros();
    delta_time = (current_time - last_update_time) / 1000000.0f;
    last_update_time = current_time;

    // Update the fusion filter
    fusion.updateIMU(gx, gy, gz, ax, ay, az, delta_time);

    // Get gravity-removed linear acceleration
    float linear_accel_z = fusion.linearAccelZ;

    // Update vertical velocity
    vz_previous = vz;
    vz += linear_accel_z * delta_time;

    // State machine logic
    switch (current_state) {
        case ON_GROUND:
            if (abs(linear_accel_z) > LAUNCH_ACCEL_THRESHOLD) {
                current_state = IN_FLIGHT;
                vz = 0.0; // Reset velocity at launch
                Serial.println("LAUNCH DETECTED");
            }
            break;

        case IN_FLIGHT:
            // Apex detection
            if (vz < 0.0 && vz_previous >= 0.0) {
                Serial.println("APEX REACHED");
            }
            
            // Landing detection
            if (abs(linear_accel_z) > LANDING_ACCEL_THRESHOLD) {
                current_state = LANDING;
                Serial.println("LANDING DETECTED");
            }
            break;

        case LANDING:
            // Optionally, add a delay or return to ON_GROUND state after a short period
            current_state = ON_GROUND;
            break;
    }
}