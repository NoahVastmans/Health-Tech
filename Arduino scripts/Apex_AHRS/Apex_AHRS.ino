#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

#undef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329576923690768489

LSM6DS3 IMU(I2C_MODE, 0x6A);
Madgwick filter;

const int soundTriggerPin = 5; // Buzzer pin

enum FlightStage { // Flight stages follow sequential order of events
    PRE_FLIGHT,
    THROWING,
    IN_AIR,
    APEX,
    CATCH
};

FlightStage currentStage = PRE_FLIGHT;

// IMU sensor data
float ax, ay, az; // Raw acceleration (m/s²)
float gx, gy, gz; // Raw gyroscope (rad/s)

// Velocity tracking
float verticalVelocity = 0.0;    // Current vertical velocity (m/s)
float prevVerticalVelocity = 0.0; // Previous vertical velocity (m/s)

// Timing
float timeStep = 0.0;
unsigned long previousTime = 0;

// Filtering
float filteredAz = 0.0;
const float alpha = 0.5; // Low-pass filter coefficient for free fall detection

// Thresholds for flight stages (may need tuning after data collection)
const float THROWING_ACCEL_THRESHOLD = 10.0;  // m/s²
const float CATCH_ACCEL_THRESHOLD = 15.0;      // m/s²
const float GRAVITY_THRESHOLD = 2.0;           // m/s²

// State tracking
bool apexSoundTriggered = false;

int config_free_fall_detect(void) {//seed sensor documentation, uses a simple low pass filter, and works to quickly identify free falling (aka ball in the air)
    uint8_t error = 0;
    uint8_t dataToWrite = 0;

    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
    error += IMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
    error += IMU.writeRegister(LSM6DS3_ACC_GYRO_FREE_FALL, 0x33);
    error += IMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x10);

    return error;
}

void setup() {
    Serial.begin(115200);//updated noah code
    while (!Serial); // Wait for serial connection

    // Initialize IMU
    if (IMU.begin() != 0) {
        Serial.println("Failed to initialize IMU!");
        while(1);
    }
    Serial.println("IMU ready");

    // Configure free-fall detection
    if (config_free_fall_detect() != 0) {
        Serial.println("Failed to configure free-fall detection!");
        while (1);
    } else {
        Serial.println("IMU configured successfully!");
    }

    // Initialize Madgwick AHRS filter
    filter.begin(100.0); // 100 Hz update rate
    Serial.println("Madgwick filter initialized.");
    
    // Setup complete
    Serial.println("System ready. Stage: PRE-FLIGHT");
    previousTime = millis();
    
    // Configure buzzer pin
    pinMode(soundTriggerPin, OUTPUT);
}

void loop() {
    // 1. Read raw sensor data
    readSensors();
    
    // 2. Update orientation filter with IMU data
    updateFilter();
    
    // 3. Calculate time step for integration
    computeTimeStep();
    
    // 4. Get gravity-corrected vertical acceleration
    float verticalAccel = getVerticalAcceleration();
    
    // 5. Apply low-pass filter for free-fall detection
    filteredAz = alpha * verticalAccel + (1 - alpha) * filteredAz;

    // 6. Integrate acceleration to get velocity (centralized)
    updateVerticalVelocity(verticalAccel);

    // 7. Check hardware free-fall interrupt
    bool freeFallDetected = checkFreeFallInterrupt();
    
    // 8. Update flight stage based on sensor data
    handleFlightStage(filteredAz, freeFallDetected); // Use filtered value for stage detection
}

void readSensors() {
    ax = IMU.readFloatAccelX();
    ay = IMU.readFloatAccelY();
    az = IMU.readFloatAccelZ();

    gx = IMU.readFloatGyroX() * DEG_TO_RAD;
    gy = IMU.readFloatGyroY() * DEG_TO_RAD;
    gz = IMU.readFloatGyroZ() * DEG_TO_RAD;
}

void updateFilter() {
    filter.updateIMU(gx, gy, gz, ax, ay, az);
}

void computeTimeStep() {
    unsigned long currentTime = millis();
    timeStep = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
}

float getVerticalAcceleration() {
    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);

    // Rotate acceleration vector into global frame using quaternion rotation
    //float gAx = ax * (1 - 2 * (qy * qy + qz * qz)) + ay * (2 * (qx * qy - qw * qz)) + az * (2 * (qx * qz + qw * qy));
    //float gAy = ax * (2 * (qx * qy + qw * qz)) + ay * (1 - 2 * (qx * qx + qz * qz)) + az * (2 * (qy * qz - qw * qx));
    float gAz = ax * (2 * (qx * qz - qw * qy)) + ay * (2 * (qy * qz + qw * qx)) + az * (1 - 2 * (qx * qx + qy * qy));

    return gAz;
}

bool checkFreeFallInterrupt() {
    uint8_t readDataByte = 0;
    IMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);
    return (readDataByte & 0x20);
}

void updateVerticalVelocity(float verticalAccel) {
    // Only integrate velocity when we're actively tracking motion
    if (currentStage == THROWING || currentStage == IN_AIR || currentStage == APEX) {
        prevVerticalVelocity = verticalVelocity;
        float netAccel = verticalAccel - 9.81; // Net acceleration (removing gravity)
        verticalVelocity += netAccel * timeStep;
        
        // Debug output
        Serial.print("Stage: ");
        Serial.print(currentStage);
        Serial.print(" | Vel: ");
        Serial.print(verticalVelocity, 2);
        Serial.print(" m/s | NetAccel: ");
        Serial.print(netAccel, 2);
        Serial.print(" m/s² | dt: ");
        Serial.println(timeStep, 4);
    }
}

void handleFlightStage(float verticalAccel, bool freeFallDetected) {
    switch (currentStage) {
        case PRE_FLIGHT:
            if (abs(verticalAccel) > THROWING_ACCEL_THRESHOLD) {
                currentStage = THROWING;
                Serial.println("Stage: THROWING");
                verticalVelocity = 0.0; // Reset velocity at start of throw
                apexSoundTriggered = false; // Reset for new throw
            }
            break;

        case THROWING:
            if (freeFallDetected || abs(verticalAccel - 9.81) < GRAVITY_THRESHOLD) {
                currentStage = IN_AIR;
                Serial.println("Stage: IN-AIR");
                Serial.print("Release velocity: ");
                Serial.print(verticalVelocity);
                Serial.println(" m/s");
            }
            break;

        case IN_AIR:
            if (prevVerticalVelocity > 2.0 && verticalVelocity <= -2.0) {
                currentStage = APEX;
                Serial.println("Stage: APEX");
                Serial.print("Apex velocity: ");
                Serial.print(verticalVelocity);
                Serial.println(" m/s");
            }
            break;

        case APEX:
            if (abs(verticalAccel) > CATCH_ACCEL_THRESHOLD) {
                currentStage = CATCH;
                Serial.println("Stage: CATCH");
            } else if (!apexSoundTriggered) {
                // Trigger sound at apex (only once per flight)
                Serial.println("Triggering sound at APEX");
                tone(soundTriggerPin, 1000, 200); // 1 kHz tone for 200ms
                apexSoundTriggered = true;
            }
            break;
            
        case CATCH:
            Serial.println("Flight sequence complete. Resetting...");
            delay(2000);
            currentStage = PRE_FLIGHT;
            verticalVelocity = 0.0; // Reset for next throw
            apexSoundTriggered = false; // Reset for next flight
            break;
    }
}
