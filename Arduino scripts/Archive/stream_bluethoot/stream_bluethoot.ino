#include <ArduinoBLE.h>
#include <LSM6DS3.h>

LSM6DS3 imu(I2C_MODE, 0x6A); // I²C mode, default address 0x6A


BLEService imuService("180A");
BLECharacteristic imuDataChar("2A57", BLERead | BLENotify, 32);


void setup() {

  delay(200);
  BLE.begin();
  BLE.setLocalName("XIAO_IMU");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuDataChar);
  BLE.addService(imuService);
  BLE.advertise();
  Serial.println("BLE ready");
  imu.begin();


}

// Helper to round float to 3 decimals and send via BLE
void sendAccelData(float ax, float ay, float az) {
  char buf[32];  // plenty for 3 floats
  snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f", ax, ay, az);
  imuDataChar.setValue(buf);
}

void sendGyroData(float gx, float gy, float gz) {
  char buf[32];  // adjust later if you change gyro format
  snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f", gx, gy, gz);
  imuDataChar.setValue(buf);
}

void loop() {
  BLE.poll();  // handle BLE events

  // read IMU
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  // send accelerometer
  sendAccelData(ax, ay, az);
  delay(10); // small gap to prevent BLE overflow

  // send gyroscope
  sendGyroData(gx, gy, gz);
  delay(50); // adjust sample rate (~10–20 Hz total)
}


