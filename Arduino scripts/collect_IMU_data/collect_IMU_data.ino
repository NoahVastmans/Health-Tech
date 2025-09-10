#include <Wire.h>
#include <LSM6DS3.h>
#include "Seeed_FS.h"
#include <Seeed_SFUD.h>


LSM6DS3 myIMU(I2C_MODE, 0x6A); // I2C adres van de IMU
File dataFile;

const float accelerationThreshold = 1; // G-kracht drempel
const unsigned long loggingDuration = 7000; // loggingtijd in milliseconden (7 sec)

bool logging = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(3000);     // extra tijd om de monitor te openen
  Serial.println("Wacht op beweging...");


  // Start IMU
  int imuStatus = myIMU.begin();
  Serial.print("IMU init status: "); Serial.println(imuStatus);
  if (imuStatus != 0) {
    Serial.println("IMU niet gevonden");
    while (1);
  }

  // Start QSPI flash
  Serial.println("Start QSPI init...");
  bool flashStatus = SFUD.begin();
  Serial.print("QSPI status: "); Serial.println(flashStatus);

  if (!flashStatus) {
    Serial.println("QSPI Flash init mislukt");
    while (1);
  }

  if (SFUD.exists("imu_log.csv")) {
    Serial.println("Bestand gevonden. Inhoud:");
    File f = SFUD.open("imu_log.csv", FILE_READ);
    while (f.available()) {
      Serial.write(f.read());
    }
    f.close();
  } else {
    Serial.println("Geen logbestand gevonden.");
  }


  //Serial.println("Wacht op beweging...");
}

void loop() {
  float aX = myIMU.readFloatAccelX();
  float aY = myIMU.readFloatAccelY();
  float aZ = myIMU.readFloatAccelZ();
  float aMagnitude = sqrt(aX * aX + aY * aY + aZ * aZ);

  Serial.print("aMagnitude: ");
  Serial.println(aMagnitude, 3);

  if (!logging) {
    // Lees acceleratie
    float aX = myIMU.readFloatAccelX();
    float aY = myIMU.readFloatAccelY();
    float aZ = myIMU.readFloatAccelZ();
    Serial.print("aX: "); Serial.print(aX, 3);
    Serial.print(" aY: "); Serial.print(aY, 3);
    Serial.print(" aZ: "); Serial.println(aZ, 3);


    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);
    Serial.print("aMagnitude: ");
    Serial.println(aSum, 3);


    if (aSum >= accelerationThreshold) {
      Serial.println("Beweging gedetecteerd! Start logging...");
      startTime = millis();
      logging = true;

      // Maak nieuw bestand aan
      dataFile = SFUD.open("imu_log.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("aX,aY,aZ,gX,gY,gZ");
      }
    }
  }

  if (logging && millis() - startTime <= loggingDuration) {
    // Verzamel en log data
    float aX = myIMU.readFloatAccelX();
    float aY = myIMU.readFloatAccelY();
    float aZ = myIMU.readFloatAccelZ();
    float gX = myIMU.readFloatGyroX();
    float gY = myIMU.readFloatGyroY();
    float gZ = myIMU.readFloatGyroZ();

    if (dataFile) {
      dataFile.print(aX, 3); dataFile.print(",");
      dataFile.print(aY, 3); dataFile.print(",");
      dataFile.print(aZ, 3); dataFile.print(",");
      dataFile.print(gX, 3); dataFile.print(",");
      dataFile.print(gY, 3); dataFile.print(",");
      dataFile.println(gZ, 3);
    }

    delay(10); // sample rate ~100Hz
  }

  if (logging && millis() - startTime > loggingDuration) {
    logging = false;
    if (dataFile) {
      dataFile.close();
    }
    Serial.println("Logging voltooid. Data uitlezen...\n");

    // Lees het bestand terug
    dataFile = SFUD.open("imu_log.csv", FILE_READ);
    if (dataFile) {
      while (dataFile.available()) {
        Serial.write(dataFile.read());
      }
      dataFile.close();
    }
    Serial.println("\nWacht op nieuwe beweging...");
  }
}