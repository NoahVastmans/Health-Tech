#include <Arduino.h>
#include "LSM6DS3.h"
#include <SPI.h>
#include <Adafruit_SPIFlash.h>

LSM6DS3 lsm6ds3;

// SPI flash setup
#define FLASH_CS 10
Adafruit_FlashTransport_SPI flashTransport(FLASH_CS, &SPI);
Adafruit_SPIFlash flash(&flashTransport);

unsigned long writeAddr = 0;
const unsigned long FLASH_SIZE = 0x400000; // adjust to your chip

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Logger start (IMU + Adafruit SPIFlash)");

  if (!lsm6ds3.begin()) {
    Serial.println("IMU init FAILED");
    while (1) delay(1000);
  }
  Serial.println("IMU init OK");

  if (!flash.begin()) {
    Serial.println("Flash init FAILED");
    while (1) delay(1000);
  }
  Serial.println("Flash init OK");

  // Erase first sector for fresh logging
  flash.eraseSector(0);
  writeAddr = 0;

  Serial.println("Commands: dump | resetlog");
}

void dumpFlash() {
  Serial.println("=== FLASH DUMP START ===");
  const size_t CHUNK = 64;
  uint8_t buf[CHUNK];
  unsigned long addr = 0;

  while (addr < writeAddr) {
    size_t readLen = min(CHUNK, writeAddr - addr);
    flash.readBuffer(addr, buf, readLen);
    for (size_t i = 0; i < readLen; i++) {
      Serial.write(buf[i]);
    }
    addr += readLen;
  }
  Serial.println("\n=== FLASH DUMP END ===");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("dump")) {
      dumpFlash();
    } else if (cmd.equalsIgnoreCase("resetlog")) {
      flash.eraseSector(0);
      writeAddr = 0;
      Serial.println("Flash reset OK");
    }
  }

  static unsigned long lastMs = 0;
  unsigned long now = millis();
  const unsigned long intervalMs = 10; // ~100 Hz

  if (now - lastMs >= intervalMs) {
    lastMs = now;

    float ax = lsm6ds3.readFloatAccelX();
    float ay = lsm6ds3.readFloatAccelY();
    float az = lsm6ds3.readFloatAccelZ();
    float gx = lsm6ds3.readFloatGyroX();
    float gy = lsm6ds3.readFloatGyroY();
    float gz = lsm6ds3.readFloatGyroZ();

    char line[128];
    int len = snprintf(line, sizeof(line),
                       "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       now, ax, ay, az, gx, gy, gz);

    if (writeAddr + len < FLASH_SIZE) {
      flash.writeBuffer(writeAddr, (uint8_t*)line, len);
      writeAddr += len;
    } else {
      Serial.println("Flash full!");
      while (1) delay(1000);
    }
  }
}
