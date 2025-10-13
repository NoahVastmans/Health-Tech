### 🕰️ `Legacy/`
Contains earlier versions of scripts and design assets intended for future development. 

## 📖 Overview

**SmartServe Ball** is an embedded system that detects and measures the motion of a thrown object using a **6-axis IMU (LSM6DS3)** and **Bluetooth Low Energy (BLE)**.

The earlier versions of the scripts feature:
- **Rotational Speed**
- The **flight stages** detection
- **AHRS algorithm** implementation
- **Quaternion rotation** for absolute global frame sensor orientation 

---

## 🧭 Setup Prerequisites

Before you begin, make sure your development environment is correctly configured for the **Seeed XIAO BLE Sense**.

1. 📘 **Read the official Seeed Studio setup guide:**  
   👉 [https://wiki.seeedstudio.com/XIAO_BLE/](https://wiki.seeedstudio.com/XIAO_BLE/)  
   This page explains how to properly set up the board, drivers, and Arduino environment.

2. ⚙️ **Install the correct board package:**  
   In the Arduino IDE, go to  
   `Tools → Board → Board Manager`  
   and install:  
   **“Seeed nRF52 mbed-enabled Boards”**  
   **Do not install the non-mbed version**, as it may cause upload and sensor initialization issues.

Once the board package is installed, select:  
`Tools → Board → Seeed XIAO BLE Sense`

---


## 🧩 Hardware Requirements

- **LSM6DS3 IMU** (integrated or connected via I²C)
- **Buzzer** connected to **pin 5**

---

## 🔧 Software Requirements

- **Arduino IDE 2.x**  
- **Libraries:**
  - `LSM6DS3` (Seeed version)
  - `ArduinoBLE`
  - `Wire`
  - `MadgwickAHRS`

---

## ⚠️ Important — LSM6DS3.cpp Modification

Before running this code, you **must replace** the `LSM6DS3.cpp` file in your installed library folder with the **modified version** included in this project.

This modified file ensures **stable and faster sensor reads** compatible with the SmartServe Ball’s timing requirements.

### 🔹 Steps to Replace the File:

1. Locate your LSM6DS3 library folder:
   - **Windows:**  
     `Documents/Arduino/libraries/Seeed_Arduino_LSM6DS3/`
   - **macOS/Linux:**  
     `~/Documents/Arduino/libraries/Seeed_Arduino_LSM6DS3/`

2. Find and replace the file:
   - Replace the original `LSM6DS3.cpp` with the version found in this SmartServe Ball project folder.

3. Restart the Arduino IDE.

---

## ⚠️ Important — MadgwickAHRS.h Modification

Before running this code, you **must replace** the `MadgwickAHRS.h` file in your installed library folder with the **modified version** included in this project.

This modified file ensures **access to quaternion variables** to obtain the acceleration in the global framework.

### 🔹 Steps to Replace the File:

1. Locate your LSM6DS3 library folder:
   - **Windows:**  
     `Documents/Arduino/libraries/Seeed_Arduino_LSM6DS3/`
   - **macOS/Linux:**  
     `~/Documents/Arduino/libraries/Seeed_Arduino_LSM6DS3/`

2. Find and replace the file:
   - Replace the original `MadgwickAHRS.cpp` with the version found in this SmartServe Ball project folder.

3. Restart the Arduino IDE.

---

## 🧠 Algorithm Summary

### 1. **Sampling**
- The Madgwick algorithm is set to sample at **100 Hz**.
- Each cycle reads acceleration and gyroscope

### 2. **Updates Orientation**
- **Madgwick Filter Update** uses both raw acceleration and gyroscope data to correct IMU orientation pith/roll.
- Yaw is not reliable, as IMU is missing magnetometer. Therefore, it will drift.

### 3. **Calculate time step**
- **Time step** for velocity integration.

### 4. **Quaternion Rotation Matrix**
- **Quaternion Matrix** rotate acceleration vectors to obtain vertical acceleration in the global frame.
- This allows to avoid "fake" accelerations caused by orientation changes.
  - Example: rotation change 20°, acceleration gets projected measures a change in the acceleration, whereas with global frame vertical acceleration has not changed.

### 5. **Filtering**
- **Low-pass filter** smooths acceleration.

### 6. **Vertical Velocity**
- **Velocity integration** vertical velocity is computed off the acceleration data.

### 7. **Throw Initiate**
- If started from a static position, triggered when vertical acceleration < `THROWING_ACCEL_THRESHOLD` (≈ 10 m/s²).

### 8. **Free-Fall Detection**
- If throw has initiate, then is triggered when vertical acceleration < `GRAVITY_THRESH` (≈ 2 m/s²) AND IMU's free fall status register.

### 9. **Apex Detection**
- Detected when filtered velocity crosses zero (positive → negative).
- A buzzer beep is scheduled.
- Small threshold added for zero crossing

### 10. **Catch Detection**
- Ends flight when acceleration norm > `CATCH_ACCEL_THRESHOLD` (≈ 15 m/s²).
---
---

## 🔔 Buzzer Behavior

| Event | Action |
|--------|--------|

| Apex | 1 kHz tone, for 0.2 s |


---

## ⚙️ Tuning Parameters

| Parameter | Description | Default |
|------------|--------------|----------|
| `alpha` | Low-pass filter for acceleration | 0.5 |
| `GRAVITY_THRESHOLD` | Free fall threshold | 2 m/s² |
| `THROWING_ACCEL_THRESH` | Throw initiate detection threshold | 10 m/s² |
| `CATCH_ACCEL_THRESH` | Catch detection or clear impact threshold | 15 m/s² |

---

## 🚀 Usage Steps

1. Replace the **LSM6DS3.cpp** and **MadgwickAhrs.h** files as described above. 
2. Open the `SmartBall.ino` sketch in Arduino IDE. 
3. Select your board (e.g., Seeed XIAO BLE Sense) and COM port. 
4. Upload the sketch. 
5. Open the serial monitor terminal  
6. Start throwing the ball — observe data and buzzer signals.

---

## 🧰 Future Extensions

- Use a bandpass algorithm to cut off **noise frequencies** for accurate acceleration.
- Synchronized IMU data sampling and Madgwick filter. 
- Implement transmit via **BLE command interface** to a connected device (e.g., smartphone, PC, or logging app).
- Log data to **SD card** for offline analysis.

---

## 📄 License

This project is open for educational and research use. 
Please credit the **Smart Ball Project** when redistributing or modifying.

