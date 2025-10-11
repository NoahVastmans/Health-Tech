# 🎾 SmartServe Ball — Apex Detection, Apex Estimation & BLE Telemetry

## 📖 Overview

**SmartServe Ball** is an embedded system that detects and measures the motion of a thrown object using a **6-axis IMU (LSM6DS3)** and **Bluetooth Low Energy (BLE)**.

It automatically detects:
- **Free fall start** and **catch impact**
- The **apex (highest point)** of flight
- **Predicted vs. actual apex times**
- And emits a **buzzer signal** for video synchronization or real-time feedback.

Flight metrics are transmitted via BLE to a connected device (e.g., smartphone, PC, or logging app).

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

## ⚙️ System Features

| Feature | Description |
|----------|-------------|
| **Free-fall detection** | Detects when acceleration drops below ~0.1 g |
| **Apex detection** | Zero-crossing of filtered vertical velocity |
| **Apex prediction** | Calculates time to apex based on release velocity |
| **Catch detection** | Detects when acceleration rises after flight |
| **BLE transmission** | Sends flight metrics as a comma-separated string |
| **Buzzer feedback** | Beeps at apex for video synchronization |
| **Continuous mode** | Supports multiple throws in sequence |

---

## 🧩 Hardware Requirements

- **Seeed XIAO BLE Sense** or compatible BLE-enabled Arduino board  
- **LSM6DS3 IMU** (integrated or connected via I²C)
- **Buzzer** connected to **pin 5**
- Optional: mobile app or BLE logger to receive flight data

---

## 🔧 Software Requirements

- **Arduino IDE 2.x**  
- **Libraries:**
  - `LSM6DS3` (Seeed version)
  - `ArduinoBLE`
  - `Wire`

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

## 🧠 Algorithm Summary

### 1. **Sampling**
- The IMU is sampled at **52 Hz** (`fs = 52.0`).
- Each cycle reads acceleration and computes its norm.

### 2. **Filtering**
- **Low-pass filter** smooths acceleration (`fc_acc = 5 Hz`).
- **High-pass filter** integrates to get drift-free velocity (`fc_vel = 50 Hz`).

### 3. **Free-Fall Detection**
- Triggered when acceleration norm < `FREEFALL_THRESH` (≈ –7 m/s²) for at least `FREEFALL_MIN` s.

### 4. **Apex Prediction**
- Using initial velocity (`v₀`), predicts the time to apex:  
  `t_pred = v₀ / g`

### 5. **Apex Detection**
- Detected when filtered velocity crosses zero (positive → negative).
- A buzzer beep is scheduled with a delay compensation of `filter_delay` s.

### 6. **Catch Detection**
- Ends flight when acceleration norm > `CATCH_THRESH` (≈ 1 m/s²).

### 7. **BLE Transmission**
- Sends a string formatted as:  
  `t_pred_apex, apex_since_freefall, freefall_duration`

Example: `0.735, 0.740, 1.488`

---

## 📡 BLE Data Format

| Field | Description | Units |
|--------|--------------|--------|
| `t_pred_apex` | Predicted time from release to apex | s |
| `apex_since_freefall` | Actual measured apex time (since free-fall start) | s |
| `freefall_duration` | Total flight time | s |

BLE UUIDs:
- **Service:** `180A`  
- **Characteristic:** `2A57`

---

## 🔔 Buzzer Behavior

| Event | Action |
|--------|--------|
| Free fall start | Short tone (500 Hz, 100 ms) |
| Apex | 1 kHz tone, delayed by `filter_delay` |
| Catch | No tone; BLE data sent |

---

## ⚙️ Tuning Parameters

| Parameter | Description | Default |
|------------|--------------|----------|
| `fs` | Sampling frequency | 52 Hz |
| `fc_acc` | Low-pass cutoff for acceleration | 5 Hz |
| `fc_vel` | High-pass cutoff for velocity | 50 Hz |
| `FREEFALL_THRESH` | Acceleration threshold (≈ 0.1 g) | –7.0 m/s² |
| `FREEFALL_MIN` | Minimum time below threshold | 0.04 s |
| `CATCH_THRESH` | Catch detection threshold | 1.0 m/s² |
| `filter_delay` | Apex delay compensation | 0.12 s |
| `buzzerDuration` | Tone duration | 500 ms |

---

## 📈 Example BLE Output

```
0.733, 0.751, 1.486
0.812, 0.798, 1.612
0.705, 0.739, 1.423
```

Each line corresponds to one complete throw:
1. Predicted apex time  
2. Detected apex time since free-fall start  
3. Flight duration (end – start)

---

## 🚀 Usage Steps

1. Replace the **LSM6DS3.cpp** file as described above.  
2. Open the `SmartBall.ino` sketch in Arduino IDE.  
3. Select your board (e.g., Seeed XIAO BLE Sense) and COM port.  
4. Upload the sketch.  
5. Open a BLE terminal or app (e.g., **nRF Connect**, **Serial Bluetooth Terminal**).  
6. Start throwing the ball — observe BLE data and buzzer signals.

---

## 🧰 Future Extensions

- Include **gyro-based orientation** for 3D trajectory tracking.  
- Add **drag correction** to apex prediction.  
- Implement **BLE command interface** for tuning parameters remotely.  
- Log data to **SD card** for offline analysis.

---

## 📄 License

This project is open for educational and research use.  
Please credit the **Smart Ball Project** when redistributing or modifying.
