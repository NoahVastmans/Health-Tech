# 🏓 SmartServe Ball — Intelligent Motion Tracking and Apex Detection

## 📘 Project Overview

The **SmartServe Ball** project combines embedded sensing, wireless communication, and data analysis to accurately detect and predict the **apex** of a tennis ball’s trajectory during a throw or serve.

Using a **Seeed XIAO BLE Sense** microcontroller with a **6-axis IMU (LSM6DS3)**, the system identifies:
- The **start of free fall**
- The **apex (highest point)** of the throw
- The **catch or impact moment**
- And transmits timing data via **Bluetooth Low Energy (BLE)**

The goal is to provide **real-time motion insight** for sports technology, validation through **video tracking**, and a foundation for further embedded motion analytics.

---

## 📁 Repository Structure

### 🧠 `Arduino Scripts/`
Contains all embedded code for the SmartServe Ball, including:
- **Apex detection**, **free-fall**, and **BLE telemetry algorithms**
- Modified **`LSM6DS3.cpp`** file required for stable IMU communication  
  *(instructions are detailed in the script-specific README inside this folder)*

Each Arduino script is fully documented in-code and explained in its local README.

---

### ⚡ `Electrical Circuit/`
Includes the electrical design and assembly details:
- **KiCad project files** for the wiring and sensor connections  
- **Photos** of the final sensor assembly

> 💡 The PCB was **not fully designed** in CAD — instead, an **adaptable PCB prototyping board** was used.  
> This allowed greater flexibility for multi-layer connections and double-sided component placement.

---

### 🎥 `Validation Videos and Video Tracking/`
Contains:
- The **validation videos** used to visually confirm the apex timing  
- **Video tracking scripts** for extracting apex frames and timestamps  
- A **comparison file** matching BLE-detected and predicted times to the visual apex times

This folder provides the link between **experimental results** and **sensor-based detection accuracy**.

---

### 📹 `SmartServe Ball - Explanation Video/`
A short **demonstration video** summarizing the SmartServe Ball project — explaining the concept, system design, and results in a clear visual format.

---

### 🖥️ `Presentation/`
Contains the **final project presentation slides**, summarizing:
- Motivation and goals  
- Technical design (hardware + software)  
- Validation results and future improvements  

---

### 📄 `Scientific Paper/`
Includes a **concise research-style paper** presenting:
- The experimental setup  
- Measured results  
- Comparison between prediction and detection  
- Relevant literature and references  

---

### 🧱 `3D CAD Files/`
3D design files for the **protective casing** used to house the electronics inside a **tennis ball**.  
The case ensures durability while maintaining balance and minimal aerodynamic impact.

---

### 🕰️ `Legacy/`
Contains earlier versions of scripts and design assets intended for future development.  
A separate README inside this folder explains each file’s purpose and how to build upon it.

---

## 🧩 Summary

| Component | Description |
|------------|--------------|
| **Embedded Code** | Apex detection, prediction, BLE telemetry |
| **Electronics** | IMU wiring and modular circuit design |
| **Validation** | Video-based apex verification |
| **Demonstration** | Project explanation video |
| **Documentation** | Presentation slides and scientific paper |
| **Mechanical Design** | 3D CAD for protective case |
| **Legacy Work** | Early-stage materials for continuation |

---

## 🔬 Purpose

The SmartServe Ball demonstrates how **low-power embedded sensing** and **motion algorithms** can yield **accurate trajectory analytics** for sports applications.  
It combines open hardware, transparent validation, and modular design to serve as a **research and development platform** for motion-based performance tracking.

---

## 🧾 License

This repository is open for **educational and research use**.  
Please credit the **SmartServe Ball Project** when reusing or modifying any material.
