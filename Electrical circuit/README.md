# ⚡ SmartServe Ball — Electrical Circuit

## 📘 Overview

This folder contains all **electrical design resources** for the SmartServe Ball project.  
The system connects a **Seeed XIAO BLE Sense** microcontroller to a **buzzer** and other peripheral components.

---

## 🧩 Contents

| Item | Description |
|------|--------------|
| `SmartServeBall.kicad_sch` | Main circuit schematic showing all connections between the XIAO BLE Sense, IMU, and buzzer |
| `SmartServeBall.kicad_pro` | KiCad project configuration file |
| `Seeeduino XIAO KICAD/` | KiCad symbol and footprint files for Seeed components (used in the schematic) |
| `Assembly Photos/` | Images of the final sensor assembly, wiring layout, and breadboard/PCB setup |

---

## 🧠 Design Notes

- Only the **schematic** was designed digitally in KiCad.  
- The **PCB itself was not routed** in KiCad or fabricated.

Instead, an **adaptable prototyping PCB** was used for greater flexibility:
- Components were mounted on **both sides** of the board.  
- Components were mounted on top of each other to save space.  
- This approach allowed **rapid iteration** and **easy modification** during testing and validation.

---

## 🔌 Connection Overview

The schematic illustrates: 
- **Power lines** (`3.3V` and `GND`) shared between all components  
- **Digital output pin (D5)** used to drive the **buzzer** for apex and event feedback

---

## 🧰 Tools

- **KiCad 7.x or later** (for viewing and editing schematic files)  
- **Seeeduino XIAO KICAD Library** (included in this folder)

To open the schematic correctly:
1. Ensure the `Seeeduino XIAO KICAD/` folder is in the same directory as the `.kicad_pro` file.  
2. Open `SmartServeBall.kicad_pro` in KiCad.  
3. Verify that component symbols and footprints load without errors.

---

## 🧱 Assembly Process

The assembly was performed in several structured steps to create a compact and robust sensor module:

1. **Top Layer Soldering:**  
   The first layer of components was soldered on the **upper side** of the adaptable PCB.  
   This included the **buzzer**, **resistor**, and **pin extensions** for the Seeed XIAO BLE Sense.

2. **Power Wiring:**  
   Next, the **power supply wires** were soldered to the corresponding **3.3V** and **GND** pins, ensuring solid electrical contact and mechanical stability.

3. **Base Layer Protection:**  
   After soldering the lower connections, a **thin layer of hot glue** was applied to the **bottom side** of the PCB to insulate and protect the solder joints.

4. **Battery Holder Placement:**  
   The **battery holder** was then positioned directly on top of the glued surface, forming the **second hardware layer** of the design.

5. **Final Power Connection:**  
   Finally, the **power wires** were soldered to the **battery holder terminals**, completing the electrical circuit and enabling portable operation.

This stacked-layer assembly provided an **efficient use of space** and **robust wiring**, allowing the electronics to fit neatly inside the SmartServe Ball’s protective enclosure.

---

## 📷 Assembly

The `Assembly_Photos/` folder shows:
- The **adaptable PCB layout**  
- The **component placement** on both board sides  

These images serve as a **reference for replication or redesign**.

---

## 🧾 License

These files are shared for **educational and research purposes** under the SmartServe Ball project.  
Please credit the **SmartServe Ball Team** when reusing or adapting the schematic or library components.
