# Linear Positioner System

A closed-loop linear actuator system built with Arduino, a stepper motor, and multiple safety sensors.  
The system supports two operating modes:

- **Continuous Mode** – The actuator oscillates between two distance thresholds.
- **Target Position Mode** – The actuator moves to a user-defined distance and stops once the target is reached.

This project demonstrates real-time sensor feedback, safe motor control, and interactive Serial Monitor configuration.

---

## ✨ Features

### 🔹 Dual Operating Modes
- **Continuous Mode:**  
  Automatically reverses direction based on IR distance readings (10–30 cm range).
- **Target Position Mode:**  
  Prompts the user for a target distance via Serial Monitor and moves the carriage to that exact position.

### 🔹 Safety Systems
- **Emergency Stop Button**  
  Immediately halts all motion.
- **Light Sensor Lockout**  
  Prevents operation in darkness.
- **Flex Sensor Over‑bend Protection**  
  Stops the system if mechanical strain is detected.
- **Current Sensor Monitoring**  
  Reads motor load (printed to Serial in continuous mode).

### 🔹 Visual Indicators
- **Green LED:** System running normally  
- **Red LED:** System stopped or unsafe  

---

## 🧠 Sensors Used

| Sensor | Purpose |
|--------|---------|
| **IR Distance Sensor** | Primary positioning feedback (cm) |
| **Flex Sensor** | Safety: detects bending/strain |
| **Current Sensor** | Motor load monitoring |
| **Light Sensor** | Safety: prevents operation in darkness |
| **Pushbutton** | Emergency stop |

---

## ⚙️ Motor Control

The stepper motor supports:
- Microstepping (full, half, quarter, eighth, sixteenth)
- Direction control (forward/reverse)
- Speed control via adjustable half‑delay
- One full revolution per `rotate()` call

---

## 🔄 Operating Logic (Summary)

### **1. Safety First**
The system stops immediately if:
- Emergency button is pressed  
- Light sensor detects darkness  
- Flex sensor exceeds safe threshold  

### **2. Continuous Mode**
- If distance < 10 cm → reverse  
- If distance > 30 cm → forward  
- Rotate one full revolution  
- Print current sensor reading  

### **3. Target Position Mode**
- Serial Monitor prompts for target distance  
- System waits for valid input  
- Computes error from target  
- Moves required number of revolutions  
- Fine‑tunes until within ±0.5 cm  
- Stops and marks position as reached  

---

## 🛠️ Hardware Requirements

- Arduino (Uno/Nano/etc.)
- A4988 or similar stepper driver
- NEMA‑style stepper motor
- IR distance sensor
- Flex sensor
- ACS712 or similar current sensor
- Light sensor (digital)
- Emergency stop pushbutton
- LEDs + resistors

---

## 📡 Serial Monitor Interaction

On startup, the system displays:
Select Mode: 1 = Continuous Mode 2 = Target Position Mode

If Mode 2 is selected, the system asks:
Enter target distance in cm:

---

## 🧩 Future Improvements

- PID control for smoother positioning  
- OLED display for live sensor readouts  
- Stall detection using current sensor  
- Non‑blocking motor stepping  
- Homing routine with limit switches  

---
## 🧩 Flow Chart
                     ┌──────────────────────┐
                     │      START SYSTEM     │
                     └───────────┬──────────┘
                                 │
                                 ▼
                   ┌──────────────────────────┐
                   │ Prompt user for mode      │
                   │ 1 = Continuous            │
                   │ 2 = Target Position       │
                   └───────────┬──────────────┘
                               │
                     ┌─────────┴─────────┐
                     ▼                   ▼
        ┌──────────────────────┐   ┌──────────────────────────┐
        │ Continuous Mode       │   │ Target Position Mode      │
        └──────────┬───────────┘   └───────────┬──────────────┘
                   │                           │
                   ▼                           ▼
       ┌──────────────────────┐     ┌──────────────────────────────┐
       │ Read IR distance     │     │ Clear Serial buffer           │
       │ <10 → reverse        │     │ Prompt for target distance    │
       │ >30 → forward        │     │ Wait for valid input          │
       └──────────┬───────────┘     └───────────┬──────────────────┘
                  │                             │
                  ▼                             ▼
       ┌──────────────────────┐     ┌──────────────────────────────┐
       │ Rotate 1 revolution  │     │ Compute error = IR - target   │
       │ Print current        │     │ Choose direction               │
       └──────────┬───────────┘     │ Move required revolutions     │
                  │                 └───────────┬──────────────────┘
                  ▼                             │
                (LOOP)                          ▼
                                     ┌──────────────────────────────┐
                                     │ Fine‑tune until error < 0.5  │
                                     │ Rotate 1 rev per adjustment  │
                                     └───────────┬──────────────────┘
                                                 │
                                                 ▼
                                               (LOOP)




