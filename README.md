# Linear Positioner System

A multi-mode Arduino-based linear positioning system with closed-loop position and force control, featuring real-time sensor feedback, PID control, and sequence automation.
---

## ✨ Features

🔄 **Continuous Mode** – Automatic back-and-forth motion within bounds
🎯 **Target Position Mode** – Closed-loop position control using IR sensor
🔁 **Sequence Mode** – Execute multiple target positions with dwell time
💪 **Force Control Mode** – Closed-loop force regulation using FSR
⚡️ **Current Sensor Monitoring**  - Reads motor load (printed to Serial in continuous mode)
📉 **EMA Filtering** – Smooth sensor readings for stable control
⚙️ **PID-Based Control** – Adjustable gains for precise movement
⏱️ **Dwell Time Control** – Pause at targets during sequence execution
🟢🔴 **Safety System** – Light sensor + emergency stop button

### 🔹 Visual Indicators
- **Green LED:** System running normally  
- **Red LED:** System stopped or unsafe  

---

## 🧠 Sensors Used

| Sensor | Purpose |
|--------|---------|
| **IR Distance Sensor** | Positioning feedback (cm) |
| **Flex FSR Sensor** | Force feedback (N) |
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
- System uses PID control to reach and hold position
- Includes tolerance band and settling time

### **4. Sequence Mode**

- User inputs multiple targets:
    e.g. 10 20 15 25
- System moves through each target in order
- Includes dwell time at each position

### **5. Force Control Mode**
- User inputs target force (N)
- System:
  - Approaches object until contact detected
  - Regulates applied force using FSR feedback
  - Uses filtered force signal and closed-loop control

---

## 🛠️ Hardware Requirements

- Arduino (Uno)
- A4988 or similar stepper driver
- NEMA‑style stepper motor
- IR distance sensor
- Thin film flexible FSR sensor
- ACS712 or similar current sensor
- Light sensor (digital)
- Emergency stop pushbutton
- LEDs + resistors

---

## 📡Sensor Filtering

An Exponential Moving Average (EMA) filter is applied:

filtered = α × new + (1 - α) × previous
Distance α ≈ 0.3
Force α ≈ 0.2

This reduces noise while maintaining responsiveness.

---

## 🧩 Future Improvements

- OLED display for live sensor readouts  
- Stall detection using current sensor  
- Non‑blocking motor stepping  
- Homing routine with limit switches
- Hybrid position + force control

---
## 🧩 Flow Chart
                                                                  ┌──────────────────────┐
                                                                  │      START SYSTEM    │
                                                                  └───────────┬──────────┘
                                                                              │
                                                                              ▼
                                                                ┌──────────────────────────┐
                                                                │ Prompt user for mode     │
                                                                │ 1 = Continuous           │
                                                                │ 2 = Target Position      │
                                                                │ 3 = Target Position      │
                                                                │ 4 = Target Position      │
                                                                └───────────┬──────────────┘
                                                                            │
                     ┌───────────────────────────────────────────┐──────────┴──────────────────────────────┐─────────────────────────────────────────┐
                     ▼                                           ▼                                         ▼                                         ▼
        ┌──────────────────────┐                  ┌──────────────────────────┐                 ┌──────────────────────┐                 ┌──────────────────────────┐
        │ Continuous Mode      │                  │ Target Position Mode     │                 │ Sequence Mode        │                 │ Force Control Mode       │ 
        └──────────┬───────────┘                  └───────────┬──────────────┘                 └──────────┬───────────┘                 └───────────┬──────────────┘      
                   ▼                                          ▼                                           ▼                                         ▼
       ┌───────────────────────────────┐      ┌────────────────────────────────────┐        ┌──────────────────────────────┐          ┌──────────────────────────────┐
       │ Read filtered IR distance     │      │ Clear Serial buffer                │        │ Clear Serial buffer          │          │ Clear Serial buffer          │
       │ If < lower bound → reverse    │      │ Prompt for target distance         │        │ Prompt for sequence input    │          │ Prompt for target force      │
       │ If > upper bound → forward    │      │ Wait for valid input               │        │ Parse values into array      │          │ Wait for valid input         │
       └──────────┬────────────────────┘      └─────────────────┬──────────────────┘        │ Validate all targets         │          └───────────┬──────────────────┘
                  │                                             │                           │ Set current target = first   │                      │
                  ▼                                             ▼                           │  in sequence                 │                      ▼
       ┌──────────────────────┐               ┌────────────────────────────────────┐        └───────────┬──────────────────┘          ┌───────────────────────────────┐  
       │ Rotate motor         │               │ Read filtered IR distance          │                    │                             │ Read FSR value                │
       │ Print current        │               │ Compute error = target - position  │                    ▼                             │ Apply EMA filtering           │
       └──────────┬───────────┘               │ Apply PID control                  │        ┌──────────────────────────────┐          │ If force < contact threshold: │
                  │                           │ Set motor direction and speed      │        │ Compute error = IR - target  │          │ → Move forward slowly         │
                  ▼                           └─────────────────┬──────────────────┘        │ Choose direction             │          └───────────┬───────────────────┘
                (LOOP)                                          ▼                           │ Move toward target (PID)     │                      │
                                              ┌───────────────────────────────────┐         └───────────┬──────────────────┘                      ▼
                                              │ If error within tolerance:        │                     │                             ┌──────────────────────────────┐
                                              │ Once settled: → Hold position     │                     ▼                             │ Compute error = N - target   │
                                              └─────────────────┬─────────────────┘        ┌──────────────────────────────┐           │ Apply control (PID)          │
                                                                │                          │ If error < tolerance:        │           │ Adjust motor speed           │
                                                                ▼                          │ → Hold position              │           │ Set direction based on error │
                                                              (LOOP)                       │ → Start dwell timer          │           └───────────┬──────────────────┘
                                                                                           └───────────┬──────────────────                        │
                                                                                                       │                                          ▼
                                                                                                       ▼                              ┌────────────────────────────┐
                                                                                           ┌──────────────────────────────┐           │ If error within tolerance: │
                                                                                           │ After dwell time:            │           │ → Hold position            │
                                                                                           │ → Move to next target        │           └──────────┬─────────────────┘
                                                                                           │ → Repeat for remainder       │                      │
                                                                                           └───────────┬──────────────────┘                      ▼
                                                                                                       │                                       (LOOP)
                                                                                                       ▼
                                                                                                     (LOOP)



