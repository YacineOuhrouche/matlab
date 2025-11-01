# 🚗 Regenerative Braking Simulation —  MATLAB

This project simulates **regenerative braking** in an electric vehicle (EV), showing how **kinetic energy** can be recovered during deceleration to **recharge the battery**.  
The model incorporates realistic **vehicle dynamics**, **motor drive behavior**, **battery characteristics**, and **thermal effects**, providing a deeper understanding of energy recovery mechanisms in electric drivetrains.

---

## 🧠 Theory Overview

**Regenerative braking** allows an electric motor to act as a **generator** when the vehicle slows down.  
Instead of wasting kinetic energy as heat (as in conventional friction brakes), the energy is **converted into electrical energy** and **stored in the battery**.

During braking:
- The **motor torque (Te)** becomes negative, generating power.  
- The **inverter** converts the generated AC power back into DC.  
- The **battery** absorbs the current, increasing its **State of Charge (SOC)**.  
- The system automatically blends **regenerative** and **mechanical braking** depending on the braking demand.

---

## ⚙️ Features of the Simulation

- Realistic **vehicle model** including drag, rolling resistance, and slope effects  
- **Motor and inverter models** with torque limits and efficiency  
- **Battery model** with SOC tracking, limits, and internal resistance  
- **Thermal model** for estimating motor temperature rise  
- A **WLTP-like drive cycle** for realistic acceleration and braking behavior  

---

## 🧩 Parameters

| Component | Key Parameters |
|------------|----------------|
| **Vehicle** | Mass = 1500 kg, Wheel radius = 0.3 m, Cd = 0.32, Cr = 0.01 |
| **Motor** | Rs = 0.03 Ω, Ld/Lq = 0.6/0.7 mH, λm = 0.015 Wb, Te_max = 250 Nm |
| **Battery** | 300 V, 50 Ah, η = 90%, R_internal = 0.05 Ω, SOC range = 0.2–1.0 |
| **Simulation** | Time = 30 s, fsim = 1 kHz, WLTP-like drive cycle |

---

## 🚘 What is the WLTP Drive Cycle?

The **Worldwide Harmonized Light Vehicle Test Procedure (WLTP)** is a standardized driving cycle used to measure a vehicle’s **energy consumption**, **CO₂ emissions**, and **range** under **realistic driving conditions**.  

In this project, a **simplified WLTP-like profile** is used to represent:
- **Acceleration** from standstill (urban conditions)  
- **Cruising** at moderate speeds  
- **Deceleration** phases to trigger regenerative braking  

This provides a **realistic variation** in vehicle speed, mimicking how an EV behaves during actual driving — ideal for testing energy recovery and battery response.

---
## 🧠 MATLAB Implementation Overview

The MATLAB script simulates the **drive cycle, battery, and vehicle behavior** in a simplified but realistic way.

### 1. Initialize Parameters
- Define key parameters for the **motor, inverter, battery, and vehicle**.  
- Set constants like mass, rolling resistance, drag coefficient, and drive cycle duration.

### 2. Define Drive Cycle (WLTP)
- The **Worldwide Harmonized Light Vehicle Test Procedure (WLTP)** is used to represent real driving conditions.  
- It defines **speed vs. time** for an entire trip, including acceleration, cruising, and braking phases.

### 3. Compute Torque and Power
- Convert the desired **vehicle speed** from the drive cycle into **wheel torque** and **motor torque** using the gear ratio and wheel radius.  
- Calculate **mechanical and electrical power** at each time step.

### 4. Battery and Energy Flow
- Estimate **battery current**, **power demand**, and **state of charge (SOC)** over time.  
- Include **regenerative braking**, where kinetic energy is recovered and stored back into the battery.

### 5. Apply System Limits
- Enforce realistic **motor speed, torque, and power limits**.  
- Account for **efficiency losses** in the inverter and drivetrain.

### 6. Plot Results
- Visualize **vehicle speed**, **torque**, **power flow**, and **battery SOC** over time.  
- Show how energy is consumed and recovered throughout the WLTP cycle.
---

## 📈 Simulation Results

**Main Outputs:**
- Vehicle speed follows the WLTP reference profile.  
- Negative torque regions indicate active regenerative braking.  
- SOC increases during deceleration, demonstrating battery charging.  
- Cumulative recovered energy rises over time.  
- Motor temperature shows a gradual thermal increase due to losses.
<img width="1167" height="788" alt="Screenshot 2025-11-01 at 16 11 03" src="https://github.com/user-attachments/assets/57286ff0-222c-4e4d-a985-6608f3506d6a" />
<img width="976" height="427" alt="Screenshot 2025-11-01 at 16 10 55" src="https://github.com/user-attachments/assets/f0ed345a-b5d8-4b58-a701-e872064304dd" />

**Old Version**
- without WLTP as drive cycle.
- Without battery voltage variation
- without motor temperature calculation
<img width="1115" height="677" alt="Screenshot 2025-11-01 at 16 21 46" src="https://github.com/user-attachments/assets/4a0f379a-7ff4-4df1-b3bf-054071c37841" />
<img width="947" height="407" alt="Screenshot 2025-11-01 at 16 21 58" src="https://github.com/user-attachments/assets/c4be20da-93e6-4d16-98e4-db5dd9de227d" />

---

## ✅ Key Insights

- Regenerative braking **recovers significant energy** during urban driving conditions.  
- Energy recovery is limited by **motor/inverter power** and **battery current limits**.  
- The system must **blend mechanical and electrical braking** for safety and control.  
- Proper modeling of **efficiencies and losses** is crucial for accurate performance estimation.  
- The simulation provides a foundation for **real-world EV control strategies** and **energy optimization**.

---

## 🔍 Future Improvements

- Include a **battery thermal model** for temperature effects.  
- Integrate **PI control** for torque blending between regen and mechanical braking.  
- Extend to multiple **drive cycles**  
- Couple with a **Field-Oriented Control (FOC)** simulation for a full drivetrain model.

---

  
