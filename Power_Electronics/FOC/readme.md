# ⚡ Field-Oriented Control (FOC) of a PMSM — MATLAB Simulation

This project demonstrates a **Field-Oriented Control (FOC)** implementation for a **Permanent Magnet Synchronous Motor (PMSM)** using **MATLAB**.  
It models both the **electrical** and **mechanical** dynamics of the motor, with cascaded **current and speed control loops**, feedforward decoupling, and voltage saturation handling.  

The goal is to understand how FOC enables **precise torque and speed control**, which is widely used in **electric vehicle drives**, **industrial motor control**, and **high-performance servo systems**.

---

## 🌀 What is Field-Oriented Control (FOC)?

Field-Oriented Control (FOC), also known as **vector control**, is a method to control AC motors (like PMSMs or BLDC motors) by **decoupling torque and flux components** of stator currents.  
This allows the motor to behave **like a separately excited DC motor**, where flux and torque are independently controllable.

### Key Concepts:
- **dq-axis transformation:** Three-phase currents (`i_a, i_b, i_c`) are transformed into a **rotating reference frame** aligned with the rotor flux.  
  - `i_d` → controls **magnetic flux**  
  - `i_q` → controls **electromagnetic torque**  
- **Decoupling:** Cross-coupling between d- and q-axis is compensated via feedforward terms, improving dynamic performance.  
- **Nested Control Loops:**
  - **Inner loop:** Regulates dq currents at high bandwidth (~kHz range)  
  - **Outer loop:** Regulates speed or position at lower bandwidth  

### Advantages of FOC:
- Smooth and precise torque control  
- Fast dynamic response and accurate speed tracking  
- Reduced torque ripple and losses  
- Widely used in EVs, robotics, and industrial drives  

---

## ⚙️ PMSM Model Overview

The PMSM is modeled using **dq-axis voltage equations** and **mechanical dynamics**.  

### Electrical Dynamics:
$$
\begin{cases}
v_d = R_s i_d + L_d \dfrac{di_d}{dt} - \omega_e L_q i_q \\[2mm]
v_q = R_s i_q + L_q \dfrac{di_q}{dt} + \omega_e (L_d i_d + \lambda_m)
\end{cases}
$$


Where:  
- `v_d, v_q` → dq-axis voltages applied by inverter  
- `i_d, i_q` → dq-axis currents  
- `R_s, L_d, L_q` → stator resistance and inductances  
- `λ_m` → permanent magnet flux linkage  
- `ω_e` → electrical angular speed  

### Mechanical Dynamics:
$$
J \, \dot{\omega}_m + B \, \omega_m = T_e - T_\text{load}
$$

Where:  
- `J` → rotor inertia  
- `B` → viscous friction  
- `T_e` → electromagnetic torque (`3/2 * p * (λ_m i_q + (L_d-L_q) i_d i_q)`)  
- `T_load` → applied mechanical load  

### Simulation Parameters:

| Parameter | Symbol | Description | Value |
|-----------|--------|-------------|-------|
| Stator resistance | Rs | Electrical resistance of stator | 0.03 Ω |
| d-axis inductance | Ld | d-axis stator inductance | 0.0006 H |
| q-axis inductance | Lq | q-axis stator inductance | 0.0007 H |
| Permanent magnet flux | λm | Rotor flux linkage | 0.015 Wb |
| Pole pairs | p | Number of rotor pole pairs | 4 |
| Rotor inertia | J | Mechanical inertia | 0.02 kg·m² |
| Viscous friction | B | Rotor friction | 0.001 N·m·s |
| DC-link voltage | Vdc | Inverter voltage | 300 V |
| Simulation freq | fsim | Sampling frequency | 20 kHz |

---

## 🎯 Control Architecture

### 🌀 Outer Speed Loop
- Generates the **torque-producing current** reference `i_q*`.
- PI controller with **anti-windup**.
- Enforces current saturation limits.

### ⚡ Inner Current Loops
- Regulate the **d-axis (`i_d`)** and **q-axis (`i_q`)** currents.
- Include **cross-coupling decoupling** terms.
- Maintain torque and flux independently for precise control.

### 🧩 Additional Features
- **SVPWM Voltage Limit:** Voltage reference is capped at `0.577 × Vdc` for sinusoidal PWM implementation.  
- **Anti-Windup:** Integral terms are limited to prevent controller saturation.  
- **Load Torque Ripple:** Simulates real-world torque disturbances for robust testing.  

---

## 🖥️ MATLAB Script Implementation

The MATLAB script implements the FOC simulation as follows:

### 1. Initialize Motor and Simulation Parameters
- Define `Rs`, `Ld`, `Lq`, `λm`, number of pole pairs, rotor inertia, friction, DC-link voltage, simulation frequency, and time vector.

### 2. Controller Setup
- Define PI gains for **current loops (inner)** and **speed loop (outer)**.
- Set voltage and current limits.

### 3. Preallocate State Variables
- Currents (`i_d`, `i_q`), voltages (`v_d`, `v_q`), rotor speed (`omega_m`), rotor angle (`theta_r`), torque, DC-link current, and phase currents.

### 4. Simulation Loop (Time-Stepping)
- **Compute electrical rotor angle** and rotor speed integration.
- **Speed control:** Calculate torque-producing current reference `i_q*`.
- **Current control:** PI regulation of `i_d` and `i_q`.
- **Feedforward decoupling:** Compensate cross-coupling terms in voltage calculation.
- **Voltage saturation:** Apply SVPWM voltage limits.
- **Motor dynamics:** Update currents (`i_d`, `i_q`) and compute electromagnetic torque.
- **Mechanical dynamics:** Update rotor speed (`omega_m`) including load torque.
- **Phase currents reconstruction** from dq-frame to abc-frame.
- **DC-link current estimation** from instantaneous power.

### 5. Plot Results
- **Rotor speed vs. reference**
- **Electromagnetic torque**
- **dq-axis currents and DC-link current**

This setup demonstrates **realistic FOC behavior**, including speed regulation, torque control, and robustness to load variations.

---

## 📊 Visualization & Results

### Rotor Speed and Torque
- Plots show rotor speed tracking the reference and torque response under load.

### dq Currents and DC-Link Current
- Plots show `i_d` maintaining flux control (near 0 for surface PMSM) and `i_q` following the torque demand, while DC-link current reflects power drawn.

---

## ✅ Key Takeaways
- FOC achieves **independent torque and flux control** for PMSMs.
- Proper **PI tuning** ensures stable speed a


## 📊 Visualization & Results

### Rotor Speed and Torque
![Speed and Torque Plot](https://github.com/user-attachments/assets/287c1c3e-1cc8-4373-a367-48345cf9433d)

### dq Currents and DC-Link Current
![Currents and DC Link Plot](https://github.com/user-attachments/assets/c3f6c8d6-58a1-4246-ab93-3043f4e0beb5)

---

## ✅ Key Takeaways

- FOC achieves **independent torque and flux control** for PMSMs.  
- Proper **PI tuning** ensures stable speed and current regulation.  
- Decoupling, voltage limits, and anti-windup are critical for realistic control.  
- Simulated load disturbances demonstrate robustness of the FOC strategy.  

---

## 🛠️ Future Work

- Extend to **sensorless FOC** using back-EMF estimation.  
- Implement **field-weakening control** for high-speed operation.  
- Integrate a **3-phase inverter model** for hardware-level validation.  
- Include **dynamic reference trajectories** to test advanced control strategies.
