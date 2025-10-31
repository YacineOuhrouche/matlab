# ⚡ Field-Oriented Control (FOC) of a PMSM — MATLAB Simulation

This project demonstrates a **Field-Oriented Control (FOC)** implementation for a **Permanent Magnet Synchronous Motor (PMSM)** using **MATLAB**.  
It models both the **electrical** and **mechanical** dynamics of the motor, with cascaded **current and speed control loops**, feedforward decoupling, and voltage saturation handling.  

The goal is to understand how FOC enables **precise torque and speed control**, which is widely used in **electric vehicle drives**, **industrial motor control**, and **high-performance servo systems**.

---

## 🌀 What is Field-Oriented Control (FOC)?

Field-Oriented Control (FOC), also known as **vector control**, is a technique for controlling AC motors (like PMSMs or BLDC motors) by **decoupling torque and flux control**, making the motor behave like a separately excited DC motor.  

Key points:  
- Converts three-phase currents into a rotating **dq-axis reference frame** aligned with the rotor flux.  
- Regulates **d-axis current (`i_d`)** to control flux, and **q-axis current (`i_q`)** to control torque independently.  
- Enables **fast dynamic response**, **high efficiency**, and **smooth torque output**.  
- Commonly used in **electric vehicles**, **robotics**, and **precision servo drives**.

Benefits:  
- Independent torque and flux control → precise and smooth operation.  
- Efficient operation across wide speed ranges.  
- Reduces torque ripple compared to simpler control methods.  

---

## ⚙️ System Overview

The PMSM model includes key machine parameters:

| Parameter | Symbol | Description | Typical Value |
|------------|---------|-------------|----------------|
| Rs | `R_s` | Stator resistance | 0.03 Ω |
| Ld | `L_d` | d-axis inductance | 0.0006 H |
| Lq | `L_q` | q-axis inductance | 0.0007 H |
| λm | `lambda_m` | Permanent magnet flux linkage | 0.015 Wb |
| p | `p` | Pole pairs | 4 |
| J | `J` | Rotor inertia | 0.02 kg·m² |
| B | `B` | Friction coefficient | 0.001 N·m·s |
| Vdc | `V_dc` | DC-link voltage | 300 V |

The system is simulated in continuous time with a sampling frequency of **20 kHz**.

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


---

## 🧩 Simulation Highlights

✅ **Nested PI Controllers** for speed and current loops  
✅ **Feedforward Decoupling** for dq-axis cross coupling  
✅ **SVPWM Voltage Limit** handling (`0.577 × Vdc`)  
✅ **Anti-Windup Integrator Protection**  
✅ **Load Torque Ripple** for realistic response  

---

## 📊 Visualization & Results
<img width="993" height="740" alt="Screenshot 2025-10-31 at 13 56 28" src="https://github.com/user-attachments/assets/287c1c3e-1cc8-4373-a367-48345cf9433d" />
<img width="994" height="645" alt="Screenshot 2025-10-31 at 13 56 19" src="https://github.com/user-attachments/assets/c3f6c8d6-58a1-4246-ab93-3043f4e0beb5" />


---

## ✅ Key Takeaways

- FOC achieves **independent torque and flux control** for PMSMs.  
- Proper **PI tuning** ensures stable speed and current regulation.  
- Decoupling and saturation handling are vital for realistic control.  

---

## 🛠️ Future Work

- Extend to **sensorless FOC** using back-EMF estimation.  
- Implement **field-weakening control** for high-speed range.  
- Integrate a **3-phase inverter model** for hardware-level validation.  

---


