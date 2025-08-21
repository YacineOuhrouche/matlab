# 🚀 PID Control of a DC Motor

This project demonstrates the modeling and control of a **DC motor** using different **PID control strategies** in MATLAB. The goal is to analyze and compare how **manual tuning**, **MATLAB’s auto-tune (`pidtune`)**, and **Ziegler–Nichols tuning** affect the motor’s performance.

---

## ⚙️ System Model

The DC motor is modeled with the following parameters:

- **J** → Rotor inertia  
- **B** → Viscous damping coefficient  
- **R** → Armature resistance  
- **L** → Armature inductance  
- **K** → Torque constant   

The transfer function relates **input voltage** to **angular speed**. 

---

## 🎛️ Controllers Implemented

### 1. Manual PID
- Proportional, integral, and derivative gains chosen manually.
- Allows direct experimentation with controller parameters.

### 2. Auto-Tuned PID
- MATLAB’s **`pidtune`** function is used to design a PID controller.
- Provides a balanced trade-off between performance and robustness.

### 3. Ziegler–Nichols PID
- Uses the **ultimate gain (Ku)** and **oscillation period (Tu)**.
- Classical tuning rules are applied to compute **Kp, Ki, Kd**.

---

## 🔄 Closed-Loop Configurations

For each controller, a **closed-loop system** is built where the motor is driven to track a desired reference input.

---

## 📈 Analysis Performed

### 1. Step Response
- Shows how the motor tracks a reference speed.
- Compares rise time, overshoot, and settling time for each controller.

<img width="845" height="540" alt="Step Response" src="https://github.com/user-attachments/assets/6637b9f4-c174-477a-8ea4-4db0fad51f5d" />

---

### 2. Root Locus
- Visualizes system poles as controller gain varies.
- Helps assess **stability margins** and pole locations.

<img width="845" height="562" alt="Root Locus 1" src="https://github.com/user-attachments/assets/9c0a8857-d200-4920-9027-9e3a9645d868" />

<img width="862" height="536" alt="Root Locus 2" src="https://github.com/user-attachments/assets/ec99de0a-cf13-471a-897b-9d104e1fae30" />

<img width="857" height="552" alt="Root Locus 3" src="https://github.com/user-attachments/assets/9ac6b7b1-0bec-4c82-9f89-086e94b82b86" />

---

### 3. Bode Plots
- Frequency-domain analysis of each closed-loop system.
- Provides insight into **gain margin** and **phase margin**.

<img width="891" height="559" alt="Bode 1" src="https://github.com/user-attachments/assets/6377a9a7-7ed5-4d3f-8d67-593ecbdddccf" />

<img width="875" height="547" alt="Bode 2" src="https://github.com/user-attachments/assets/75b1653f-8cd5-4fce-bd28-4b308768e719" />

<img width="865" height="545" alt="Bode 3" src="https://github.com/user-attachments/assets/9b677f06-dbf5-495d-850b-4c15fd002762" />

---

### 4. Performance Metrics
- MATLAB’s **`stepinfo`** function extracts:
  - Rise Time  
  - Settling Time  
  - Overshoot  
  - Steady-State Error  

<img width="449" height="574" alt="Stepinfo Results" src="https://github.com/user-attachments/assets/a85fd983-3f24-4235-9a4f-d088b0cc5daa" />

---

## 📊 Results & Observations

From the simulations, each controller shows distinct behavior:

- **Manual PID** → Works, but requires careful gain selection. Performance is acceptable, but it may result in slower rise time and larger steady-state error if not tuned precisely. Good for learning, but not optimal.  

- **Auto-Tuned PID** → Provides a well-balanced response with **low overshoot and good stability margins**. MATLAB’s tuning algorithm is reliable for achieving robustness without extensive manual adjustments.  

- **Ziegler–Nichols PID** → Produces a **much faster response** but often at the cost of **higher overshoot** and more oscillatory behavior. This makes it suitable when speed is prioritized, but less ideal when smoothness and stability are critical.  

✅ **Conclusion**: Auto-tuned PID provided the most practical balance, while Ziegler–Nichols was the fastest but least stable. Manual tuning is best for experimentation and intuition building.  

---

## ✅ Key Takeaways

- The project highlights how different tuning methods impact system behavior.  
- **Trade-offs** exist between speed, overshoot, and stability.  
- MATLAB’s built-in tools greatly simplify **PID tuning and performance evaluation**.  

---

## 🛠️ Future Improvements

- Add **nonlinear effects** such as actuator saturation and load disturbances.  
- Extend to **position control** (by dividing the transfer function by `s`).    
- Integrate **discrete-time control** for embedded system deployment.  

---
