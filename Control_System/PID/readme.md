# PID Control of a DC Motor

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

##  Controllers Implemented

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

##  Closed-Loop Configurations

For each controller, a **closed-loop system** is built where the motor is driven to track a desired reference input.

---

##  Analysis Performed

### 1. Step Response
- Shows how the motor tracks a reference speed.
- Compares rise time, overshoot, and settling time for each controller.

<img width="845" height="540" alt="Step Response" src="https://github.com/user-attachments/assets/6637b9f4-c174-477a-8ea4-4db0fad51f5d" />

---

### 2. Root Locus
- Visualizes system poles as controller gain varies.
- Helps assess **stability margins** and pole locations.
<img width="898" height="567" alt="Screenshot 2025-08-21 at 14 40 15" src="https://github.com/user-attachments/assets/cbbbbec0-ae86-4f21-8028-60713c20899e" />


---

### 3. Bode Plots
- Frequency-domain analysis of each closed-loop system.
- Provides insight into **gain margin** and **phase margin**.
<img width="875" height="572" alt="Screenshot 2025-08-21 at 14 40 20" src="https://github.com/user-attachments/assets/f28a0a64-ce98-4734-ab98-d7bc95a42dc1" />


---

### 4. Performance Metrics
- MATLAB’s **`stepinfo`** function extracts:
  - Rise Time  
  - Settling Time  
  - Overshoot  
  - Steady-State Error  

<img width="449" height="574" alt="Stepinfo Results" src="https://github.com/user-attachments/assets/a85fd983-3f24-4235-9a4f-d088b0cc5daa" />

---

##  Results & Observations

From the simulations, each controller shows distinct behavior:

- **Manual PID** → Works, but requires careful gain selection. Performance is acceptable, but it may result in slower rise time and larger steady-state error if not tuned precisely. Good for learning, but not optimal.  

- **Auto-Tuned PID** → Provides a well-balanced response with **low overshoot and good stability margins**. MATLAB’s tuning algorithm is reliable for achieving robustness without extensive manual adjustments.  

- **Ziegler–Nichols PID** → Produces a **much faster response** but often at the cost of **higher overshoot** and more oscillatory behavior. This makes it suitable when speed is prioritized, but less ideal when smoothness and stability are critical.  

✅ **Conclusion**: Auto-tuned PID provided the most practical balance, while Ziegler–Nichols was the fastest but least stable. Manual tuning is best for experimentation and intuition building.  

---

##  Key Takeaways

- The project highlights how different tuning methods impact system behavior.  
- **Trade-offs** exist between speed, overshoot, and stability.  
- MATLAB’s built-in tools greatly simplify **PID tuning and performance evaluation**.  

---

## 🛠️ Future Improvements

- Add **nonlinear effects** such as actuator saturation and load disturbances.  
- Extend to **position control** (by dividing the transfer function by `s`).    
- Integrate **discrete-time control** for embedded system deployment.  

---
