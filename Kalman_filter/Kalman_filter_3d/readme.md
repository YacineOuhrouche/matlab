# 🚀 3D Object Tracking with Extended Kalman Filter (EKF)

This project demonstrates **tracking a 3D object moving along a complex trajectory** using an **Extended Kalman Filter (EKF)** in MATLAB.  
The EKF estimates **position** and **velocity** in all three dimensions from noisy position measurements while handling nonlinear motion dynamics.

---

## ⚙️ System Model

The object moves along a **3D circular + sinusoidal trajectory** with:

- **Radius (X-Y plane)** → 10 m  
- **Angular velocity** → 0.2 rad/s  
- **Z-axis amplitude** → 5 m  
- **Z-axis oscillation frequency** → 0.1 Hz  

The EKF uses a **state vector** containing the object's **3D position and velocity**, while measurements provide only noisy **x, y, z positions**.  

Since the motion is nonlinear, the EKF **linearizes the system at each time step** to improve estimation accuracy.

---

## 🔄 EKF Implementation

### 1. Prediction
- Predicts the object's next state based on **previous estimates** and **3D motion dynamics**.  
- Accounts for **process noise** to handle uncertainty.

### 2. Update
- Corrects the predicted state using **noisy 3D measurements**.  
- Computes the **Kalman Gain** to optimally combine predictions and measurements.

---

## 📈 Visualization

### 1. 3D Position Tracking
- Compares the **true 3D path**, **noisy measurements**, and **EKF estimates**.  
<img width="827" height="558" alt="Screenshot 2025-09-27 at 17 33 24" src="https://github.com/user-attachments/assets/7b34408a-6c04-4826-b5d2-119c643c0aa3" />



### 2. Velocity Tracking
- Tracks the **x, y, and z velocities** estimated by the EKF against the true velocities.  

<img width="890" height="556" alt="Screenshot 2025-09-27 at 17 33 29" src="https://github.com/user-attachments/assets/294e56a3-6d4e-4585-bf82-896f0569963a" />


### 3. Estimation Errors
- Plots **position and velocity estimation errors over time** to analyze EKF performance.  

<img width="826" height="240" alt="Screenshot 2025-09-27 at 17 33 39" src="https://github.com/user-attachments/assets/441a4f63-e28f-4362-9e79-65d4b11affa3" />

<img width="848" height="256" alt="Screenshot 2025-09-27 at 17 33 43" src="https://github.com/user-attachments/assets/33383d9b-9cc6-4ea4-b8fe-91c0bb62d79e" />

---

## 📊 Key Observations

- The EKF effectively **follows the 3D trajectory** despite measurement noise.  
- Velocity estimates capture the **changing direction in all three axes**.  
- Estimation error plots show **small deviations**, confirming filter accuracy.  
- Using a linear Kalman Filter would be **less accurate** for this nonlinear 3D system.

---

## 🛠️ Future Improvements

- Introduce **sensor fusion** (e.g., GPS + IMU) for more realistic tracking.  
- Implement **Unscented Kalman Filter (UKF)** for improved nonlinear estimation.  
- Visualize **animated 3D tracking** to better demonstrate filter performance.  
- Quantitatively analyze **estimation errors** using RMSE or covariance plots.
