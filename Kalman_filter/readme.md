# 🚀 2D Object Tracking with Extended Kalman Filter (EKF)

This project demonstrates **tracking a 2D object moving in a circular path** using an **Extended Kalman Filter (EKF)** in MATLAB.  
The EKF estimates both **position** and **velocity** of the object from noisy position measurements while handling nonlinear motion dynamics.

---

## ⚙️ System Model

The object moves along a circular trajectory with:

- **Radius** → 10 m  
- **Angular velocity** → 0.2 rad/s  

The EKF uses a **state vector** containing the object's **position and velocity**, while measurements provide only the noisy **x and y positions**.  

Since the motion is nonlinear, the EKF linearizes the system at each time step to improve estimation accuracy.

---

## 🔄 EKF Implementation

### 1. Prediction
- Predicts the object's next state based on the previous estimate and motion dynamics.  
- Accounts for process noise to handle uncertainty.

### 2. Update
- Corrects the predicted state using noisy measurements.  
- Computes the Kalman Gain to optimally combine predictions and measurements.

---

## 📈 Visualization

### 1. Position Tracking
- Compares the **true circular path**, **noisy measurements**, and **EKF estimates**.
<img width="883" height="567" alt="Screenshot 2025-09-27 at 17 10 16" src="https://github.com/user-attachments/assets/400b22cf-7f23-4502-8d89-d1f9f739c32f" />

### 2. Velocity Tracking
- Tracks the **x and y velocities** estimated by the EKF against the true velocities.
<img width="885" height="567" alt="Screenshot 2025-09-27 at 17 10 23" src="https://github.com/user-attachments/assets/75b90aa7-6a1a-487b-9520-16c9904b851c" />

---

## 📊 Key Observations

- The EKF effectively **follows the circular path** despite measurement noise.  
- Velocity estimates capture the changing direction of motion.  
- Using a linear Kalman Filter would be **less accurate** for this nonlinear system.

---

## 🛠️ Future Improvements

- Extend to **3D object tracking**.  
- Introduce **sensor fusion** (e.g., combining GPS and IMU data).  
- Implement **unscented Kalman filter (UKF)** for improved nonlinear performance.  
- Visualize **estimation errors** over time for deeper analysis.
