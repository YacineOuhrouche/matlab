# 🤖 2-Link Planar Manipulator — Forward Kinematics Simulation (MATLAB)

This project demonstrates the **forward kinematics (FK)** of a **2-link planar robotic manipulator** using **MATLAB scripting**.  
It computes the **end-effector position** based on the given **joint angles**, visualizes the **link motion**, and traces the **reachable workspace** of the robot arm.  

This project lays the foundation for advanced robotics topics such as **inverse kinematics**, **trajectory generation**, and **path planning**.

---

## ⚙️ System Overview

A **2-link planar manipulator** consists of two rigid links connected by **revolute (rotational)** joints.  
Each joint can rotate, changing the configuration of the arm in the plane.  
The goal of **forward kinematics** is to determine the **position and orientation** of the end-effector given the joint angles.

---

### 🔩 Parameters

| Symbol | Description | Example Value |
|---------|--------------|---------------|
| `L₁` | Length of the first link | 1.0 m |
| `L₂` | Length of the second link | 0.7 m |
| `θ₁` | Joint 1 rotation angle | -90° → +90° |
| `θ₂` | Joint 2 rotation angle | -90° → +90° |

The **forward kinematics equations** are derived using trigonometry:

$$
\begin{align}
x &= L_1 \cos(\theta_1) + L_2 \cos(\theta_1 + \theta_2) \\
y &= L_1 \sin(\theta_1) + L_2 \sin(\theta_1 + \theta_2)
\end{align}
$$

These equations map the robot’s **joint space** (angles) to its **Cartesian space** (end-effector position).

---

## 🧠 Theoretical Background

### 🔹 1. Robotics Fundamentals

In robotics, a manipulator is modeled as a series of **links** (rigid bodies) connected by **joints** (which allow relative motion).  
The robot’s configuration is defined by its **joint variables**  either **angles** (for revolute joints) or **displacements** (for prismatic joints).  

We typically work in two coordinate spaces:
- **Joint Space** → defined by joint variables $(\theta_1, \theta_2, ...)$  
- **Cartesian Space** → defined by the end-effector position and orientation $(x, y, z)$  

**Kinematics** establishes the relationship between these two spaces:
- **Forward Kinematics (FK)** → joint angles → end-effector position  
- **Inverse Kinematics (IK)** → end-effector position → joint angles  

---

### 🔹 2. Matrix Transformations in Robotics

The motion of robotic arms is elegantly expressed using **homogeneous transformation matrices**, which unify **rotation** and **translation** in a single matrix form.  

A general **3D homogeneous transformation** is given by:

$$
T =
\begin{bmatrix}
R & P \\
0 & 1
\end{bmatrix}
$$

Where:  
- **R** → 3×3 rotation matrix (orientation)  
- **P** → 3×1 position vector (translation)  

For a **2D planar manipulator**, this simplifies to:

$$
T =
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta &  \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
$$

By multiplying transformations for each joint, the overall motion of the manipulator can be obtained:

$$
T = T_1^0 \times T_2^1
$$

Matrix transformations are fundamental because:
- They allow compact representation of rotation and translation.  
- Multiple link transformations can be **chained together** through matrix multiplication.  
- The same principles extend naturally to **3D robotic arms**.

---

### 🔹 3. Denavit–Hartenberg (DH) Convention

The **Denavit–Hartenberg (DH)** convention standardizes how robotic link geometry is described using four parameters per link:

| Parameter | Symbol | Meaning |
|------------|---------|----------|
| Link Length | $a_i$ | Distance between $Z_{i-1}$ and $Z_i$ along $X_i$ |
| Link Twist | $α_i$ | Angle between $Z_{i-1}$ and $Z_i$ about $X_i$ |
| Link Offset | $d_i$ | Distance between $X_{i-1}$ and $X_i$ along $Z_{i-1}$ |
| Joint Angle | $θ_i$ | Rotation about $Z_{i-1}$ |

Each joint transformation can be represented as:

$$
T_i^{i-1} =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i &  \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The **overall transformation** from the base to the end-effector is obtained by multiplying all joint transformations:

$$
T = T_1^0 \, T_2^1
$$

The **top-right column** of $T$ gives the **end-effector position** in Cartesian coordinates.

---

## 🧩 MATLAB Implementation Overview

The MATLAB script:
- Defines **link lengths** and **joint angle ranges**.  
- Constructs **transformation matrices** for each link.  
- Computes **end-effector coordinates** by matrix multiplication.  
- Iterates over all joint angle combinations to generate the **workspace map**.  
- Visualizes:
  - A **3D workspace surface**  
  - A **2D animated arm** showing real-time motion  

During animation:
- **Blue link** → Link 1 rotating at the base  
- **Green link** → Link 2 following Link 1’s motion  
- **Red dot** → End-effector  
- **Yellow trace** → Path of the end-effector  

---


## 📊 Results & Observations

- The reachable workspace forms a **crescent (arc-shaped)** region.  
- Link lengths determine the **maximum reachable area**.  
- Visualization highlights how **joint coupling** affects motion.  
- Confirms theoretical FK results through simulation.
<img width="880" height="562" alt="Screenshot 2025-11-01 at 10 57 06" src="https://github.com/user-attachments/assets/2a8957d4-3381-4b14-9b57-61592236983f" />



https://github.com/user-attachments/assets/a00018eb-e262-4d62-9efd-4dfcae753399


---

## ✅ Key Takeaways

- **Matrix transformations** elegantly capture robotic motion.  
- **DH convention** provides a consistent modeling framework.  
- **Forward Kinematics** is the basis for inverse kinematics and control.  
- MATLAB’s matrix computation and plotting tools make it ideal for robot simulation.

---

## 🛠️ Future Work

- Implement **Inverse Kinematics (IK)** to target specific end-effector points.  
- Add **trajectory generation** and **path planning**.  
- Extend to **3-link** or **3D manipulators**.  
- Integrate **PID or torque control** for motion regulation.

---


