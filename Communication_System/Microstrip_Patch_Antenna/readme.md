# 📡 Microstrip Patch Antenna (Strip-Fed Slot) Project  

## Overview 🚀  
This project demonstrates the design and simulation of a **strip-fed slot microstrip patch antenna** using **MATLAB Antenna Toolbox**.  
The antenna is designed on an **FR4 PCB substrate** to operate near **1.94 GHz**, and includes custom slot shapes to modify bandwidth and improve matching.  

---

## Why Use a Microstrip Patch Antenna? 📶  
- **Low Profile:** Ideal for compact, portable devices.  
- **Lightweight & Inexpensive:** Can be fabricated on standard PCB material.  
- **Customizable:** Shape, size, and feed can be tuned for specific frequencies.  
- **Widely Used:** In Wi-Fi, GPS, radar, and mobile communication systems.  

---

## Antenna Specifications 📋  

| Parameter              | Value            | Description                              |
|------------------------|------------------|------------------------------------------|
| Substrate Material     | FR4              | Common PCB dielectric                    |
| Relative Permittivity  | 4.4              | Electrical property of FR4               |
| PCB Thickness          | 1.6 mm           | Dielectric thickness                     |
| PCB Size               | 152.4 × 101.6 mm | Overall board dimensions                 |
| Feed Type              | Strip-fed slot   | Edge-fed with slot structure             |
| Operating Frequency    | ~1.94 GHz        | Resonant frequency                       |
| Ground Plane           | Full rectangular | Covers entire bottom PCB layer           |

---

## Design Description 🛠️  
- The **top layer** contains the patch and custom slot geometry.  
- The **bottom layer** is a full ground plane.  
- The **substrate** is FR4 dielectric between the patch and ground.  
- A **strip-fed slot** feed is used to excite the antenna.  
- Slots are added to the patch to adjust the current path, tuning bandwidth and frequency.  

---

## Simulation Results 📊  
- **Radiation Pattern:** Shows the directional gain at 1.94 GHz.  
- **Input Impedance Plot:** Displays impedance variation across the frequency range.  
- **S11 Return Loss:** Indicates how well the antenna is matched to 50 Ω.  
- **Antenna Geometry:** Visual representation of the patch, slots, and ground plane.  
<img width="896" height="529" alt="Screenshot 2025-08-15 at 20 41 20" src="https://github.com/user-attachments/assets/c42bc526-18b6-409c-998f-e5dbf7bebeff" />

<img width="864" height="522" alt="Screenshot 2025-08-15 at 20 41 30" src="https://github.com/user-attachments/assets/94a1c7ad-ba7a-4a38-a98d-c954ed936205" />


---

##

## Future Enhancements 🔮  
- Optimize slot dimensions for improved bandwidth.  
- Implement microstrip line feed for different matching characteristics.  
- Use a **low-loss substrate** for higher efficiency at GHz frequencies.  
- Fabricate and test the design to compare with simulated results.  

---

