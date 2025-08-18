# 📶 Frequency Shift Keying (FSK) Simulation Project  

## Overview 🚀  
This project demonstrates the simulation of **Frequency Shift Keying (FSK)**.  
FSK is a **digital modulation technique** where the **frequency of a carrier signal** changes according to the binary input data.  
The simulation generates a **digital message**, two **carrier frequencies** for `0` and `1`, and the resulting **FSK waveform**, visualizing both the **message and modulated signal**.  

---

## Why Study FSK? 📡  
- **Noise-Resistant Digital Modulation:** Frequency changes make FSK less susceptible to amplitude noise.  
- **Fundamental in Communications:** Basis for many modem and digital radio systems.  
- **Visual Clarity:** Easy to observe frequency changes corresponding to digital bits.  

---

## Project Specifications 📋  

| Parameter             | Value               | Description                              |
|-----------------------|-------------------|------------------------------------------|
| Sampling Frequency    | 2 kHz             | Ensures smooth waveform representation   |
| Bit Duration          | 0.1 s             | Duration of each digital bit             |
| Carrier Frequency for 0 | 50 Hz           | Frequency representing bit = 0          |
| Carrier Frequency for 1 | 100 Hz          | Frequency representing bit = 1          |
| Amplitude             | 1                 | Carrier amplitude                        |
| Message Bits          | [1 0 1 1 0 0 1]  | Example digital message                  |
| Time Duration         | 0.7 s             | Total simulation time for all bits       |

---

## Simulation Results 📊  
- **Message Signal:** Sequence of binary bits (`0` and `1`).  
- **Carrier Signals:** Two reference carriers at 50 Hz and 100 Hz.  
- **FSK Signal:** Carrier frequency switches according to the message bits.  
- **Visual Observation:** Low-frequency segments correspond to bit 0, high-frequency segments correspond to bit 1.  
<img width="873" height="544" alt="Screenshot 2025-08-18 at 16 56 52" src="https://github.com/user-attachments/assets/631dc02c-9cac-441e-baf3-5899256f5c72" />


---

## Future Enhancements 🔮  
- Implement **FSK demodulation** to recover the original digital bits.    
- Explore **multi-level FSK** (more than two frequencies for multiple-bit symbols).  
