# 🔹 Binary Phase Shift Keying (BPSK) Simulation Project  

## Overview 🚀  
This project demonstrates the simulation of **Binary Phase Shift Keying (BPSK)**.  
BPSK is a **digital modulation technique** where the **phase of a carrier signal** is shifted according to the binary input data.  
The simulation generates a **digital message**, a **carrier signal**, and the resulting **BPSK waveform**, visualizing both the **message and modulated signal**.  

---

## Why Study BPSK? 📡  
- **Noise-Resistant Digital Modulation:** Phase changes make BPSK less sensitive to amplitude noise.  
- **Fundamental in Communications:** Widely used in digital radio, satellite, and modem systems.  
- **Visual Clarity:** Easy to observe phase changes corresponding to digital bits.  

---

## Project Specifications 📋  

| Parameter             | Value               | Description                              |
|-----------------------|-------------------|------------------------------------------|
| Sampling Frequency    | 1 kHz             | Ensures smooth waveform representation   |
| Bit Duration          | 0.1 s             | Duration of each digital bit             |
| Carrier Frequency     | 50 Hz             | Frequency of the carrier signal          |
| Amplitude             | 1                 | Carrier amplitude                        |
| Message Bits          | [1 0 1 1 0 0 1]  |  digital message                  |
| Phase Shift for 0     | 180°              | Phase shift applied when bit = 0         |
| Phase Shift for 1     | 0°                | No phase shift applied when bit = 1      |
| Time Duration         | 0.7 s             | Total simulation time for all bits       |

---

## Simulation Results 📊  
- **Message Signal:** Sequence of binary bits (`0` and `1`).  
- **Carrier Signal:** Pure sinusoidal wave at 50 Hz.  
- **BPSK Signal:** Carrier phase flips according to the message bits.  
- **Visual Observation:** Phase changes are clearly visible at bit transitions.  

<img width="866" height="541" alt="Screenshot 2025-08-18 at 17 08 51" src="https://github.com/user-attachments/assets/298b2689-22e6-4b29-a4c1-da779d518eab" />

---

## Future Enhancements 🔮  
- Implement **BPSK demodulation** to recover the original digital bits.  
- Explore **higher-order PSK** (e.g., QPSK, 8-PSK) for multi-bit symbols.  
 
