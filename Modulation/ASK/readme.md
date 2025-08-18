# 📡 Amplitude Shift Keying (ASK) Simulation Project  

## Overview 🚀  
This project demonstrates the simulation of **Amplitude Shift Keying (ASK)**.  
ASK is a **digital modulation scheme** where the **amplitude of a carrier signal** is varied according to the binary input data.  
The simulation generates a **digital message**, a **carrier signal**, and the resulting **ASK waveform**, visualizing both the **message and modulated signal**.  

---

## Why Study ASK? 📶  
- **Fundamental Digital Modulation:** Basis for many digital communication systems.  
- **Simple Implementation:** Easy to modulate and demodulate.  
- **Visual Clarity:** Directly shows amplitude changes corresponding to digital data.  

---

## Project Specifications 📋  

| Parameter             | Value           | Description                              |
|-----------------------|----------------|------------------------------------------|
| Sampling Frequency    | 1 kHz          | Ensures smooth waveform representation   |
| Bit Duration          | 0.1 s          | Duration of each digital bit             |
| Carrier Frequency     | 50 Hz          | Frequency of the unmodulated carrier     |
| Amplitude for Bit 1   | 1              | Carrier amplitude when bit = 1           |
| Amplitude for Bit 0   | 0              | Carrier amplitude when bit = 0           |
| Message Bits          | [1 0 1 1 0 0 1]| digital message                  |
| Time Duration         | 0.7 s          | Total simulation time for all bits       |

---

## Simulation Results 📊  
- **Message Signal:** Sequence of binary bits (`0` and `1`).  
- **Carrier Signal:** Pure sinusoidal wave at 50 Hz.  
- **ASK Signal:** Carrier amplitude switches according to the message bits.  
- **Visual Observation:** “On” periods correspond to bit 1 and “off” periods correspond to bit 0.  

  <img width="875" height="532" alt="Screenshot 2025-08-18 at 16 44 35" src="https://github.com/user-attachments/assets/805f82b5-8661-4c7a-9931-81f0724a5437" />


---

## Future Enhancements 🔮  
- Implement **ASK demodulation** to recover the original digital bits.  
- Explore **multi-level ASK** (higher-order amplitude modulation).  

