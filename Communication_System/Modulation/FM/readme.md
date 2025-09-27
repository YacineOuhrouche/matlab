# 📻 Frequency Modulation (FM) Simulation Project  

## Overview 🚀  
This project demonstrates the simulation of **Frequency Modulation (FM)**.  
It generates a **message signal**, a **carrier**, and the resulting **FM waveform**, and visualizes both the **time-domain signals**.  

---

## Why Study FM? 📶  
- **Robust to Noise:** FM signals have better noise immunity compared to AM.  
- **Higher Fidelity:** Widely used in high-quality audio broadcasting (FM radio).  
- **Fundamental Concept:** Basis for many modern communication systems.   

---

## Project Specifications 📋  

| Parameter             | Value          | Description                              |
|-----------------------|----------------|------------------------------------------|
| Sampling Frequency    | 10 kHz         | Ensures smooth waveform representation   |
| Message Frequency     | 50 Hz          | Low-frequency baseband signal            |
| Carrier Frequency     | 500 Hz         | Frequency of the unmodulated carrier     |
| Message Amplitude     | 1              | Amplitude of message signal              |
| Carrier Amplitude     | 1              | Amplitude of carrier signal              |
| Frequency Deviation   | 200 Hz         | Determines modulation index              |
| Time Duration         | 0.1 s          | Simulation time                          |

---

## Simulation Results 📊  
- **Message Signal:** A low-frequency (50 Hz) cosine wave.  
- **Carrier Signal:** A pure 500 Hz cosine wave.  
- **FM Signal:** Carrier frequency varies with message amplitude (compressed & stretched cycles visible).  
<img width="852" height="532" alt="Screenshot 2025-08-18 at 16 14 42" src="https://github.com/user-attachments/assets/6d8402c1-e73e-4ea2-83d3-3e983e63e2a4" />


---

## Future Enhancements 🔮  
- Implement **FM demodulation** to recover the message signal.  
- Explore **different frequency deviations** to study bandwidth effects.  
- Compare **FM vs AM** under noisy conditions.   
