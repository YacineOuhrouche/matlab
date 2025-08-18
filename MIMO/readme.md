# 📡 MIMO (Multiple-Input Multiple-Output) System Simulation in MATLAB  

## Overview 🚀  
This project demonstrates the simulation of a **2x2 MIMO system** using **16-QAM modulation** in MATLAB.  
The simulation evaluates the **Bit Error Rate (BER)** performance of two linear detection techniques:  
- **Zero Forcing (ZF)**  
- **Minimum Mean Square Error (MMSE)**  

---

## Why Study MIMO Systems? 📶  
- **High Capacity:** Increases data throughput without extra bandwidth.  
- **Robustness:** Improves reliability under fading channels.  
- **Foundation for Modern Wireless:** Essential for **4G LTE** and **5G NR**.  
- **Performance Insights:** BER curves reveal tradeoffs between detection methods.  

---

## System Specifications 📋  

| Parameter            | Value                | Description                                |
|----------------------|----------------------|--------------------------------------------|
| Modulation Scheme    | 16-QAM              | 4 bits per symbol                          |
| MIMO Configuration   | 2×2                 | 2 transmit, 2 receive antennas             |
| Channel Model        | Rayleigh fading     | Models multipath wireless propagation      |
| Detection Methods    | ZF & MMSE           | Linear equalizers compared                 |
| SNR Range            | 0 – 30 dB (step 5) | Sweep to measure BER performance           |
| Symbols Simulated    | 5000                | Number of transmitted symbols per SNR      |

---

## Simulation Description 🛠️  
- Random data bits are generated and **16-QAM modulated**.  
- Data passes through a **2×2 Rayleigh fading channel**.  
- **AWGN noise** is added to simulate realistic wireless conditions.  
- At the receiver:  
  - **Zero Forcing (ZF):** Uses pseudoinverse of channel matrix for detection.  
  - **MMSE:** Improves on ZF by considering noise power in the equalizer.  
- Demodulation converts received symbols back to bits.  
- **BER is calculated** by comparing transmitted and received bits.  

---

## Future Enhancements 🔮  
- Extend to **4x4 MIMO** for higher capacity gains.  
- Implement **Maximum Likelihood Detection (MLD)** for optimal performance.  
- Simulate **OFDM-MIMO** (basis of 4G/5G systems).  

---
