# 🔲 Quadrature Amplitude Modulation (QAM) Simulation Project  

## Overview 🚀  
This project demonstrates the simulation of **Quadrature Amplitude Modulation (QAM)**, specifically **16-QAM**.  
QAM is a **digital modulation technique** that conveys data by varying both the **amplitude** and **phase** of a carrier signal.  
The simulation generates a **digital message**, modulates it into QAM symbols, adds noise, and visualizes both the **constellation diagram** and **received noisy signal**.  

---

## Why Study QAM? 📡  
- **High Spectral Efficiency:** Transmits multiple bits per symbol (16-QAM = 4 bits per symbol).  
- **Widely Used:** Basis of modern digital communication systems (Wi-Fi, LTE, DVB).  
- **Practical Learning:** Visualizes symbol mapping and constellation distortion due to noise.  

---

## Project Specifications 📋  

| Parameter             | Value             | Description                              |
|-----------------------|------------------|------------------------------------------|
| Modulation Scheme     | 16-QAM           | 4 bits per symbol, 16 constellation points |
| Symbols Transmitted   | 10,000           | Random data symbols                      |
| Carrier Amplitude     | Normalized       | Unit average power                       |
| Channel               | AWGN             | Additive White Gaussian Noise             |
| Eb/No                 | 10 dB            | Signal-to-noise ratio per bit            |

---

## Simulation Results 📊  
- **Transmitted Constellation:** Ideal 16-QAM constellation points in the I/Q plane.  
- **Received Constellation:** Noisy points scattered around the ideal grid (distortion visible at lower SNR).  
- **Bit Error Rate (BER):** Calculated by comparing transmitted and received symbols.  
<img width="543" height="558" alt="Screenshot 2025-08-18 at 17 33 23" src="https://github.com/user-attachments/assets/91d6e0e0-e8f3-4465-8acc-8dae8d9a8a91" />
<img width="547" height="563" alt="Screenshot 2025-08-18 at 17 33 28" src="https://github.com/user-attachments/assets/c5ec65b9-e320-4fd5-bf74-a7ec8aefd10a" />


---

## Future Enhancements 🔮  
- Implement **QAM demodulation** with Gray coding for efficient bit mapping.  
- Explore **higher-order QAM** (e.g., 64-QAM, 256-QAM) to analyze trade-offs between data rate and noise robustness.  
