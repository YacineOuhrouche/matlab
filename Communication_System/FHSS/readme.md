# 📡 Frequency Hopping Spread Spectrum (FHSS) Simulation in MATLAB  

## Overview 🚀  
This project demonstrates the simulation of a **Frequency Hopping Spread Spectrum (FHSS)** communication system in MATLAB.  
FHSS is a **spread spectrum modulation technique** where the **carrier frequency changes (hops)** according to a predefined hopping sequence.  


---

## Why Study FHSS? 📶  
- **Anti-Jamming:** Frequency hopping makes it difficult for narrowband interferers to disrupt communication.  
- **Security:** The hopping sequence acts as a "key," preventing unauthorized receivers from decoding.  
- **Multipath Resistance:** Reduces effects of fading in wireless channels.  
- **Practical Relevance:** FHSS is used in **Bluetooth**, **military radios**, and **satellite communications**.  

---

## System Specifications 📋  

| Parameter              | Value                          | Description                                |
|------------------------|--------------------------------|--------------------------------------------|
| Modulation Scheme      | BPSK  | Digital modulation for data symbols        |
| Hopping Frequencies    | 8 carriers  | Frequency pool for hopping                 |
| Hopping Pattern        | Pseudo-random sequence        | Defines carrier hopping order              |
| Bit Duration           | 0.1 s                         | Each bit is modulated within a hop period  |
| Sampling Frequency     | 2 kHz                         | Ensures smooth waveform representation     |

---

## Simulation Description 🛠️  
1. **Data Generation:**  
   - A binary message sequence (0s and 1s) is generated.  

2. **Digital Modulation:**  
   - Each bit is mapped to a baseband modulated signal  

3. **Hopping Sequence:**  
   - A **pseudo-random hopping sequence** is generated that selects the carrier frequency for each bit.  

4. **Carrier Hopping:**  
   - The baseband signal is upconverted to the carrier chosen by the hopping sequence.  
   - For bit `1`, a different carrier is chosen than for bit `0`.  

5. **FHSS Signal Transmission:**  
   - The final FHSS waveform is the concatenation of all hopped carriers carrying their respective modulated data.  


---

## Visualization 📊  
- **Message Bits:** Binary sequence used for modulation.  
- **Hopping Pattern:** Shows how frequency changes per bit.  
- **FHSS Signal:** The modulated waveform switching carriers.  
<img width="915" height="721" alt="Screenshot 2025-08-19 at 11 39 53" src="https://github.com/user-attachments/assets/71fc8cca-8bb0-426c-8463-8ce7546ec47b" />


---

## Future Enhancements 🔮  
- Implement **synchronization algorithms** at the receiver.  
- Add **MIMO-FHSS combination** for higher capacity and robustness.  
- Compare **FHSS vs DSSS (Direct Sequence Spread Spectrum)** performance.  
- Simulate BER performance under **jamming or interference**.  

---
