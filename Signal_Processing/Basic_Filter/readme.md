# 🎵  Filters in MATLAB

This project demonstrates the design and implementation of ** filters** using MATLAB, including **Low-pass, High-pass, Band-pass, Notch, and Multi-band FIR/IIR filters**. The goal is to analyze and visualize how these filters affect signals in both **time and frequency domains**, and explore **Bode plots** for frequency response insights.

---

## ⚙️ System Setup

- **Sampling Frequency (Fs)** → 1000 Hz  
- **Test Signal** → Sum of sinusoids at multiple frequencies (50 Hz, 150 Hz, 220 Hz, 350 Hz) + random noise.  
- **Signal Duration** → 1 second  

---

## 🎛️ Filters Implemented

<img width="895" height="561" alt="Screenshot 2025-09-26 at 14 08 47" src="https://github.com/user-attachments/assets/006b7bed-b9d8-48ee-97a6-682b8453125a" />

### 1. Low-pass Filter
- Allows signals **below 100 Hz** to pass and attenuates higher frequencies.  
- Implemented using **4th-order Butterworth IIR filter**.  
<img width="869" height="510" alt="Screenshot 2025-09-26 at 14 08 54" src="https://github.com/user-attachments/assets/aa86a0b1-5887-4645-8993-009b0e436e50" />


### 2. High-pass Filter
- Allows signals **above 200 Hz** to pass and attenuates lower frequencies.  
- Useful to remove low-frequency noise or DC offset.  
<img width="885" height="548" alt="Screenshot 2025-09-26 at 14 09 01" src="https://github.com/user-attachments/assets/83c60599-586e-4a32-89f3-cf2ca2ac9606" />

### 3. Band-pass Filter
- Passes signals in the **150–300 Hz** range and attenuates frequencies outside this band.  
- Combines low-pass and high-pass characteristics.  
<img width="852" height="547" alt="Screenshot 2025-09-26 at 14 09 10" src="https://github.com/user-attachments/assets/91367893-f118-42e8-89ae-aed0a16f350f" />

### 4. Notch (Band-stop) Filter
- Removes a narrow frequency band **240–260 Hz**, effectively suppressing unwanted tones.  
<img width="868" height="552" alt="Screenshot 2025-09-26 at 14 09 17" src="https://github.com/user-attachments/assets/4095a624-e162-4ccf-88f1-a6fd540dfad1" />

### 5. Multi-band Filter
- Combines **several passbands**: 50–100 Hz, 200–250 Hz, 350–400 Hz.  
- Implemented as **FIR filter using `firpm`** for precise control over multiple bands.  
<img width="864" height="542" alt="Screenshot 2025-09-26 at 14 09 23" src="https://github.com/user-attachments/assets/dc1b3e94-ede8-44e9-ae4d-7e618e335624" />

---


## 📊 Results & Observations

- **Low-pass Filter** → Smooths high-frequency noise, but some ripples appear near cutoff due to filter order.  
- **High-pass Filter** → Removes low-frequency noise effectively, retaining higher-frequency components.  
- **Band-pass Filter** → Isolates the desired frequency band while suppressing signals outside it.  
- **Notch Filter** → Successfully removes narrow unwanted frequencies without affecting other bands.  
- **Multi-band Filter** → Efficiently passes multiple bands; ideal for **audio equalization** or **signal separation**.  

✅ **Conclusion**: The filters behave as expected, and ** filter design** allows precise control over the frequency response. IIR filters are efficient for single-band filtering, while FIR filters are better for multi-band and linear-phase applications.

---


## 🛠️ Future Improvements

- Implement **real-time audio filtering** for music or speech.  
- Add **interactive multi-band equalizer** with adjustable gains.  
- Experiment with **adaptive filters (LMS, RLS)** for noise cancellation applications.  
- Compare **FIR vs IIR performance** for different filter orders and applications.  

---
