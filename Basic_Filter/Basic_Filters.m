clc; clear; close all;

%% Sampling settings
Fs = 1000;               
t = 0:1/Fs:1;            

%% Test signal (sum of sinusoids + noise)
x = sin(2*pi*50*t) + sin(2*pi*150*t) + sin(2*pi*220*t) + sin(2*pi*350*t) + 0.5*randn(size(t));

%% Frequency vector 
f = (0:length(x)-1)*Fs/length(x);

%% Plot original signal (Figure 1)
figure('Name','Original Signal','NumberTitle','off');
subplot(2,1,1);
plot(t,x,'k'); 
title('Original Signal (Time Domain)');
xlabel('Time (s)'); ylabel('Amplitude');

subplot(2,1,2);
X = abs(fft(x));
plot(f(1:floor(end/2)), X(1:floor(end/2)),'k');
title('Original Signal Spectrum');
xlabel('Frequency (Hz)'); ylabel('|X(f)|');

%% Filter parameters (normalized by Nyquist frequency = Fs/2)
low_cutoff   = 100/(Fs/2);
high_cutoff  = 200/(Fs/2);
band_cutoff  = [150 300]/(Fs/2);
notch_cutoff = [240 260]/(Fs/2);
multi_cutoff = [50 100 200 250 350 400]/(Fs/2);

%% IIR Filter Designs (Butterworth)
order = 4;
[b_low,a_low]     = butter(order, low_cutoff, 'low');
[b_high,a_high]   = butter(order, high_cutoff, 'high');
[b_band,a_band]   = butter(order, band_cutoff, 'bandpass');
[b_notch,a_notch] = butter(order, notch_cutoff, 'stop');

%% FIR Multi-Band Filter Design using firpm
order_fir = 200;
f_bands = [0 50 100 200 250 350 400 500]/(Fs/2);
a_bands = [1 1 0 0 1 1 0 0];
b_multi = firpm(order_fir, f_bands, a_bands);

%% Apply filters
y_low   = filter(b_low, a_low, x);
y_high  = filter(b_high, a_high, x);
y_band  = filter(b_band, a_band, x);
y_notch = filter(b_notch, a_notch, x);
y_multi = filter(b_multi, 1, x);

%% List of signals and filters
signals = {y_low, y_high, y_band, y_notch, y_multi};
titles  = {'Low-pass Filtered (≤100 Hz)', ...
           'High-pass Filtered (≥200 Hz)', ...
           'Band-pass Filtered (150–300 Hz)', ...
           'Notch Filtered (removes 240–260 Hz)', ...
           'Multi-band Filtered (50-100,200-250,350-400 Hz)'};
b = {b_low, b_high, b_band, b_notch, b_multi};
a = {a_low, a_high, a_band, a_notch, 1};  % FIR: a=1

%% Plot filtered signals (Figure 2+)
for k = 1:length(signals)
    figure('Name', titles{k}, 'NumberTitle','off');
    
    % --- Time domain ---
    subplot(3,1,1);
    plot(t, signals{k}, 'r');
    title([titles{k} ' - Time Domain']);
    xlabel('Time (s)'); ylabel('Amplitude');

    % --- Frequency spectrum ---
    subplot(3,1,2);
    Y = abs(fft(signals{k}));
    plot(f(1:floor(end/2)), Y(1:floor(end/2)), 'r');
    title([titles{k} ' - Frequency Spectrum']);
    xlabel('Frequency (Hz)'); ylabel('|Y(f)|');

    % --- Bode plot ---
    subplot(3,1,3);
    [h,freq] = freqz(b{k}, a{k}, 1024, Fs);
    yyaxis left
    plot(freq, 20*log10(abs(h)), 'b', 'LineWidth', 1.2);
    ylabel('Magnitude (dB)');
    yyaxis right
    plot(freq, angle(h)*180/pi, 'r', 'LineWidth', 1.2);
    ylabel('Phase (deg)');
    xlabel('Frequency (Hz)');
    title([titles{k} ' - Bode Plot']);
    legend('Magnitude','Phase');
end
