% FM Modulation 
fs = 1e4;         % Sampling frequency 
t = 0:1/fs:0.1;   % Time vector 

% Message signal (
fm = 50;          % Message frequency 
Am = 1;           %  amplitude
m = Am*cos(2*pi*fm*t);

% Carrier signal
fc = 500;         % Carrier frequency 
Ac = 1;           %  amplitude

% Frequency deviation 
kf = 200;          

% FM signal
y = Ac * cos(2*pi*fc*t + (kf/fm)*sin(2*pi*fm*t));

% --- Plots ---
figure;

subplot(3,1,1);
plot(t, m, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Amplitude');
title('Message Signal (m(t))');
grid on;

subplot(3,1,2);
plot(t, cos(2*pi*fc*t), 'r', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Amplitude');
title('Carrier Signal (c(t))');
grid on;

subplot(3,1,3);
plot(t, y, 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Amplitude');
title('FM Signal ');
grid on;

