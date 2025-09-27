clc; clear; close all;

%% Parameters
fs = 1000;              % Sampling frequency
t = 0:1/fs:2;           % Time vector (2 seconds)

% Clean signal (speech-like sine wave)
f_signal = 50;          % Frequency of clean signal
s = sin(2*pi*f_signal*t);

% Noise (make it stronger and more random)
noise = 2*sin(2*pi*60*t) + 1*randn(size(t));

% Primary input (signal + noise)
d = s + noise;

% Reference noise input (correlated noise for cancellation)
x = 2*sin(2*pi*60*t) + 1*randn(size(t));

%% LMS Adaptive Filter
M = 32;                % Filter order
mu = 0.01;             % Step size (learning rate)
lms = dsp.LMSFilter('Length', M, 'StepSize', mu);

[y, e] = lms(x', d');   % y = estimated noise, e = error (cleaned signal)

%% Plot results
figure;
subplot(3,1,1);
plot(t, d); title('Noisy Input Signal (Very Noisy)'); xlabel('Time (s)'); ylabel('Amplitude');

subplot(3,1,2);
plot(t, e); title('Output After LMS Noise Cancellation'); xlabel('Time (s)'); ylabel('Amplitude');

subplot(3,1,3);
plot(t, s); title('Original Clean Signal (Reference)'); xlabel('Time (s)'); ylabel('Amplitude');

%% Compare in Frequency Domain
figure;
pwelch(d, [], [], [], fs); hold on;
pwelch(e, [], [], [], fs);
legend('Noisy Signal','Filtered Signal');
title('Power Spectral Density Comparison');
