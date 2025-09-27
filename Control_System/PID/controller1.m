% PID Control of a DC Motor

clear; clc; close all;

%% Motor Parameter
J = 0.01;   % Rotor inertia 
B = 0.1;    % Viscous damping coefficient
R = 1;      % resistance 
L = 0.5;    % inductance [H]
K = 0.01;   % Motor torque constant

%% Transfer function
s = tf('s');
P_motor = K / ((L*s + R)*(J*s + B) + K^2);  

%% Controllers
% 1) Manual PID
Kp = 100; Ki = 200; Kd = 10;
C_manual = pid(Kp, Ki, Kd);

% 2) Autotune PID 
[C_auto, infoAuto] = pidtune(P_motor, 'PID');

% 3) Ziegler–Nichols Tuning 
Ku = 300;   % Ultimate gain 
Tu = 0.5;   % Oscillation period 

Kp_ZN = 0.6 * Ku;
Ki_ZN = 2*Kp_ZN / Tu;
Kd_ZN = Kp_ZN * Tu / 8;

C_ZN = pid(Kp_ZN, Ki_ZN, Kd_ZN);

%% Closed-loop systems 
CL_manual = feedback(C_manual * P_motor, 1);
CL_auto   = feedback(C_auto   * P_motor, 1);
CL_ZN     = feedback(C_ZN     * P_motor, 1);

%% Time Response 
tEnd = 2;         
t    = 0:0.01:tEnd;

ref_speed = 50;   

[y_manual, t] = step(CL_manual*ref_speed, t);
[y_auto,   ~] = step(CL_auto*ref_speed,   t);
[y_ZN,     ~] = step(CL_ZN*ref_speed,     t);

figure;
plot(t, y_manual, 'b-', 'LineWidth', 2); hold on;
plot(t, y_auto,   'r--', 'LineWidth', 2);
plot(t, y_ZN,     'g-.', 'LineWidth', 2);
yline(ref_speed, 'k--', 'LineWidth', 1.5); 
grid on; xlabel('Time (s)'); ylabel('Angular Speed (rad/s)');
title('DC Motor: Step Response with PID Controllers');
legend('Manual PID', 'Auto-tuned PID', 'Ziegler–Nichols PID', 'Reference');

%% ✅ Combined Root Locus
figure;
hold on; grid on;
rlocus(C_manual * P_motor);  
rlocus(C_auto * P_motor);    
rlocus(C_ZN * P_motor);      
title('Root Locus Comparison: PID Controllers + DC Motor');
legend('Manual PID', 'Auto-tuned PID', 'Ziegler–Nichols PID');

%% ✅ Combined Bode Plots
figure;
margin(C_manual*P_motor); hold on;
margin(C_auto*P_motor);
margin(C_ZN*P_motor);
title('Bode Plot Comparison: PID Controllers + DC Motor');
legend('Manual PID', 'Auto-tuned PID', 'Ziegler–Nichols PID');

%% Performance Metrics
S_manual = stepinfo(CL_manual);
S_auto   = stepinfo(CL_auto);
S_ZN     = stepinfo(CL_ZN);

disp('--- Step Performance (Manual PID) ---');
disp(S_manual);

disp('--- Step Performance (Auto-tuned PID) ---');
disp(S_auto);

disp('--- Step Performance (Ziegler–Nichols PID) ---');
disp(S_ZN);
