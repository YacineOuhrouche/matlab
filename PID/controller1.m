% ============================================================
% PID Control of a DC Motor
% Transfer function: W(s)/Va(s) = K / [(L*s+R)(J*s+B) + K^2]
% Includes: manual PID, auto-tuned PID, Ziegler–Nichols PID,
%           step responses, root locus, bode plots, and metrics
% ============================================================

clear; clc; close all;

%% --------------------- Motor Parameters ---------------------
J = 0.01;   % Rotor inertia [kg*m^2]
B = 0.1;    % Viscous damping coefficient [N*m*s]
R = 1;      % Armature resistance [Ohm]
L = 0.5;    % Armature inductance [H]
K = 0.01;   % Motor torque & back-emf constant

%% --------------------- Transfer Function --------------------
s = tf('s');
P_motor = K / ((L*s + R)*(J*s + B) + K^2);   % input: Va(s), output: omega(s)

% If you want position(theta), uncomment:
% P_motor = P_motor / s;

%% --------------------- Controllers --------------------------
% 1) Manual PID
Kp = 100; Ki = 200; Kd = 10;
C_manual = pid(Kp, Ki, Kd);

% 2) Auto-tuned PID (MATLAB)
[C_auto, infoAuto] = pidtune(P_motor, 'PID');

% 3) Ziegler–Nichols Tuning (example Ku & Tu, adjust experimentally)
Ku = 300;   % Ultimate gain (found via oscillation test)
Tu = 0.5;   % Oscillation period [s]

Kp_ZN = 0.6 * Ku;
Ki_ZN = 2*Kp_ZN / Tu;
Kd_ZN = Kp_ZN * Tu / 8;

C_ZN = pid(Kp_ZN, Ki_ZN, Kd_ZN);

%% --------------------- Closed-Loop Systems ------------------
CL_manual = feedback(C_manual * P_motor, 1);
CL_auto   = feedback(C_auto   * P_motor, 1);
CL_ZN     = feedback(C_ZN     * P_motor, 1);

%% --------------------- Time Response ------------------------
tEnd = 2;         
t    = 0:0.01:tEnd;

ref_speed = 1;   % reference (rad/s) → change this to test other refs (e.g. 50, 100)

[y_manual, t] = step(CL_manual*ref_speed, t);
[y_auto,   ~] = step(CL_auto*ref_speed,   t);
[y_ZN,     ~] = step(CL_ZN*ref_speed,     t);

figure;
plot(t, y_manual, 'b-', 'LineWidth', 2); hold on;
plot(t, y_auto,   'r--', 'LineWidth', 2);
plot(t, y_ZN,     'g-.', 'LineWidth', 2);
yline(ref_speed, 'k--', 'LineWidth', 1.5); % reference line
grid on; xlabel('Time (s)'); ylabel('Angular Speed (rad/s)');
title('DC Motor: Step Response with PID Controllers');
legend('Manual PID', 'Auto-tuned PID', 'Ziegler–Nichols PID', 'Reference');

%% --------------------- Root Locus ----------------------------
figure;
rlocus(C_manual * P_motor);
title('Root Locus: Manual PID + DC Motor'); grid on;

figure;
rlocus(C_auto * P_motor);
title('Root Locus: Auto-tuned PID + DC Motor'); grid on;

figure;
rlocus(C_ZN * P_motor);
title('Root Locus: Ziegler–Nichols PID + DC Motor'); grid on;

%% --------------------- Bode Plots ----------------------------
figure;
margin(C_manual*P_motor);
title('Bode Plot: Manual PID + DC Motor');

figure;
margin(C_auto*P_motor);
title('Bode Plot: Auto-tuned PID + DC Motor');

figure;
margin(C_ZN*P_motor);
title('Bode Plot: Ziegler–Nichols PID + DC Motor');

%% --------------------- Performance Metrics ------------------
S_manual = stepinfo(CL_manual);
S_auto   = stepinfo(CL_auto);
S_ZN     = stepinfo(CL_ZN);

disp('--- Step Performance (Manual PID) ---');
disp(S_manual);

disp('--- Step Performance (Auto-tuned PID) ---');
disp(S_auto);

disp('--- Step Performance (Ziegler–Nichols PID) ---');
disp(S_ZN);
