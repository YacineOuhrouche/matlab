% DC Motor PID Control - Full Analysis + Nonlinear Sim
% - Plant: armature-controlled DC motor
% - Controllers: Manual PID and Auto-tuned PID (pidtune)
% - Analyses: Step, Root Locus, Bode/Margins
% - Nonlinear Sim: actuator saturation, anti-windup, load disturbance, noise
% - Option: Position control view (theta output)

clear; clc; close all;

%% --------------------- Motor Parameters ---------------------
J = 0.01;    % Rotor inertia [kg*m^2]
B = 0.10;    % Viscous friction [N*m*s]
R = 1.00;    % Armature resistance [Ohm]
L = 0.50;    % Armature inductance [H]
K = 0.01;    % Torque constant (= back-emf const) [N*m/A = V*s/rad]

% For reference / scaling
Vmax = 12;             % Actuator (drive) voltage limit [V] for nonlinear sim
ref_speed = 1.0;       % Speed step command [rad/s]
ref_pos   = 1.0;       % Position step command [rad] (used if you plot position)

%% --------------------- Linear Plant Models -------------------
s = tf('s');
% Voltage -> Speed transfer function  (omega/Va)
P_motor = K / ((L*s + R)*(J*s + B) + K^2);        % omega(s)/Va(s)
% Voltage -> Position transfer function (theta/Va)
P_pos   = P_motor / s;                             % position view (optional)

%% --------------------- Controllers --------------------------
% 1) Manual PID (tweak these)
Kp = 100;
Ki = 200;
Kd = 10;
Tf = 1e-3;                           % derivative filter time constant (Kd*s/(Tf*s+1))
C_manual = pid(Kp, Ki, Kd, Tf);

% 2) Auto-tuned PID (Z-N style starting point via pidtune)
%    You can bias aggressiveness by target bandwidth (wb ~ 1/T) if desired
[C_auto, infoAuto] = pidtune(P_motor, 'PID');     % auto tuned on speed plant

%% --------------------- Closed-Loop (Linear) ------------------
CL_speed_manual = feedback(C_manual*P_motor, 1);
CL_speed_auto   = feedback(C_auto  *P_motor, 1);

% Optional: position closed-loop (for reference)
CL_pos_manual = feedback(C_manual*P_pos, 1);
CL_pos_auto   = feedback(C_auto  *P_pos, 1);

%% --------------------- Time Response (Tracking) --------------
tEnd = 2;                    % motors are pretty quick with these params
t    = 0:0.001:tEnd;

y_manual = step(ref_speed*CL_speed_manual, t);
y_auto   = step(ref_speed*CL_speed_auto,   t);

figure;
plot(t, y_manual, 'LineWidth', 2); hold on;
plot(t, y_auto,   'LineWidth', 2);
yline(ref_speed, '--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Speed \omega (rad/s)');
title('DC Motor: Step Tracking (Speed) — Manual vs Auto-tuned PID');
legend('Manual PID','Auto-tuned PID','Reference','Location','best');

% (Optional) position view
% figure;
% step(ref_pos*CL_pos_manual, t); hold on; step(ref_pos*CL_pos_auto, t);
% grid on; xlabel('Time (s)'); ylabel('\theta (rad)');
% title('DC Motor: Step Tracking (Position)');
% legend('Manual PID (pos)','Auto PID (pos)','Location','best');

%% --------------------- Root Locus ----------------------------
figure;
rlocus(C_manual*P_motor);
grid on; title('Root Locus: Manual PID + DC Motor');

figure;
rlocus(C_auto*P_motor);
grid on; title('Root Locus: Auto-tuned PID + DC Motor');

%% --------------------- Frequency-Domain Margins --------------
figure;
margin(C_manual*P_motor); grid on;
title('Bode + Stability Margins: Manual PID');

figure;
margin(C_auto*P_motor); grid on;
title('Bode + Stability Margins: Auto-tuned PID');

%% --------------------- Performance Metrics -------------------
S_manual = stepinfo(ref_speed*CL_speed_manual, t);
S_auto   = stepinfo(ref_speed*CL_speed_auto,   t);

disp('--- Step Performance (Speed) ---');
disp('Manual PID:'); disp(S_manual);
disp('Auto-tuned PID:'); disp(S_auto);

%% =============================================================
%           NONLINEAR DISCRETE-TIME SIM (REALISM)
% - Actuator saturation (±Vmax)
% - Anti-windup (integrator clamping)
% - Measurement noise on speed
% - Load disturbance torque step
% State model used (continuous):
%   di/dt = -(R/L)i - (K/L)omega + (1/L)Va
%   dω/dt =  (K/J)i - (B/J)omega - (1/J)Tl
% We discretize for simulation and implement a discrete PID with clamped I.
% =============================================================

% Build continuous state-space (inputs: Va, Tl; outputs: omega)
A = [ -R/L   -K/L;
       K/J   -B/J ];
B_ss = [ 1/L   0;    % input 1: Va, input 2: Tl
         0    -1/J ];
C_ss = [0 1];        % output: omega
D_ss = [0 0];

sysc = ss(A,B_ss,C_ss,D_ss);

% Discretize
Ts   = 1e-3;                   % sim step [s]
sysd = c2d(sysc, Ts, 'zoh');
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

% Extract columns for readability
Bv = Bd(:,1);    % Va path
Bt = Bd(:,2);    % Tl path

% Nonlinear sim horizon
Tsim = 2.0;                      % [s]
N    = round(Tsim/Ts);
tN   = (0:N-1)*Ts;

% Choose which controller to run in the nonlinear loop:
useAuto = true;   % set false to test manual in nonlinear sim

if useAuto
    Kp_nl = C_auto.Kp; Ki_nl = C_auto.Ki; Kd_nl = C_auto.Kd;
    Tf_nl = max(1e-4, C_auto.Tf);    % ensure >0
else
    Kp_nl = Kp;      Ki_nl = Ki;      Kd_nl = Kd;
    Tf_nl = Tf;
end

% Setpoint (speed)
r = ref_speed * ones(1,N);

% Disturbance torque: step at t = 1.0 s
Tl = zeros(1,N);
Tl(tN >= 1.0) = 0.02;     % [N*m] change as desired

% Measurement noise (speed)
noise_std = 0.02;         % rad/s std dev
rng(1);                   % repeatable
eta = noise_std*randn(1,N);

% Discrete PID (position form with filtered derivative)
% y_m    : measured speed (with noise)
% e      : error (r - y_m)
% u_pid  : unsaturated controller output
% Va     : saturated actuator voltage (±Vmax)
% Anti-windup: simple integrator clamping when |Va|=Vmax and e drives windup
x = [0; 0];               % states: [i; omega]
theta_accum = 0;          % (optional) integrated position if you want it
e_prev = 0; d_prev = 0; I_term = 0; Va = 0;

% Precompute derivative filter alpha = Ts/(Tf+Ts)
alpha_d = Ts / (Tf_nl + Ts);

y_log   = zeros(1,N);
Va_log  = zeros(1,N);
e_log   = zeros(1,N);
Tl_log  = Tl;

for k = 1:N
    % Output (true) and measured
    y_true = Cd*x + Dd*[Va; Tl(k)];     % omega
    y_meas = y_true + eta(k);

    % Error
    e = r(k) - y_meas;

    % Derivative (filtered on measurement)
    d = (1 - alpha_d)*d_prev + alpha_d*(e - e_prev)/Ts;

    % PI with derivative-on-measurement filter (standard)
    P_term = Kp_nl * e;
    I_term = I_term + Ki_nl*Ts*e;     % integrate

    u_pid  = P_term + I_term + Kd_nl*d;

    % Saturation
    Va_unsat = u_pid;
    Va = min(max(Va_unsat, -Vmax), Vmax);

    % Anti-windup (integrator clamping)
    % If saturating and control is trying to push further into saturation, freeze I_term
    if (Va ~= Va_unsat)
        % if pushing beyond positive limit and e>0 OR pushing beyond negative and e<0 -> clamp
        if (Va == Vmax && u_pid > Vmax && e > 0) || (Va == -Vmax && u_pid < -Vmax && e < 0)
            I_term = I_term - Ki_nl*Ts*e;  % undo last integration
        end
    end

    % Plant update (discrete state-space)
    x = Ad*x + Bv*Va + Bt*Tl(k);

    % Optional position accumulation
    theta_accum = theta_accum + y_true*Ts;

    % Logs
    y_log(k)  = y_true;
    Va_log(k) = Va;
    e_log(k)  = e;

    % Shift
    e_prev = e; d_prev = d;
end

%% --------------------- Nonlinear Sim Plots -------------------
figure;
subplot(3,1,1);
plot(tN, r, 'k--', 'LineWidth', 1.2); hold on;
plot(tN, y_log, 'LineWidth', 1.8);
grid on; ylabel('\omega (rad/s)');
title(sprintf('Nonlinear Sim: Saturation (\\pm%.1f V), Anti-windup, Noise, Load Step', Vmax));
legend('Reference','Output','Location','best');

subplot(3,1,2);
plot(tN, Va_log, 'LineWidth', 1.8); grid on;
ylabel('V_a (V)'); yline(Vmax,'--'); yline(-Vmax,'--');

subplot(3,1,3);
plot(tN, Tl_log, 'LineWidth', 1.8); grid on;
xlabel('Time (s)'); ylabel('T_L (N·m)');

%% --------------------- Disturbance Rejection (Linear view) ---
% A linear, two-input disturbance model is more complex in pure TF form,
% but the nonlinear sim above already demonstrates load-step rejection.
% If you still want a linearized disturbance-to-output TF, you can derive it
% from the state-space model about the operating point and analyze with 'lsim'.

%% --------------------- Notes / How to Use --------------------
% 1) Tune Kp, Ki, Kd (and Tf) for C_manual, then re-run.
% 2) Flip 'useAuto' true/false to test automatic vs manual in the nonlinear loop.
% 3) Adjust Vmax, noise_std, and disturbance Tl step to stress-test robustness.
% 4) For outer-loop position control, use CL_pos_* and/or integrate omega in nonlinear loop.
% 5) Use 'margin' plots to judge robustness (target PM ~ 45–60 deg; GM > ~6 dB).
