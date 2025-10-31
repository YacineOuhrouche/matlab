%% ===============================================================
% Field-Oriented Control (FOC) for PMSM - Realistic MATLAB Script
% Author: Yacine Ouhrouche
% ===============================================================
clear; close all; clc;

%% -------------------- Motor & Drive Parameters -------------------------
params.Rs        = 0.03;        % Stator resistance [Ohm]
params.Ld        = 0.0006;      % d-axis inductance [H]
params.Lq        = 0.0007;      % q-axis inductance [H]
params.lambda_m  = 0.015;       % Permanent magnet flux linkage [Wb]
params.p         = 4;           % Number of pole pairs
params.J         = 0.02;        % Rotor inertia [kg·m²]
params.B         = 0.001;       % Viscous friction coefficient [N·m·s]
params.Vdc       = 300;         % DC-link voltage [V]

%% -------------------- Simulation Settings ------------------------------
fsim = 20e3;                    % Simulation frequency [Hz]
dt   = 1/fsim;
Tsim = 1.0;                     % Total simulation time [s]
time = 0:dt:Tsim;
N    = numel(time);

%% -------------------- Controller Parameters ----------------------------
% --- Current loops (inner) ---
ctrl.Kp_id = 10;   ctrl.Ki_id = 500;
ctrl.Kp_iq = 10;   ctrl.Ki_iq = 500;

% --- Speed loop (outer) ---
ctrl.Kp_spd = 0.8; ctrl.Ki_spd = 25;

% --- Saturations & Limits ---
ctrl.Vdq_max  = 0.577 * params.Vdc;  % dq voltage limit (SVPWM)
ctrl.Iq_max   = 180;
ctrl.Id_max   = 50;
ctrl.int_max  = 50;                  % Anti-windup limit

%% -------------------- Reference Commands -------------------------------
speed_ref_rpm = 2000;               
speed_ref     = speed_ref_rpm * 2*pi/60;  % rad/s
id_ref        = 0;                        % Surface PMSM (no field weakening)

%% -------------------- Preallocate State Variables ----------------------
id = zeros(1,N); iq = zeros(1,N);
vd = zeros(1,N); vq = zeros(1,N);
omega_m = zeros(1,N); theta_r = zeros(1,N);
torque = zeros(1,N); iq_ref = zeros(1,N);
ia = zeros(1,N); ib = zeros(1,N); ic = zeros(1,N);
idc = zeros(1,N);

% Integrators
int_id = 0; int_iq = 0; int_spd = 0;

%% -------------------- Helper Functions ---------------------------------
park      = @(alpha,beta,theta) [cos(theta) sin(theta); -sin(theta) cos(theta)] * [alpha; beta];
invPark   = @(vd,vq,theta)      [cos(theta) -sin(theta); sin(theta) cos(theta)] * [vd; vq];
invClarke = @(v_alpha,v_beta)   [v_alpha; -0.5*v_alpha+(sqrt(3)/2)*v_beta; -0.5*v_alpha-(sqrt(3)/2)*v_beta];

%% -------------------- Simulation Loop ----------------------------------
for k = 1:N-1
    
    % --- Electrical angle & speed ---
    omega_e = params.p * omega_m(k);
    theta_r(k+1) = theta_r(k) + omega_m(k)*dt;
    
    % --- Speed control (outer loop) ---
    spd_err = speed_ref - omega_m(k);
    int_spd = max(min(int_spd + spd_err*dt, ctrl.int_max), -ctrl.int_max);
    iq_ref(k) = ctrl.Kp_spd*spd_err + ctrl.Ki_spd*int_spd;
    iq_ref(k) = max(min(iq_ref(k), ctrl.Iq_max), -ctrl.Iq_max);

    % --- Current control (inner loops) ---
    err_id = id_ref - id(k);
    err_iq = iq_ref(k) - iq(k);

    int_id = max(min(int_id + err_id*dt, ctrl.int_max), -ctrl.int_max);
    int_iq = max(min(int_iq + err_iq*dt, ctrl.int_max), -ctrl.int_max);

    % Feedforward decoupling
    vd_ref = ctrl.Kp_id*err_id + ctrl.Ki_id*int_id - omega_e * params.Lq * iq(k);
    vq_ref = ctrl.Kp_iq*err_iq + ctrl.Ki_iq*int_iq + omega_e * (params.Ld*id(k) + params.lambda_m);
    
    % Voltage saturation
    Vmag = sqrt(vd_ref^2 + vq_ref^2);
    if Vmag > ctrl.Vdq_max
        scale = ctrl.Vdq_max / Vmag;
        vd_ref = vd_ref * scale;
        vq_ref = vq_ref * scale;
    end
    vd(k) = vd_ref; vq(k) = vq_ref;
    
    % --- PMSM dq model ---
    did = (vd(k) - params.Rs*id(k) + omega_e*params.Lq*iq(k)) / params.Ld;
    diq = (vq(k) - params.Rs*iq(k) - omega_e*(params.Ld*id(k) + params.lambda_m)) / params.Lq;

    id(k+1) = id(k) + did*dt;
    iq(k+1) = iq(k) + diq*dt;

    % --- Electromagnetic torque ---
    Te = (3/2) * params.p * (params.lambda_m*iq(k) + (params.Ld - params.Lq)*id(k)*iq(k));
    torque(k) = Te;
    
    % --- Mechanical model ---
    Tload = 0.2*sign(omega_m(k)) + 0.5*sin(4*pi*time(k)); % load ripple
    domega = (Te - Tload - params.B*omega_m(k)) / params.J;
    omega_m(k+1) = omega_m(k) + domega*dt;

    % --- Phase currents reconstruction ---
    i_alpha_beta = [cos(theta_r(k)) -sin(theta_r(k)); sin(theta_r(k)) cos(theta_r(k))]*[id(k); iq(k)];
    ia(k) = i_alpha_beta(1);
    ib(k) = -0.5*i_alpha_beta(1) + (sqrt(3)/2)*i_alpha_beta(2);
    ic(k) = -0.5*i_alpha_beta(1) - (sqrt(3)/2)*i_alpha_beta(2);

    % --- DC link current (power estimation) ---
    Pelec = (3/2)*(vd(k)*id(k) + vq(k)*iq(k));
    idc(k) = Pelec / params.Vdc;
end

%% -------------------- Plot Results -------------------------------------
figure('Name','PMSM FOC Performance','Position',[100 100 1000 700]);

subplot(3,1,1)
plot(time, omega_m*60/(2*pi), 'LineWidth',1.6); hold on;
yline(speed_ref_rpm, 'k--','Speed Ref');
ylabel('Speed [rpm]'); title('Speed Response');
grid on; legend('Measured','Reference');

subplot(3,1,2)
plot(time, iq, 'r', 'LineWidth',1.3); hold on;
plot(time, iq_ref, 'k--','LineWidth',1.2);
ylabel('q-axis Current [A]'); legend('i_q','i_q^*');
grid on; title('Current Tracking');

subplot(3,1,3)
plot(time, torque, 'm', 'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Torque [N·m]');
grid on; title('Electromagnetic Torque');

figure('Name','Currents & DC Link','Position',[120 100 1000 600]);
subplot(2,1,1)
plot(time, id,'b','LineWidth',1.3);
ylabel('d-axis Current [A]'); grid on;
title('i_d Current');

subplot(2,1,2)
plot(time, idc,'LineWidth',1.3);
ylabel('DC Link Current [A]'); xlabel('Time [s]');
grid on; title('Estimated DC Current');

disp('✅ FOC simulation completed successfully.');
