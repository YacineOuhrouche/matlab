clc; clear; close all;

%% Parameters
dt = 0.1;               % Time step 
T  = 20;                % Total time 
N  = T/dt;              % Number of steps

%% Object moving in a 3D complex trajectory
r = 10;                 %  radius
omega = 0.2;            % Angular velocity
z_amp = 5;              % Z-axis amplitude
z_freq = 0.1;           % Z-axis oscillation frequency

%% EKF State-space model
% State vector: [x; y; z; vx; vy; vz]
Q = 0.05 * eye(6);      % Process noise covariance
R = 0.5 * eye(3);       % Measurement noise covariance
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];      % Measurement matrix (position only)

%% Initialization
x_est = [r; 0; 0; 0; r*omega; 0];  % Initial EKF estimate
P = eye(6);                         % Initial covariance

%% Storage
true_states = zeros(6,N);
meas = zeros(3,N);
estimates = zeros(6,N);
pos_error = zeros(3,N);
vel_error = zeros(3,N);

%% Helper function for Gaussian noise
randn_cov = @(cov,n) chol(cov,'lower') * randn(size(cov,1),n);

%% Simulation loop
for k = 1:N
    t = (k-1)*dt;
    
    % --- True 3D complex motion ---
    x_true = [r*cos(omega*t);
              r*sin(omega*t);
              z_amp * sin(2*pi*z_freq*t);
             -r*omega*sin(omega*t);
              r*omega*cos(omega*t);
              2*pi*z_freq*z_amp*cos(2*pi*z_freq*t)];
    true_states(:,k) = x_true;
    
    % --- Measurement (noisy position only) ---
    z = H * x_true + randn_cov(R,1);
    meas(:,k) = z;
    
    % --- EKF Prediction ---
    f = @(x) [x(1) + dt*x(4);
              x(2) + dt*x(5);
              x(3) + dt*x(6);
              -omega^2*x(1)*dt + x(4);
              -omega^2*x(2)*dt + x(5);
              - (2*pi*z_freq)^2*x(3)*dt + x(6)];
    
    F = [1 0 0 dt 0 0;
         0 1 0 0 dt 0;
         0 0 1 0 0 dt;
        -omega^2*dt 0 0 1 0 0;
         0 -omega^2*dt 0 0 1 0;
         0 0 -(2*pi*z_freq)^2*dt 0 0 1];
     
    x_pred = f(x_est);
    P_pred = F * P * F' + Q;
    
    % --- Measurement Update ---
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z - H*x_pred);
    P = (eye(6) - K*H) * P_pred;
    
    % Store estimates and errors
    estimates(:,k) = x_est;
    pos_error(:,k) = x_true(1:3) - x_est(1:3);
    vel_error(:,k) = x_true(4:6) - x_est(4:6);
end

%% --- Visualization ---

t = 0:dt:T-dt;

% 1) 3D Position tracking
figure; hold on; grid on; axis equal;
plot3(true_states(1,:), true_states(2,:), true_states(3,:), 'b-', 'LineWidth', 2);
plot3(meas(1,:), meas(2,:), meas(3,:), 'rx', 'MarkerSize', 6);
plot3(estimates(1,:), estimates(2,:), estimates(3,:), 'g--', 'LineWidth', 2);
legend('True Path','Measurements','EKF Estimate');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Complex Motion Tracking with EKF');
view(45,30);

% 2) Velocity tracking
figure;
subplot(3,1,1);
plot(t, true_states(4,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates(4,:), 'g--', 'LineWidth', 2);
legend('True vx','EKF vx'); xlabel('Time (s)'); ylabel('Velocity X (m/s)');

subplot(3,1,2);
plot(t, true_states(5,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates(5,:), 'g--', 'LineWidth', 2);
legend('True vy','EKF vy'); xlabel('Time (s)'); ylabel('Velocity Y (m/s)');

subplot(3,1,3);
plot(t, true_states(6,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates(6,:), 'g--', 'LineWidth', 2);
legend('True vz','EKF vz'); xlabel('Time (s)'); ylabel('Velocity Z (m/s)');

% 3) Position estimation errors
figure;
subplot(3,1,1);
plot(t, pos_error(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, pos_error(2,:), 'm', 'LineWidth', 1.2);
plot(t, pos_error(3,:), 'c', 'LineWidth', 1.2); grid on;
legend('X Error','Y Error','Z Error'); xlabel('Time (s)'); ylabel('Error (m)');
title('Position Estimation Errors');

% 4) Velocity estimation errors
figure;
subplot(3,1,1);
plot(t, vel_error(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, vel_error(2,:), 'm', 'LineWidth', 1.2);
plot(t, vel_error(3,:), 'c', 'LineWidth', 1.2); grid on;
legend('vx Error','vy Error','vz Error'); xlabel('Time (s)'); ylabel('Error (m/s)');
title('Velocity Estimation Errors');
