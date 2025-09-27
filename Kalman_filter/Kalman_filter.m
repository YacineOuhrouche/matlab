clc; clear; close all;

%% Parameters
dt = 0.1;               % Time step (s)
T  = 20;                % Total time (s)
N  = T/dt;              % Number of steps

%% True system (object moving in 2D with constant velocity)
vx_true = 1;            % True x velocity (m/s)
vy_true = 0.5;          % True y velocity (m/s)

%% State-space model
% State vector: [x; y; vx; vy]
A = [1 0 dt 0;   % State transition
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];

H = [1 0 0 0;   % Measurement matrix (we only measure position)
     0 1 0 0];

Q = 0.01 * eye(4);  % Process noise covariance
R = 0.5 * eye(2);   % Measurement noise covariance

%% Initialization
x = [0; 0; vx_true; vy_true];  % Initial true state
x_est = [0; 0; 0; 0];          % Initial estimate
P = eye(4);                    % Initial estimation covariance

%% Storage
true_states = zeros(4,N);
meas = zeros(2,N);
estimates = zeros(4,N);

%% Helper function for multivariate Gaussian noise (no toolbox needed)
randn_cov = @(cov,n) chol(cov,'lower') * randn(size(cov,1),n);

%% Simulation loop
for k = 1:N
    % True motion with process noise
    x = A * x + randn_cov(Q,1);
    true_states(:,k) = x;
    
    % Measurement (position only, noisy)
    z = H * x + randn_cov(R,1);
    meas(:,k) = z;
    
    % === Kalman Filter ===
    % Prediction
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;
    
    % Update
    K = P_pred * H' / (H * P_pred * H' + R);   % Kalman gain
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(4) - K*H) * P_pred;
    
    % Store
    estimates(:,k) = x_est;
end

%% Visualization
figure; hold on; grid on;
plot(true_states(1,:), true_states(2,:), 'b-', 'LineWidth', 2);       % True path
plot(meas(1,:), meas(2,:), 'rx', 'MarkerSize', 6);                    % Noisy measurements
plot(estimates(1,:), estimates(2,:), 'g-', 'LineWidth', 2);           % Kalman estimate
legend('True Path','Measurements','Kalman Estimate');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('2D Object Tracking with Kalman Filter');
