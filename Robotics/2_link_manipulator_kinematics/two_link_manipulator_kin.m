%% ===============================================================
%  2-Link Planar Manipulator – Forward Kinematics Simulation
%  Author: Yacine Ouhrouche
%  ===============================================================
%  What:   Simulate a 2-link planar robotic arm and compute the
%          end-effector position using trigonometric and DH equations.
%  How:    Use homogeneous transformation matrices for each link.
%  Why:    Builds intuition for robotic arm geometry and motion.
%  ===============================================================

clear; close all; clc;

%% ---------------------- Link Parameters --------------------------
L1 = 1.0;      % Length of Link 1 (m)
L2 = 0.7;      % Length of Link 2 (m)

% Define joint angle ranges (in radians)
theta1 = linspace(-pi/2, pi/2, 50);   % Joint 1 rotation
theta2 = linspace(-pi/2, pi/2, 50);   % Joint 2 rotation

%% ---------------------- Forward Kinematics -----------------------
% Using Denavit–Hartenberg approach for a 2-link planar arm

% End-effector position arrays
X = zeros(length(theta1), length(theta2));
Y = zeros(length(theta1), length(theta2));

for i = 1:length(theta1)
    for j = 1:length(theta2)
        % Transformation matrices
        T1 = [cos(theta1(i)) -sin(theta1(i)) 0 L1*cos(theta1(i));
              sin(theta1(i))  cos(theta1(i)) 0 L1*sin(theta1(i));
              0                0              1 0;
              0                0              0 1];
        
        T2 = [cos(theta2(j)) -sin(theta2(j)) 0 L2*cos(theta2(j));
              sin(theta2(j))  cos(theta2(j)) 0 L2*sin(theta2(j));
              0                0              1 0;
              0                0              0 1];
        
        % Compute overall transformation
        T = T1 * T2;
        
        % End-effector position
        X(i,j) = T(1,4);
        Y(i,j) = T(2,4);
    end
end

%% ---------------------- Visualization ----------------------------
figure('Color','w');
surf(X, Y, zeros(size(X)), 'FaceAlpha', 0.7, 'EdgeColor', 'none');
hold on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel(' ');
title('2-Link Planar Manipulator Workspace');
grid on; axis equal;
colormap jet;

%% ---------------------- Animation --------------------------------
figure('Color','w');
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('2-Link Planar Manipulator – Animation');

for k = 1:length(theta1)
    % Select a corresponding second angle (for animation simplicity)
    t1 = theta1(k);
    t2 = theta2(k);
    
    % Joint positions
    joint1 = [L1*cos(t1); L1*sin(t1)];
    end_eff = [L1*cos(t1) + L2*cos(t1 + t2);
               L1*sin(t1) + L2*sin(t1 + t2)];
    
    % Plot arm
    plot([0 joint1(1) end_eff(1)], [0 joint1(2) end_eff(2)], 'LineWidth', 2, 'Color', [0 0.4 0.8]);
    hold on;
    plot(end_eff(1), end_eff(2), 'ro', 'MarkerFaceColor', 'r');
    hold off;
    
    xlim([-2 2]); ylim([-2 2]);
    pause(0.05);
end

disp('Simulation complete.');
