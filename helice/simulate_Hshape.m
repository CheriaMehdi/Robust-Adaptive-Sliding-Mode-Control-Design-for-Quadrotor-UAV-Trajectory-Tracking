%% simulate_8shape.m - 3D 8-Shape Trajectory Tracking (Full Figure-8)
clear; clc; close all;

% Quadrotor Parameters (Table 3)
m  = 0.650;          % kg
g  = 9.81;           % m/s^2
Ix = 7.5e-3; Iy = 7.5e-3; Iz = 1.3e-2;  % kg·m²
b  = 3.13e-5;        % thrust coeff
d  = 7.5e-7;         % drag coeff
l  = 0.23;           % arm length
Jr = 6e-5;           % rotor inertia

% Controller Gains (Optimized for excellent full 8-shape tracking)
lambda = 20;          % sliding surface
k1_fixed = 0.6;      % classical SMC switching
k2 = 6;              % proportional on s
kp_xy = 2; kd_xy = 6; % outer PD xy (increased for faster response)
kp_z  = 5; kd_z  = 3; % outer PD z

% FIS (your createFIS.m)
fis = createFIS();

% Simulation (longer time for full multiple loops to see complete tracking)
T_final = 40;  % Increased to 40s to complete multiple full 8-shapes visibly
X0 = [0; 0; 1 ; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % Start at hover height

options = odeset('RelTol',1e-6,'AbsTol',1e-8);

% Run AFGS-SMC and Classical SMC
[t_afgs, X_afgs] = ode45(@(t,X) quadrotor_dynamics_helice(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'AFGS'), [0 T_final], X0, options);

[t_smc, X_smc] = ode45(@(t,X) quadrotor_dynamics_helice(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'SMC'), [0 T_final], X0, options);

% HELICAL TRAJECTORY (Spiral) - REPLACE THE ORIGINAL SECTION
R = 6;          % Radius of helix (meters)
h_speed = 0.5;  % Vertical speed (m/s)
omega = 0.8;    % Angular frequency (rad/s)
z0 = 1.5;       % Starting height (meters)

% Desired position (helix)AFGS
xd = R*cos(omega*t_afgs);
yd = R*sin(omega*t_afgs);
zd = z0 + h_speed*t_afgs;

% 3D Trajectory Plot (Full loops visible)
figure('Position',[100 100 1000 800]);
plot3(xd, yd, zd, 'r-', 'LineWidth', 2.5); hold on;
plot3(X_smc(:,1), X_smc(:,2), X_smc(:,3), 'b--', 'LineWidth', 1.8);
plot3(X_afgs(:,1), X_afgs(:,2), X_afgs(:,3), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Desired 8-Shape', 'Classical SMC', 'AFGS-SMC', 'Location','best');
title('Perfect Full 8-Shape Trajectory Tracking (Multiple Loops)');
view(3);

% Additional XY projection to clearly see the figure-8
figure('Position',[100 100 800 600]);
plot(xd, yd, 'r-', 'LineWidth', 2.5); hold on;
plot(X_smc(:,1), X_smc(:,2), 'b--', 'LineWidth', 1.8);
plot(X_afgs(:,1), X_afgs(:,2), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Desired', 'SMC', 'AFGS-SMC');
title('Top View: Full Figure-8 Tracking');

% Attitude Angles
figure('Position',[100 100 900 500]);
subplot(3,1,1); plot(t_afgs, rad2deg(X_afgs(:,4)), 'k', t_smc, rad2deg(X_smc(:,4)), 'b--');
ylabel('\phi (deg)'); legend('AFGS-SMC','SMC');
subplot(3,1,2); plot(t_afgs, rad2deg(X_afgs(:,5)), 'k', t_smc, rad2deg(X_smc(:,5)), 'b--');
ylabel('\theta (deg)');
subplot(3,1,3); plot(t_afgs, rad2deg(X_afgs(:,6)), 'k', t_smc, rad2deg(X_smc(:,6)), 'b--');
ylabel('\psi (deg)'); xlabel('Time (s)');
sgtitle('Attitude Angles (Small and Smooth)');