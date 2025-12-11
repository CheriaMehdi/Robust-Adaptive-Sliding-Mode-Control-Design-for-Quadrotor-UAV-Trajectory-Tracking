%% simulate_8shape.m - 3D single-loop Figure-8 tracking (AFGS vs SMC)
clear; clc; close all;

% Quadrotor Parameters
m  = 0.650; g = 9.81;
Ix = 7.5e-3; Iy = 7.5e-3; Iz = 1.3e-2;
b  = 3.13e-5; d = 7.5e-7; l = 0.23; Jr = 6e-5;

% Controller Gains
lambda = 6;          % sliding surface
k2 = 6;              % proportional on s
kp_xy = 6; kd_xy = 4;
kp_z  = 5; kd_z  = 3;

% Fixed k1 for classical SMC and fallback for AFGS (if you want pass small)
k1_fixed_smc  = 0.9;   % larger switching for classical SMC
k1_fixed_afgs = 0.6;   % fallback (AFGS uses FIS normally)

% FIS
fis = createFIS();   % assume returns valid FIS

% Trajectory parameters (centralised in main)
traj.a   = 10;
traj.b   = 5;
traj.wz  = 0.3;
traj.zd  = 4.5;   % constant altitude

% Simulate one full fundamental period -> single figure-8 loop
T_final = 2*pi / traj.wz;   % one period for sin(wz*t)
X0 = [0; 0; traj.zd; 0; 0; 0; 0; 0; 0; 0; 0; 0];

options = odeset('RelTol',1e-6,'AbsTol',1e-8);

% AFGS-SMC run (adaptive fuzzy gains)
[t_afgs, X_afgs] = ode45(@(t,X) quadrotor_dynamics(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr, ...
    lambda,k1_fixed_afgs,k2,kp_xy,kd_xy,kp_z,kd_z,'AFGS', traj), [0 T_final], X0, options);

% Classical SMC run (fixed switching)
[t_smc, X_smc]   = ode45(@(t,X) quadrotor_dynamics(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr, ...
    lambda,k1_fixed_smc,k2,kp_xy,kd_xy,kp_z,kd_z,'SMC', traj), [0 T_final], X0, options);

% Desired trajectory (for plotting) using same traj
xd = traj.a * sin(traj.wz * t_afgs);
yd = traj.b * sin(2 * traj.wz * t_afgs);
zd = traj.zd * ones(size(t_afgs));

% Plot results (single loop)
figure('Position',[100 100 1000 800]);
plot3(xd, yd, zd, 'r-', 'LineWidth', 2.5); hold on;
plot3(X_smc(:,1), X_smc(:,2), X_smc(:,3), 'b--', 'LineWidth', 1.8);
plot3(X_afgs(:,1), X_afgs(:,2), X_afgs(:,3), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Desired 8-Shape', 'Classical SMC', 'AFGS-SMC', 'Location','best');
title('Single Figure-8 Loop Tracking');
view(3);

% XY projection
figure('Position',[100 100 800 600]);
plot(xd, yd, 'r-', 'LineWidth', 2.5); hold on;
plot(X_smc(:,1), X_smc(:,2), 'b--', 'LineWidth', 1.8);
plot(X_afgs(:,1), X_afgs(:,2), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Desired', 'SMC', 'AFGS-SMC');
title('Top View: Single Figure-8 Loop');

% Attitude Angles
figure('Position',[100 100 900 500]);
subplot(3,1,1); plot(t_afgs, rad2deg(X_afgs(:,4)), 'k', t_smc, rad2deg(X_smc(:,4)), 'b--');
ylabel('\phi (deg)'); legend('AFGS-SMC','SMC');
subplot(3,1,2); plot(t_afgs, rad2deg(X_afgs(:,5)), 'k', t_smc, rad2deg(X_smc(:,5)), 'b--');
ylabel('\theta (deg)');
subplot(3,1,3); plot(t_afgs, rad2deg(X_afgs(:,6)), 'k', t_smc, rad2deg(X_smc(:,6)), 'b--');
ylabel('\psi (deg)'); xlabel('Time (s)');
sgtitle('Attitude Angles (Single Loop)');
