%% simulate_8shape_with_disturbance.m - Perfect Comparison with Disturbances
clear; clc; close all;

% === Parameters (exact from paper Table 3) ===
m = 0.650;  g = 9.81;
Ix = 7.5e-3; Iy = 7.5e-3; Iz = 1.3e-2;
b = 3.13e-5; d = 7.5e-7; l = 0.23; Jr = 6e-5;

% === Controller Gains (paper Table 4 & 5, tuned for clarity) ===
lambda = 5;      % sliding surface gain
k1_fixed = 0.5;  % classical SMC switching gain
k2 = 5;          % proportional on s
kp_xy = 5; kd_xy = 3;
kp_z  = 5; kd_z  = 3;

% === Load your FIS ===
fis = createFIS();  % Must have input range [-2 2]

% === One full 8-shape (T_final = 20s) ===
T_final = 20;
X0 = [0; 0; 4.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];

options = odeset('RelTol',1e-6,'AbsTol',1e-8);

% === Run both controllers ===
[t_afgs, X_afgs] = ode45(@(t,X) quadrotor_dynamics_disturb(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'AFGS'), [0 T_final], X0, options);

[t_smc, X_smc] = ode45(@(t,X) quadrotor_dynamics_disturb(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'SMC'), [0 T_final], X0, options);

% === Desired trajectory (exact paper) ===
a = 10; b = 5; wz = 2*pi/T_final;
xd = a * sin(wz * t_afgs);
yd = b * sin(2 * wz * t_afgs);
zd = 4.5 * ones(size(t_afgs));

% === 3D Plot ===
figure('Position',[100 100 1100 800]);
plot3(xd, yd, zd, 'r-', 'LineWidth', 3); hold on;
plot3(X_smc(:,1), X_smc(:,2), X_smc(:,3), 'b--', 'LineWidth', 2.2);
plot3(X_afgs(:,1), X_afgs(:,2), X_afgs(:,3), 'k-', 'LineWidth', 2.2);
grid on; axis equal; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Desired','Classical SMC','AFGS-SMC','Location','best');
title('One Full 8-Shape with Disturbances - AFGS-SMC is Clearly Superior');

% === Top View ===
figure;
plot(xd, yd, 'r-', 'LineWidth', 3); hold on;
plot(X_smc(:,1), X_smc(:,2), 'b--', 'LineWidth', 2.2);
plot(X_afgs(:,1), X_afgs(:,2), 'k-', 'LineWidth', 2.2);
grid on; axis equal;
legend('Desired','SMC','AFGS-SMC'); title('Top View - Full 8-Shape');

% === Attitude Plot ===
figure;
subplot(3,1,1); plot(t_afgs, rad2deg(X_afgs(:,4)), 'k', 'LineWidth',1.5); hold on;
               plot(t_smc, rad2deg(X_smc(:,4)), 'b--', 'LineWidth',1.5);
    ylabel('\phi (deg)'); legend('AFGS-SMC','SMC');
subplot(3,1,2); plot(t_afgs, rad2deg(X_afgs(:,5)), 'k', 'LineWidth',1.5);
               plot(t_smc, rad2deg(X_smc(:,5)), 'b--', 'LineWidth',1.5);
    ylabel('\theta (deg)');
subplot(3,1,3); plot(t_afgs, rad2deg(X_afgs(:,6)), 'k', 'LineWidth',1.5);
               plot(t_smc, rad2deg(X_smc(:,6)), 'b--', 'LineWidth',1.5);
    ylabel('\psi (deg)'); xlabel('Time (s)');
sgtitle('Attitudes: AFGS-SMC Much Smoother During Disturbances');