% %% simulate_8shape.m - 3D 8-Shape Trajectory Tracking (Full Figure-8)
% clear; clc; close all;
% 
% % Quadrotor Parameters (Table 3)
% m  = 0.650;          % kg
% g  = 9.81;           % m/s^2
% Ix = 7.5e-3; Iy = 7.5e-3; Iz = 1.3e-2;  % kg·m²
% b  = 3.13e-5;        % thrust coeff
% d  = 7.5e-7;         % drag coeff
% l  = 0.23;           % arm length
% Jr = 6e-5;           % rotor inertia
% 
% % Controller Gains (Optimized for excellent full 8-shape tracking)
% lambda = 6;          % sliding surface
% k1_fixed = 0.6;      % classical SMC switching
% k2 = 6;              % proportional on s
% kp_xy = 6; kd_xy = 4; % outer PD xy (increased for faster response)
% kp_z  = 5; kd_z  = 3; % outer PD z
% 
% % FIS (your createFIS.m)
% fis = createFIS();
% 
% % Simulation (longer time for full multiple loops to see complete tracking)
% T_final = 40;  % Increased to 40s to complete multiple full 8-shapes visibly
% X0 = [0; 0; 4.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % Start at hover height
% 
% options = odeset('RelTol',1e-6,'AbsTol',1e-8);
% 
% % Run AFGS-SMC and Classical SMC
% [t_afgs, X_afgs] = ode45(@(t,X) quadrotor_dynamics(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
%     lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'AFGS'), [0 T_final], X0, options);
% 
% [t_smc, X_smc] = ode45(@(t,X) quadrotor_dynamics(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
%     lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'SMC'), [0 T_final], X0, options);
% 
% % Desired Trajectory (slower for smooth realistic motion)
% a = 10; b_val = 5; wz = 0.3;  % wz=0.3 rad/s gives period ~20.9s for one full 8
% xd = a * sin(wz * t_afgs);
% yd = b_val * sin(2 * wz * t_afgs);
% zd = 4.5 * ones(size(t_afgs));
% 
% % 3D Trajectory Plot (Full loops visible)
% figure('Position',[100 100 1000 800]);
% plot3(xd, yd, zd, 'r-', 'LineWidth', 2.5); hold on;
% plot3(X_smc(:,1), X_smc(:,2), X_smc(:,3), 'b--', 'LineWidth', 1.8);
% plot3(X_afgs(:,1), X_afgs(:,2), X_afgs(:,3), 'k-', 'LineWidth', 1.8);
% grid on; axis equal;
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% legend('Desired 8-Shape', 'Classical SMC', 'AFGS-SMC', 'Location','best');
% title('Perfect Full 8-Shape Trajectory Tracking (Multiple Loops)');
% view(3);
% 
% % Additional XY projection to clearly see the figure-8
% figure('Position',[100 100 800 600]);
% plot(xd, yd, 'r-', 'LineWidth', 2.5); hold on;
% plot(X_smc(:,1), X_smc(:,2), 'b--', 'LineWidth', 1.8);
% plot(X_afgs(:,1), X_afgs(:,2), 'k-', 'LineWidth', 1.8);
% grid on; axis equal;
% xlabel('X (m)'); ylabel('Y (m)');
% legend('Desired', 'SMC', 'AFGS-SMC');
% title('Top View: Full Figure-8 Tracking');
% 
% % Attitude Angles
% figure('Position',[100 100 900 500]);
% subplot(3,1,1); plot(t_afgs, rad2deg(X_afgs(:,4)), 'k', t_smc, rad2deg(X_smc(:,4)), 'b--');
% ylabel('\phi (deg)'); legend('AFGS-SMC','SMC');
% subplot(3,1,2); plot(t_afgs, rad2deg(X_afgs(:,5)), 'k', t_smc, rad2deg(X_smc(:,5)), 'b--');
% ylabel('\theta (deg)');
% subplot(3,1,3); plot(t_afgs, rad2deg(X_afgs(:,6)), 'k', t_smc, rad2deg(X_smc(:,6)), 'b--');
% ylabel('\psi (deg)'); xlabel('Time (s)');
% sgtitle('Attitude Angles (Small and Smooth)');





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
lambda = 6;          % sliding surface
k1_fixed = 0.6;      % classical SMC switching
k2 = 6;              % proportional on s
kp_xy = 6; kd_xy = 4; % outer PD xy (increased for faster response)
kp_z  = 5; kd_z  = 3; % outer PD z

% FIS (your createFIS.m)
fis = createFIS();

% Simulation (longer time for full multiple loops to see complete tracking)
T_final = 40;  % Increased to 40s to complete multiple full 8-shapes visibly
X0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % Start at hover height

options = odeset('RelTol',1e-6,'AbsTol',1e-8);

% Run AFGS-SMC and Classical SMC
[t_afgs, X_afgs] = ode45(@(t,X) quadrotor_dynamics_disturb(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'AFGS'), [0 T_final], X0, options);

[t_smc, X_smc] = ode45(@(t,X) quadrotor_dynamics_disturb(t,X,fis,m,Ix,Iy,Iz,g,b,d,l,Jr,...
    lambda,k1_fixed,k2,kp_xy,kd_xy,kp_z,kd_z,'SMC'), [0 T_final], X0, options);

% =====================================================================
% CALCULATE PERFORMANCE METRICS
% =====================================================================

% Create desired trajectories for BOTH time vectors
xd_afgs = 10 * sin(0.3 * t_afgs);
yd_afgs = 5 * sin(2 * 0.3 * t_afgs);
zd_afgs = 4.5 * ones(size(t_afgs));

xd_smc = 10 * sin(0.3 * t_smc);
yd_smc = 5 * sin(2 * 0.3 * t_smc);
zd_smc = 4.5 * ones(size(t_smc));

% Position errors - calculate separately for each trajectory
pos_error_afgs = sqrt((X_afgs(:,1)-xd_afgs).^2 + (X_afgs(:,2)-yd_afgs).^2 + (X_afgs(:,3)-zd_afgs).^2);
pos_error_smc = sqrt((X_smc(:,1)-xd_smc).^2 + (X_smc(:,2)-yd_smc).^2 + (X_smc(:,3)-zd_smc).^2);

% Calculate RMSE (Root Mean Square Error)
rmse_afgs_total = sqrt(mean(pos_error_afgs.^2));
rmse_smc_total = sqrt(mean(pos_error_smc.^2));

% Calculate ISE (Integral Square Error)
dt_afgs = diff(t_afgs);
ise_afgs_total = sum(pos_error_afgs(1:end-1).^2 .* dt_afgs);

dt_smc = diff(t_smc);
ise_smc_total = sum(pos_error_smc(1:end-1).^2 .* dt_smc);

% For attitude errors (roll, pitch, yaw)
phi_desired_afgs = zeros(size(t_afgs));  % Desired roll = 0
theta_desired_afgs = zeros(size(t_afgs)); % Desired pitch = 0
psi_desired_afgs = zeros(size(t_afgs));  % Desired yaw = 0

phi_desired_smc = zeros(size(t_smc));  % Desired roll = 0
theta_desired_smc = zeros(size(t_smc)); % Desired pitch = 0
psi_desired_smc = zeros(size(t_smc));  % Desired yaw = 0

% Attitude RMSE
rmse_phi_afgs = sqrt(mean((X_afgs(:,4) - phi_desired_afgs).^2));
rmse_theta_afgs = sqrt(mean((X_afgs(:,5) - theta_desired_afgs).^2));
rmse_psi_afgs = sqrt(mean((X_afgs(:,6) - psi_desired_afgs).^2));

rmse_phi_smc = sqrt(mean((X_smc(:,4) - phi_desired_smc).^2));
rmse_theta_smc = sqrt(mean((X_smc(:,5) - theta_desired_smc).^2));
rmse_psi_smc = sqrt(mean((X_smc(:,6) - psi_desired_smc).^2));

% Attitude ISE
ise_phi_afgs = sum((X_afgs(1:end-1,4) - phi_desired_afgs(1:end-1)).^2 .* dt_afgs);
ise_theta_afgs = sum((X_afgs(1:end-1,5) - theta_desired_afgs(1:end-1)).^2 .* dt_afgs);
ise_psi_afgs = sum((X_afgs(1:end-1,6) - psi_desired_afgs(1:end-1)).^2 .* dt_afgs);

ise_phi_smc = sum((X_smc(1:end-1,4) - phi_desired_smc(1:end-1)).^2 .* dt_smc);
ise_theta_smc = sum((X_smc(1:end-1,5) - theta_desired_smc(1:end-1)).^2 .* dt_smc);
ise_psi_smc = sum((X_smc(1:end-1,6) - psi_desired_smc(1:end-1)).^2 .* dt_smc);

% Also calculate metrics for different time periods
% 1. Disturbance period (20-30s)
idx_disturb_afgs = (t_afgs >= 20) & (t_afgs <= 30);
idx_disturb_smc = (t_smc >= 20) & (t_smc <= 30);

if any(idx_disturb_afgs)
    rmse_afgs_disturb = sqrt(mean(pos_error_afgs(idx_disturb_afgs).^2));
    
    % For disturbance period ISE
    t_disturb_afgs = t_afgs(idx_disturb_afgs);
    pos_error_disturb_afgs = pos_error_afgs(idx_disturb_afgs);
    dt_disturb_afgs = diff(t_disturb_afgs);
    ise_afgs_disturb = sum(pos_error_disturb_afgs(1:end-1).^2 .* dt_disturb_afgs);
else
    rmse_afgs_disturb = NaN;
    ise_afgs_disturb = NaN;
end

if any(idx_disturb_smc)
    rmse_smc_disturb = sqrt(mean(pos_error_smc(idx_disturb_smc).^2));
    
    t_disturb_smc = t_smc(idx_disturb_smc);
    pos_error_disturb_smc = pos_error_smc(idx_disturb_smc);
    dt_disturb_smc = diff(t_disturb_smc);
    ise_smc_disturb = sum(pos_error_disturb_smc(1:end-1).^2 .* dt_disturb_smc);
else
    rmse_smc_disturb = NaN;
    ise_smc_disturb = NaN;
end

% 2. After disturbance (30-40s) to see recovery
idx_recovery_afgs = (t_afgs >= 30) & (t_afgs <= 40);
idx_recovery_smc = (t_smc >= 30) & (t_smc <= 40);

if any(idx_recovery_afgs)
    rmse_afgs_recovery = sqrt(mean(pos_error_afgs(idx_recovery_afgs).^2));
else
    rmse_afgs_recovery = NaN;
end

if any(idx_recovery_smc)
    rmse_smc_recovery = sqrt(mean(pos_error_smc(idx_recovery_smc).^2));
else
    rmse_smc_recovery = NaN;
end

% =====================================================================
% DISPLAY PERFORMANCE METRICS
% =====================================================================
fprintf('\n================== PERFORMANCE METRICS ==================\n');
fprintf('\n--- OVERALL PERFORMANCE (0-40s) ---\n');
fprintf('RMSE - Total Position Error:\n');
fprintf('  AFGS-SMC: %.6f m\n', rmse_afgs_total);
fprintf('  SMC:      %.6f m\n', rmse_smc_total);
fprintf('  Improvement: %.2f%%\n', (rmse_smc_total - rmse_afgs_total)/rmse_smc_total * 100);

fprintf('\nISE - Total Position Error:\n');
fprintf('  AFGS-SMC: %.6f m²·s\n', ise_afgs_total);
fprintf('  SMC:      %.6f m²·s\n', ise_smc_total);
fprintf('  Improvement: %.2f%%\n', (ise_smc_total - ise_afgs_total)/ise_smc_total * 100);

fprintf('\n--- ATTITUDE ERRORS (0-40s) ---\n');
fprintf('Roll RMSE:\n');
fprintf('  AFGS-SMC: %.6f rad (%.2f deg)\n', rmse_phi_afgs, rad2deg(rmse_phi_afgs));
fprintf('  SMC:      %.6f rad (%.2f deg)\n', rmse_phi_smc, rad2deg(rmse_phi_smc));

fprintf('\nPitch RMSE:\n');
fprintf('  AFGS-SMC: %.6f rad (%.2f deg)\n', rmse_theta_afgs, rad2deg(rmse_theta_afgs));
fprintf('  SMC:      %.6f rad (%.2f deg)\n', rmse_theta_smc, rad2deg(rmse_theta_smc));

fprintf('\n--- DISTURBANCE PERIOD (20-30s) ---\n');
fprintf('Position RMSE during disturbance:\n');
fprintf('  AFGS-SMC: %.6f m\n', rmse_afgs_disturb);
fprintf('  SMC:      %.6f m\n', rmse_smc_disturb);
if ~isnan(rmse_smc_disturb) && rmse_smc_disturb > 0
    fprintf('  Improvement: %.2f%%\n', (rmse_smc_disturb - rmse_afgs_disturb)/rmse_smc_disturb * 100);
end

fprintf('\nPosition ISE during disturbance:\n');
fprintf('  AFGS-SMC: %.6f m²·s\n', ise_afgs_disturb);
fprintf('  SMC:      %.6f m²·s\n', ise_smc_disturb);
if ~isnan(ise_smc_disturb) && ise_smc_disturb > 0
    fprintf('  Improvement: %.2f%%\n', (ise_smc_disturb - ise_afgs_disturb)/ise_smc_disturb * 100);
end

fprintf('\n======================================================\n');

% =====================================================================
% PLOT 1: 3D Trajectory Plot
% =====================================================================
figure('Position',[100 100 1000 800]);
% For 3D plot, we need common reference trajectory - use AFGS time as reference
plot3(xd_afgs, yd_afgs, zd_afgs, 'r-', 'LineWidth', 2.5); hold on;
plot3(X_smc(:,1), X_smc(:,2), X_smc(:,3), 'b--', 'LineWidth', 1.8);
plot3(X_afgs(:,1), X_afgs(:,2), X_afgs(:,3), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Desired 8-Shape', 'Classical SMC', 'AFGS-SMC', 'Location','best');
title('3D Trajectory Tracking - 8-Shape Pattern');
view(3);

% =====================================================================
% PLOT 2: XY Projection
% =====================================================================
figure('Position',[100 100 800 600]);
plot(xd_afgs, yd_afgs, 'r-', 'LineWidth', 2.5); hold on;
plot(X_smc(:,1), X_smc(:,2), 'b--', 'LineWidth', 1.8);
plot(X_afgs(:,1), X_afgs(:,2), 'k-', 'LineWidth', 1.8);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
legend('Desired', 'SMC', 'AFGS-SMC');
title('Top View: Figure-8 Tracking');

% =====================================================================
% PLOT 3: Position Errors Over Time and Performance Metrics
% =====================================================================
figure('Position',[100 100 1200 800]);

subplot(3,2,1);
plot(t_afgs, pos_error_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, pos_error_smc, 'b--', 'LineWidth', 1.5);
ylabel('Position Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Total Position Error Over Time');
grid on;
xlabel('Time (s)');

% Add disturbance region shading
ylim_curr = ylim;
patch([20 30 30 20], [ylim_curr(1) ylim_curr(1) ylim_curr(2) ylim_curr(2)], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(findobj(gca, 'Type', 'patch'), 'bottom');

subplot(3,2,2);
semilogy(t_afgs, pos_error_afgs.^2, 'k-', 'LineWidth', 1.5); hold on;
semilogy(t_smc, pos_error_smc.^2, 'b--', 'LineWidth', 1.5);
ylabel('Squared Error (m²)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Squared Position Error (Log Scale)');
grid on;
xlabel('Time (s)');

% Add disturbance region shading
ylim_curr = ylim;
patch([20 30 30 20], [ylim_curr(1) ylim_curr(1) ylim_curr(2) ylim_curr(2)], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(findobj(gca, 'Type', 'patch'), 'bottom');

% =====================================================================
% PLOT 4: Cumulative ISE Over Time
% =====================================================================
% Calculate cumulative ISE
cumulative_ise_afgs = zeros(size(t_afgs));
cumulative_ise_afgs(2:end) = cumsum(pos_error_afgs(1:end-1).^2 .* dt_afgs);

cumulative_ise_smc = zeros(size(t_smc));
cumulative_ise_smc(2:end) = cumsum(pos_error_smc(1:end-1).^2 .* dt_smc);

subplot(3,2,3);
plot(t_afgs, cumulative_ise_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, cumulative_ise_smc, 'b--', 'LineWidth', 1.5);
ylabel('Cumulative ISE (m²·s)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Cumulative Integral Square Error (ISE)');
grid on;
xlabel('Time (s)');

% Add disturbance region shading
ylim_curr = ylim;
patch([20 30 30 20], [ylim_curr(1) ylim_curr(1) ylim_curr(2) ylim_curr(2)], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(findobj(gca, 'Type', 'patch'), 'bottom');

% =====================================================================
% PLOT 5: RMSE Comparison Bar Chart
% =====================================================================
subplot(3,2,4);
rmse_data = [rmse_afgs_total, rmse_smc_total; 
             rmse_afgs_disturb, rmse_smc_disturb;
             rmse_afgs_recovery, rmse_smc_recovery];
bar_labels = {'Overall (0-40s)', 'Disturbance (20-30s)', 'Recovery (30-40s)'};
bar_handle = bar(rmse_data);
set(gca, 'XTickLabel', bar_labels);
ylabel('RMSE (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('RMSE Comparison for Different Periods');
grid on;

% Add values on top of bars
for i = 1:size(rmse_data, 1)
    for j = 1:size(rmse_data, 2)
        if ~isnan(rmse_data(i,j))
            text(i + (j-1.5)*0.2, rmse_data(i,j) + max(rmse_data(:))*0.02, ...
                sprintf('%.4f', rmse_data(i,j)), 'HorizontalAlignment', 'center', 'FontSize', 8);
        end
    end
end

% =====================================================================
% PLOT 6: Attitude Angles
% =====================================================================
subplot(3,2,5);
plot(t_afgs, rad2deg(X_afgs(:,4)), 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rad2deg(X_smc(:,4)), 'b--', 'LineWidth', 1.5);
ylabel('\phi (deg)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Roll Angle');
grid on;
xlabel('Time (s)');

% Add disturbance region shading
ylim_curr = ylim;
patch([20 30 30 20], [ylim_curr(1) ylim_curr(1) ylim_curr(2) ylim_curr(2)], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(findobj(gca, 'Type', 'patch'), 'bottom');

subplot(3,2,6);
plot(t_afgs, rad2deg(X_afgs(:,5)), 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rad2deg(X_smc(:,5)), 'b--', 'LineWidth', 1.5);
ylabel('\theta (deg)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Pitch Angle');
grid on;
xlabel('Time (s)');

% Add disturbance region shading
ylim_curr = ylim;
patch([20 30 30 20], [ylim_curr(1) ylim_curr(1) ylim_curr(2) ylim_curr(2)], ...
    [0.9 0.9 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
uistack(findobj(gca, 'Type', 'patch'), 'bottom');

sgtitle('Performance Analysis: AFGS-SMC vs Classical SMC');

% =====================================================================
% PLOT 7: Separate Attitude Angles (Original)
% =====================================================================
figure('Position',[100 100 900 500]);

subplot(3,1,1); 
plot(t_afgs, rad2deg(X_afgs(:,4)), 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rad2deg(X_smc(:,4)), 'b--', 'LineWidth', 1.5);
ylabel('\phi (deg)'); 
legend('AFGS-SMC','SMC', 'Location', 'best');
title('Roll Angle');
grid on;

subplot(3,1,2); 
plot(t_afgs, rad2deg(X_afgs(:,5)), 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rad2deg(X_smc(:,5)), 'b--', 'LineWidth', 1.5);
ylabel('\theta (deg)');
title('Pitch Angle');
grid on;

subplot(3,1,3); 
plot(t_afgs, rad2deg(X_afgs(:,6)), 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rad2deg(X_smc(:,6)), 'b--', 'LineWidth', 1.5);
ylabel('\psi (deg)'); 
xlabel('Time (s)');
title('Yaw Angle');
grid on;

sgtitle('Attitude Angles Comparison');

% =====================================================================
% PLOT 8: Summary Table of Metrics
% =====================================================================
figure('Position', [300 100 800 400]);

% Create a table of metrics
metrics_table = {
    'Metric', 'AFGS-SMC', 'SMC', 'Improvement (%)';
    'Position RMSE (m)', sprintf('%.6f', rmse_afgs_total), sprintf('%.6f', rmse_smc_total), sprintf('%.2f', (rmse_smc_total - rmse_afgs_total)/rmse_smc_total * 100);
    'Position ISE (m²·s)', sprintf('%.6f', ise_afgs_total), sprintf('%.6f', ise_smc_total), sprintf('%.2f', (ise_smc_total - ise_afgs_total)/ise_smc_total * 100);
    'Roll RMSE (deg)', sprintf('%.4f', rad2deg(rmse_phi_afgs)), sprintf('%.4f', rad2deg(rmse_phi_smc)), sprintf('%.2f', (rmse_phi_smc - rmse_phi_afgs)/rmse_phi_smc * 100);
    'Pitch RMSE (deg)', sprintf('%.4f', rad2deg(rmse_theta_afgs)), sprintf('%.4f', rad2deg(rmse_theta_smc)), sprintf('%.2f', (rmse_theta_smc - rmse_theta_afgs)/rmse_theta_smc * 100);
    'Disturbance RMSE (m)', sprintf('%.6f', rmse_afgs_disturb), sprintf('%.6f', rmse_smc_disturb), sprintf('%.2f', (rmse_smc_disturb - rmse_afgs_disturb)/rmse_smc_disturb * 100);
    'Disturbance ISE (m²·s)', sprintf('%.6f', ise_afgs_disturb), sprintf('%.6f', ise_smc_disturb), sprintf('%.2f', (ise_smc_disturb - ise_afgs_disturb)/ise_smc_disturb * 100);
};

% Create annotation instead of uitable (more compatible)
ax = axes('Position', [0.1 0.1 0.8 0.8], 'Visible', 'off');

% Create text for each cell
[n_rows, n_cols] = size(metrics_table);
cell_width = 0.8 / n_cols;
cell_height = 0.8 / n_rows;

for i = 1:n_rows
    for j = 1:n_cols
        x_pos = (j-1) * cell_width + 0.1;
        y_pos = 0.9 - i * cell_height;
        
        % Make header row bold
        if i == 1
            font_weight = 'bold';
            background_color = [0.8 0.8 0.9];
        else
            font_weight = 'normal';
            if mod(i,2) == 0
                background_color = [0.95 0.95 0.95];
            else
                background_color = [1 1 1];
            end
        end
        
        % Create background rectangle
        rectangle('Position', [x_pos, y_pos-cell_height+0.02, cell_width, cell_height-0.02], ...
            'FaceColor', background_color, 'EdgeColor', 'k');
        
        % Add text
        text(x_pos + cell_width/2, y_pos - cell_height/2, metrics_table{i,j}, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 9, 'FontWeight', font_weight);
    end
end

title('Performance Metrics Summary', 'FontSize', 14, 'FontWeight', 'bold', 'Position', [0.5 0.95]);

% =====================================================================
% Additional Plot: Tracking Error Components
% =====================================================================
figure('Position', [100 100 1200 600]);

% X tracking error
subplot(2,3,1);
plot(t_afgs, X_afgs(:,1)-xd_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, X_smc(:,1)-xd_smc, 'b--', 'LineWidth', 1.5);
ylabel('X Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('X Position Tracking Error');
grid on;
xlabel('Time (s)');

% Y tracking error
subplot(2,3,2);
plot(t_afgs, X_afgs(:,2)-yd_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, X_smc(:,2)-yd_smc, 'b--', 'LineWidth', 1.5);
ylabel('Y Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Y Position Tracking Error');
grid on;
xlabel('Time (s)');

% Z tracking error
subplot(2,3,3);
plot(t_afgs, X_afgs(:,3)-zd_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, X_smc(:,3)-zd_smc, 'b--', 'LineWidth', 1.5);
ylabel('Z Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title('Z Position Tracking Error');
grid on;
xlabel('Time (s)');

% RMS of X error over time (moving window)
window_size = 100; % points
rms_x_afgs = movsqrt(movmean((X_afgs(:,1)-xd_afgs).^2, window_size));
rms_x_smc = movsqrt(movmean((X_smc(:,1)-xd_smc).^2, window_size));

subplot(2,3,4);
plot(t_afgs, rms_x_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rms_x_smc, 'b--', 'LineWidth', 1.5);
ylabel('RMS X Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title(sprintf('Moving RMS of X Error (Window = %d points)', window_size));
grid on;
xlabel('Time (s)');

% RMS of Y error over time
rms_y_afgs = movsqrt(movmean((X_afgs(:,2)-yd_afgs).^2, window_size));
rms_y_smc = movsqrt(movmean((X_smc(:,2)-yd_smc).^2, window_size));

subplot(2,3,5);
plot(t_afgs, rms_y_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rms_y_smc, 'b--', 'LineWidth', 1.5);
ylabel('RMS Y Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title(sprintf('Moving RMS of Y Error (Window = %d points)', window_size));
grid on;
xlabel('Time (s)');

% RMS of total error over time
rms_total_afgs = movsqrt(movmean(pos_error_afgs.^2, window_size));
rms_total_smc = movsqrt(movmean(pos_error_smc.^2, window_size));

subplot(2,3,6);
plot(t_afgs, rms_total_afgs, 'k-', 'LineWidth', 1.5); hold on;
plot(t_smc, rms_total_smc, 'b--', 'LineWidth', 1.5);
ylabel('RMS Total Error (m)');
legend('AFGS-SMC', 'SMC', 'Location', 'best');
title(sprintf('Moving RMS of Total Position Error (Window = %d points)', window_size));
grid on;
xlabel('Time (s)');

sgtitle('Detailed Error Analysis');

% Helper function for moving RMS
function y = movsqrt(x)
    y = sqrt(x);
    y(isnan(y)) = 0;
end