function dXdt = quadrotor_dynamics_disturb(t, X, fis, m, Ix, Iy, Iz, g, b, d, l, Jr,...
    lambda, k1_fixed, k2, kp_xy, kd_xy, kp_z, kd_z, controller_type)

% States
pos = X(1:3); att = X(4:6); vel = X(7:9); angvel = X(10:12);
phi = att(1); theta = att(2); psi = att(3);
dphi = angvel(1); dtheta = angvel(2); dpsi = angvel(3);

% === Desired trajectory (paper) ===
T = 20; a = 10; b = 5; wz = 2*pi/T;
xd = a*sin(wz*t); yd = b*sin(2*wz*t); zd = 4.5;
dxd = a*wz*cos(wz*t); dyd = 2*b*wz*cos(2*wz*t);
ddxd = -a*wz^2*sin(wz*t); ddyd = -4*b*wz^2*sin(2*wz*t);

% === Outer loop PD (paper eq.21) ===
Ux = ddxd + kp_xy*(xd - pos(1)) + kd_xy*(dxd - vel(1));
Uy = ddyd + kp_xy*(yd - pos(2)) + kd_xy*(dyd - vel(2));
Uz = 0    + kp_z *(zd - pos(3)) + kd_z *(0    - vel(3));

Ux = clamp(Ux,-10,10); Uy = clamp(Uy,-10,10); Uz = clamp(Uz,-12,12);

% === Thrust u4 (paper eq.22) ===
cos_term = cos(phi)*cos(theta);
u4 = m * (Uz + g) / max(cos_term, 0.1);
u4 = clamp(u4, 0.1*m*g, 3.5*m*g);

% === Desired roll & pitch (paper eq.23-24, psi_d=0) ===
phi_d   = asin( clamp(Ux*m/u4, -0.95, 0.95) );
theta_d = asin( clamp(Uy*m/u4 / cos(phi_d), -0.95, 0.95) );
phi_d   = clamp(phi_d,   -pi/4, pi/4);
theta_d = clamp(theta_d, -pi/4, pi/4);

% === Attitude errors ===
e_phi = phi - phi_d;     de_phi = dphi;
e_theta = theta - theta_d; de_theta = dtheta;
e_psi = psi;             de_psi = dpsi;

% === Sliding surfaces (paper eq.12) ===
s_phi   = de_phi   + lambda*e_phi;
s_theta = de_theta + lambda*e_theta;
s_psi   = de_psi   + lambda*e_psi;

% === Scale inputs to FIS [-2 2] (no warning) ===
scale = 0.4;
s_phi_in   = [s_phi;   de_phi]   * scale;
s_theta_in = [s_theta; de_theta] * scale;
s_psi_in   = [s_psi;   de_psi]   * scale;

% === Switching gain k1 (this is where they differ!) ===
if strcmp(controller_type, 'AFGS')
    k1_phi   = max(evalfis(fis, s_phi_in),   0.1);
    k1_theta = max(evalfis(fis, s_theta_in), 0.1);
    k1_psi   = max(evalfis(fis, s_psi_in),   0.1);
else
    k1_phi = k1_fixed; k1_theta = k1_fixed; k1_psi = k1_fixed;
end

% === Boundary layer (smooth) ===
delta = 0.06;
sat_phi   = sat(s_phi,   delta);
sat_theta = sat(s_theta, delta);
sat_psi   = sat(s_psi,   delta);

% === Coefficients (paper eq.4) ===
a1 = (Iy - Iz)/Ix; a2 = Jr/Ix;
a3 = (Iz - Ix)/Iy; a4 = Jr/Iy;
a5 = (Ix - Iy)/Iz;

% === Torques (paper eq.15) ===
u1_temp = Ix * (-a1*dtheta*dpsi - lambda*de_phi - k1_phi*sat_phi - k2*s_phi);
u2_temp = Iy * (-a3*dphi*dpsi   - lambda*de_theta - k1_theta*sat_theta - k2*s_theta);
u3      = Iz * (                - lambda*de_psi - k1_psi*sat_psi - k2*s_psi - a5*dphi*dtheta);

% === Rotor allocation (paper eq.7) ===
U_temp = [u4; u1_temp; u2_temp; u3];
alloc = [b  b  b  b;
         0 -l*b 0 l*b;
        -l*b 0 l*b 0;
        -d  d -d  d];
Omega_sq = alloc \ U_temp;
Omega_sq = max(Omega_sq, 0);
Omega_d = -sqrt(Omega_sq(1)) + sqrt(Omega_sq(2)) - sqrt(Omega_sq(3)) + sqrt(Omega_sq(4));

% === Final torques with gyro ===
u1 = Ix * (-a1*dtheta*dpsi - a2*dtheta*Omega_d - lambda*de_phi - k1_phi*sat_phi - k2*s_phi);
u2 = Iy * (-a3*dphi*dpsi   - a4*dphi*Omega_d   - lambda*de_theta - k1_theta*sat_theta - k2*s_theta);

% === Translational accelerations (paper eq.4) ===
ddx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * (u4/m);
ddy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * (u4/m);
ddz = cos(phi)*cos(theta)*(u4/m) - g;

% === Rotational accelerations (paper eq.4) ===
ddphi   = a1*dtheta*dpsi + a2*dtheta*Omega_d + u1/Ix;
ddtheta = a3*dphi*dpsi   + a4*dphi*Omega_d   + u2/Iy;
ddpsi   = a5*dphi*dtheta + u3/Iz;

% === EXTERNAL DISTURBANCES ===
% Wind gust in X (t=6-12s)
if t > 6 && t < 12
    ddx = ddx + 0.5;  % strong wind
end
% Torque gust on roll (t=15-18s)
if t > 15 && t < 18
    ddphi = ddphi + 12;
end

% Clamp
ddphi   = clamp(ddphi,   -80, 80);
ddtheta = clamp(ddtheta, -80, 80);
ddpsi   = clamp(ddpsi,   -60, 60);

% === Output ===
dXdt = [vel; angvel; ddx; ddy; ddz; ddphi; ddtheta; ddpsi];
end

% === Helpers ===
function y = clamp(x, lo, hi)
    y = max(min(x, hi), lo);
end
function y = sat(s, delta)
    if abs(s) <= delta
        y = s / delta;
    else
        y = sign(s);
    end
end