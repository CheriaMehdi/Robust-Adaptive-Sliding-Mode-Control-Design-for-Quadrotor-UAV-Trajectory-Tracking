function dXdt = quadrotor_dynamics_disturb(t, X, fis, m, Ix, Iy, Iz, g, b, d, l, Jr,...
    lambda, k1_fixed, k2, kp_xy, kd_xy, kp_z, kd_z,controller_type)

% States
pos = X(1:3); att = X(4:6); vel = X(7:9); angvel = X(10:12);
phi = att(1); theta = att(2); psi = att(3);
dphi = angvel(1); dtheta = angvel(2); dpsi = angvel(3);

% Desired Trajectory
a = 10; b_val = 5; wz = 0.3;
xd = a*sin(wz*t); yd = b_val*sin(2*wz*t); zd = 4.5;
dxd = a*wz*cos(wz*t); dyd = 2*b_val*wz*cos(2*wz*t); dzd = 0;
ddxd = -a*wz^2*sin(wz*t); ddyd = -4*b_val*wz^2*sin(2*wz*t); ddzd = 0;

% Desired yaw = 0
psi_d = 0;

% Outer Loop: Feedforward acceleration + PD error correction
Ux = ddxd + kp_xy*(xd - pos(1)) + kd_xy*(dxd - vel(1));
Uy = ddyd + kp_xy*(yd - pos(2)) + kd_xy*(dyd - vel(2));
Uz = ddzd + kp_z*(zd - pos(3)) + kd_z*(dzd - vel(3));

% Safety clamps
Ux = clamp(Ux, -12, 12);
Uy = clamp(Uy, -12, 12);
Uz = clamp(Uz, -15, 15);

% Total thrust u4
ct = cos(phi)*cos(theta);
u4 = m * (Uz + g) / (ct + 1e-8);
u4 = clamp(u4, 0.05*m*g, 3.5*m*g);

% Desired roll and pitch
phi_d   = asin(clamp( m*(Ux*sin(psi_d) - Uy*cos(psi_d))/u4 , -0.95, 0.95));
theta_d = asin(clamp( m*(Ux*cos(psi_d) + Uy*sin(psi_d))/u4 / cos(phi_d + 1e-8) , -0.95, 0.95));

max_angle = pi/5.5;  % ~33 deg
phi_d   = clamp(phi_d,   -max_angle, max_angle);
theta_d = clamp(theta_d, -max_angle, max_angle);

% Attitude errors (desired rates = 0)
e_phi = phi - phi_d;     de_phi = dphi;
e_theta = theta - theta_d; de_theta = dtheta;
e_psi = psi - psi_d;     de_psi = dpsi;

% Sliding surfaces
s_phi   = de_phi   + lambda * e_phi;
s_theta = de_theta + lambda * e_theta;
s_psi   = de_psi   + lambda * e_psi;

% Boundary layer for smooth control
delta = 0.06;
sat_phi   = sat(s_phi,   delta);
sat_theta = sat(s_theta, delta);
sat_psi   = sat(s_psi,   delta);

% Adaptive or fixed switching gains
if strcmp(controller_type,'AFGS')
    k1_phi   = max(evalfis(fis, [s_phi; de_phi]), 0.1);
    k1_theta = max(evalfis(fis, [s_theta; de_theta]), 0.1);
    k1_psi   = max(evalfis(fis, [s_psi; de_psi]), 0.1);
else
    k1_phi = k1_fixed; k1_theta = k1_fixed; k1_psi = k1_fixed;
end

% Dynamics coefficients (from paper eq. 4)
a1 = (Iy - Iz)/Ix;   a2 = Jr/Ix;
a3 = (Iz - Ix)/Iy;   a4 = Jr/Iy;
a5 = (Ix - Iy)/Iz;

% Temporary torques (Omega_d = 0 first)
u1_temp = Ix * (-a1*dtheta*dpsi - lambda*de_phi - k1_phi*sat_phi - k2*s_phi);
u2_temp = Iy * (-a3*dphi*dpsi - lambda*de_theta - k1_theta*sat_theta - k2*s_theta);
u3 = Iz * (-a5*dphi*dtheta - lambda*de_psi - k1_psi*sat_psi - k2*s_psi);

% Rotor allocation to get Omega_d
U_temp = [u1_temp; u2_temp; u3; u4];
alloc_mat = [b, b, b, b;
             0, -b*l, 0, b*l;
             -b*l, 0, b*l, 0;
             -d, d, -d, d];
Omega_sq = alloc_mat \ U_temp;
Omega_sq = max(Omega_sq, 0);
Omega_d = -sqrt(Omega_sq(1)) + sqrt(Omega_sq(2)) - sqrt(Omega_sq(3)) + sqrt(Omega_sq(4));

% Final torques with gyro effect
u1 = Ix * (-a1*dtheta*dpsi - a2*dtheta*Omega_d - lambda*de_phi - k1_phi*sat_phi - k2*s_phi);
u2 = Iy * (-a3*dphi*dpsi - a4*dphi*Omega_d - lambda*de_theta - k1_theta*sat_theta - k2*s_theta);

% Translational accelerations
ddx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * (u4/m);
ddy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * (u4/m);
ddz = cos(phi)*cos(theta)*(u4/m) - g;

% Rotational accelerations
% ddphi   = a1*dtheta*dpsi + a2*dtheta*Omega_d + u1/Ix;
% ddtheta = a3*dphi*dpsi + a4*dphi*Omega_d + u2/Iy;
% ddpsi   = a5*dphi*dtheta + u3/Iz;
% Rotational accelerations WITH disturbances
ddphi   = a1*dtheta*dpsi + a2*dtheta*Omega_d + u1/Ix + Disturb1(t);  % Add disturbance here
ddtheta = a3*dphi*dpsi + a4*dphi*Omega_d + u2/Iy + Disturb2(t);      % Add disturbance here
ddpsi   = a5*dphi*dtheta + u3/Iz;

% Clamp angular accelerations for stability
ddphi   = clamp(ddphi, -60, 60);
ddtheta = clamp(ddtheta, -60, 60);
ddpsi   = clamp(ddpsi, -40, 40);

% State derivatives
dXdt = [vel;
        angvel;
        ddx; ddy; ddz;
        ddphi; ddtheta; ddpsi];

end

% Helper functions
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
function d1= Disturb1(t)
 if (t>=20&&t<=30)
     d1=20*cos(0.5*t);
 else
 d1=0;
 end
end 
function d2= Disturb2(t)
  if(t>=20&&t<=30)

     d2=20*cos(0.5*t);
  else
 d2=0;
  end
 end 

