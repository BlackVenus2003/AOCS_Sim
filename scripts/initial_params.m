function [r0, v0, q0, omega0, q_desired, T_external, F_external] = initial_conditions()

% Initial orbital position (e.g., 700 km circular orbit in ECI frame)
r0 = [7000; 0; 0];          % km
v0 = [0; 7.5; 0];           % km/s (rough circular speed)

% Initial attitude (slightly off identity quaternion)
q0 = [0.999; 0.01; 0.01; 0.01];  % Should be normalized

% Initial angular velocity (rad/s)
omega0 = [0.01; 0.005; -0.01];   % Small rotation rate

% Desired attitude
q_desired = [1; 0; 0; 0];  % No rotation (identity quaternion)

% External disturbance torque (Nm)
T_external = [0.0001; 0.0001; 0.0002];

% External force (e.g., drag, solar radiation pressure) (N)
F_external = [0; 0; 0];  % Start with 0, can be added later

end
