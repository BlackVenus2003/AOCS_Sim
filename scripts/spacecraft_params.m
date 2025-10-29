function [I, I_rw, tau_max_rw, omega_rw_max, h_max_rw, ...
          tau_thruster, tau_thruster_min, tau_thruster_max, ...
          dt_min_thruster, mass_total] = spacecraft_parameters()

% Spacecraft Moment of Inertia (kg·m²)
I = diag([2.5, 2.0, 1.8]);  % Customize per spacecraft

% Reaction Wheel Parameters
I_rw = [0.02; 0.02; 0.02];                    % Reaction wheel inertias
tau_max_rw = [0.1; 0.1; 0.1];                 % Max torque per RW (Nm)
omega_rw_max = [6000; 6000; 6000] * 2*pi/60;                  % Max RW speed (rad/s)
h_max_rw = I_rw .* omega_rw_max;             % Max angular momentum storage

% Thruster Parameters
tau_thruster = [0; 0; 0.5];      % Constant torque or command placeholder
tau_thruster_min = 0.01;         % Minimum on-time torque (Nm)
tau_thruster_max = 1.0;          % Max safe torque (Nm)
dt_min_thruster = 0.1;           % Min pulse time (s)

% Spacecraft Total Mass
mass_total = 50;  % kg

end
