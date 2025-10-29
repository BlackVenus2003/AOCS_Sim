function [q_est, omega_est] = attitude_ekf(q_meas, omega_meas, s_meas, B_meas)

% ======= Input Guards =======
if isempty(q_meas) || norm(q_meas) == 0 || any(isnan(q_meas))
    q_meas = [1; 0; 0; 0];  % fallback to identity quaternion
end

if isempty(omega_meas) || any(isnan(omega_meas))
    omega_meas = [0; 0; 0];
end

% ======= Persistent Variables =======
persistent x P
dt = 0.01;

if isempty(x)
    x = [q_meas; omega_meas];     % Initial state
    P = 1e-2 * eye(7);             % Initial covariance
end

% ======= Prediction Step =======
q = x(1:4);
omega = x(5:7);

% Quaternion derivative using Omega matrix
Omega = [0, -omega';
         omega, -skew(omega)];

q_dot = 0.5 * Omega * q;
q_pred = q + dt * q_dot;

% Normalize prediction
if norm(q_pred) == 0 || any(isnan(q_pred))
    q_pred = [1; 0; 0; 0];
else
    q_pred = q_pred / norm(q_pred);
end

omega_pred = omega;

% Jacobians (simplified)
F = eye(7);
Q = 1e-4 * eye(7);

x_pred = [q_pred; omega_pred];
P = F * P * F' + Q;

% ======= Measurement Update =======
H = eye(7);                 % Direct observation
R = 1e-3 * eye(7);           % Measurement noise

z = [q_meas; omega_meas];  % Measurement vector
z_pred = x_pred;           % Predicted measurement

% Kalman Gain
K = P * H' / (H * P * H' + R);

% EKF Update
x = x_pred + K * (z - z_pred);

% Normalize quaternion again
if norm(x(1:4)) == 0 || any(isnan(x(1:4)))
    x(1:4) = [1; 0; 0; 0];
else
    x(1:4) = x(1:4) / norm(x(1:4));
end

P = (eye(7) - K * H) * P;

% ======= Outputs =======
q_est = x(1:4);
omega_est = x(5:7);

end

% ======= Helper Function =======
function S = skew(w)
S = [ 0    -w(3)  w(2);
      w(3)  0    -w(1);
     -w(2)  w(1)  0   ];
end
