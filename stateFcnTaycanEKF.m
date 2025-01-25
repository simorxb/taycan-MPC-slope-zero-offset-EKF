function xk1 = stateFcnTaycanEKF(xk, uk)
% State function for Taycan vehicle dynamics model
% Discrete-time state equations for Extended Kalman Filter implementation
%
% Inputs:
%   xk: State vector at current time step k
%       xk(1): Current vehicle speed (m/s)
%       xk(2): Force disturbance (N)
%   uk: Input vector at current time step k
%       uk(1): Force input command (N)
%       uk(2): Process noise - Rate of change of force disturbance (N/s)
%       uk(3): Sample time (s)
%
% Outputs:
%   xk1: State vector at next time step k+1
%       xk1(1): Vehicle speed at next step (m/s)
%       xk1(2): Force disturbance at next step (N)

%% Vehicle Parameters
% Vehicle mass (kg)
m = 2140;
% Drag coefficient * Frontal area (m^2)
CdA = 0.513;
% Air density (kg/m^3)
rho = 1.293;

% Combined aerodynamic drag coefficient (kg/m)
b = 0.5*CdA*rho;

% Lookup table for maximum force vs speed
% Speed points (m/s)
v_to_F_max_lu = [0 72];
% Maximum force points (N) - decreases with speed due to power limit
F_max_lu = [22000 1710];
% Minimum force (N) - maximum braking force
F_min = -20000;

% Extract current state and input
v = xk(1);    % Vehicle speed (m/s)
Fd = xk(2);   % Force disturbance (N)
F = uk(1);    % Force command (N)
dFd = uk(2);  % Process noise - Rate of change of force disturbance (N/s)
Ts = uk(3);   % Sample time (s)

% Calculate speed-dependent maximum force limit using linear interpolation
F_max = interp1(v_to_F_max_lu, F_max_lu, v, 'linear');

% Apply force limits to respect physical constraints
F = min(F, F_max);  % Limit to maximum force
F = max(F, F_min);  % Limit to minimum force

% Calculate acceleration using F = ma + drag + disturbance
dv = (F - v^2*b - Fd)/m;  % Net acceleration (m/s^2)

% Discrete-time state update using Euler integration
xk1 = xk + Ts*[dv; dFd];

end
