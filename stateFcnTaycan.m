function dxdt = stateFcnTaycan(x, u)
% State function for Taycan vehicle dynamics model
%
% Inputs:
%   x: State vector
%       x(1): Current vehicle speed (m/s)
%       x(2): Force disturbance (N)
%   u: Input vector
%       u(1): Force input command (N) 
%       u(2): Rate of change of force disturbance (N/s)
%
% Outputs:
%   dxdt: State derivative vector
%       dxdt(1): Rate of change of vehicle speed (m/s^2)
%       dxdt(2): Rate of change of force disturbance (N/s)

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
v = x(1);    % Vehicle speed (m/s)
Fd = x(2);   % Force disturbance (N)
F = u(1);    % Force command (N)
dFd = u(2);  % Rate of change of force disturbance (N/s)

% Calculate speed-dependent maximum force limit using linear interpolation
F_max = interp1(v_to_F_max_lu, F_max_lu, v, 'linear');

% Apply force limits to respect physical constraints
F = min(F, F_max);  % Limit to maximum force
F = max(F, F_min);  % Limit to minimum force

% Calculate acceleration using F = ma + drag + disturbance
dv = (F - v^2*b - Fd)/m;  % Net acceleration (m/s^2)

dxdt = [dv; dFd];

end

