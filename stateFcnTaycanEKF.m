function xk1 = stateFcnTaycanEKF(xk, uk)
% State function for Taycan vehicle dynamics model

% Inputs:

%   xk(1): Current vehicle speed (m/s)
%   xk(2): Force disturbance
%   uk(1): Force input command (N)
%   uk(2): White noise = Rate of change of force disturbance (N/s)

% Outputs:

%   xk1(1): Vehicle speed at next step (m/s)
%   xk1(2): Force disturbance at next step (N)

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
% Maximum force points (N) - decreases with speed
F_max_lu = [22000 1710];
% Minimum force (N) - maximum braking force
F_min = -20000;

% Extract current state and input
v = xk(1);  % Vehicle speed (m/s)
Fd = xk(2);  % Force disturbance (N)
F = uk(1);  % Force command (N)
dFd = uk(2); % White noise = Rate of change of force disturbance (N/s)
Ts = uk(3);

% Calculate speed-dependent maximum force limit
F_max = interp1(v_to_F_max_lu, F_max_lu, v, 'linear');

% Apply force limits
F = min(F, F_max);  % Limit to maximum force
F = max(F, F_min);  % Limit to minimum force

% Calculate acceleration (F = ma + drag)
dv = (F - v^2*b - Fd)/m;  % Net acceleration considering force and drag

xk1 = xk + Ts*[dv; dFd];

end

