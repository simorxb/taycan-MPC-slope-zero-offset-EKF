function y = outputFcnTaycan(x, u)
% State function for Taycan vehicle dynamics model

% Inputs:

%   x(1): Current vehicle speed (m/s)
%   x(2): Force disturbance
%   u(1): Force input command (N)
%   u(2): White noise = Rate of change of force disturbance (N/s)

% Outputs:

%   dxdt(1): Rate of change of vehicle speed (m/s^2)
%   dxdt(2): Rate of change of force disturbance (N/s)

y = x(1);

end

