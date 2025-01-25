function y = outputFcnTaycan(x, u)
% Output function for Taycan vehicle dynamics model
% This function defines the measured output for the nonlinear MPC controller
% The only measured output is the vehicle speed

% Inputs:
%   x: State vector
%       x(1): Current vehicle speed (m/s)
%       x(2): Force disturbance (N)
%   u: Input vector  
%       u(1): Force input command (N)
%       u(2): White noise disturbance - Rate of change of force disturbance (N/s)

% Outputs:
%   y: Measured output
%       y = x(1): Vehicle speed (m/s)

y = x(1);

end

