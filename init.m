%% Vehicle Physical Parameters
% Mass of the vehicle (kg)
m = 2140;
% Drag coefficient * Frontal area (m^2)
CdA = 0.513;
% Air density (kg/m^3)
rho = 1.293;

% Combined aerodynamic drag coefficient
b = 0.5*CdA*rho;

% Lookup table for maximum force vs speed
% Speed points (m/s)
v_to_F_max = [0 72];
% Maximum force points (N) - decreases with speed
F_max = [22000 1710];
% Minimum force (N) - maximum braking force
F_min = -20000;

% Gravity
g = 9.81;

% Simulation sample time
Tsim = 0.2;

%% Nonlinear Model Predictive Controller Configuration

% Create nonlinear MPC controller object
nlobj = nlmpc(2 , 1, 'MV', 1, 'UD', 2);

% Controller sample time (seconds)
Ts = 1;
nlobj.Ts = Ts;

% Controller horizons
nlobj.PredictionHorizon = 20; % Look ahead 20 seconds
nlobj.ControlHorizon = 10;    % Optimize control for 10 seconds

% Set vehicle dynamics model
nlobj.Model.StateFcn = "stateFcnTaycan";
nlobj.Model.OutputFcn = "outputFcnTaycan";

% Input constraints based on physical limits
nlobj.ManipulatedVariables.Max = F_max(1);  % Maximum force
nlobj.ManipulatedVariables.Min = F_min;     % Minimum force

% Cost function weights
nlobj.Weights.OutputVariables = 1;          % Track reference trajectory
nlobj.Weights.ManipulatedVariables = 0;     % No penalty on force magnitude
nlobj.Weights.ManipulatedVariablesRate = 0.002; % Small penalty on force changes

% Initial conditions for model validation
x0 = [10; 0];    % Initial speed (m/s)
u0 = 500;   % Initial force (N)
validateFcns(nlobj, x0, u0, [], {});