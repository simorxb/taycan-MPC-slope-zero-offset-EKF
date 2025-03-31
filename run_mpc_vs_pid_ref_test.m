% This scripts should be run after init.m
init;

% Select reference
ref = 2;

% PID tuning
kp = 2200;
ki = 300;

% EKF tuning
cov_proc = diag([0 1e6]);
cov_meas = 1;

% Adjust MPC tuning to obtain a behaviour comparable to the PID
nlobj.Weights.ManipulatedVariablesRate = 0.0001;

% Road slope angle (degrees)
theta_V = [0 0 0 0 0 -15 0 15];

% Reference speed trajectory points (m/s)
speed_ref_V = [10 50 60 10 30 30 30 30];

% Controller types (1 = MPC, 2 = PID)
controllers = [1, 2];
controller_names = {'MPC', 'PID'};

% Initialize cell arrays to store simulation results
out = cell(1, length(controllers));

% Pre-allocate cell arrays for time series data
speed = cell(1, length(controllers));
t_speed = cell(1, length(controllers));
speed_ref = cell(1, length(controllers));
t_speed_ref = cell(1, length(controllers)); 
force = cell(1, length(controllers));
t_force = cell(1, length(controllers));
theta = cell(1, length(controllers));
t_theta = cell(1, length(controllers));

% Run simulations for each controller type
for ctrl_idx = 1:length(controllers)
    % Set controller type
    controller = controllers(ctrl_idx);
    
    % Run simulation and store results
    fprintf('Running simulation with %s controller...\n', controller_names{ctrl_idx});
    out{ctrl_idx} = sim('model_with_PID');
    
    % Extract time series data from simulation results
    % Vehicle speed
    speed{ctrl_idx} = out{ctrl_idx}.logsout.get('speed').Values.Data;
    t_speed{ctrl_idx} = out{ctrl_idx}.logsout.get('speed').Values.Time;

    % Reference speed trajectory
    speed_ref{ctrl_idx} = out{ctrl_idx}.logsout.get('speed_ref').Values.Data;
    t_speed_ref{ctrl_idx} = out{ctrl_idx}.logsout.get('speed_ref').Values.Time;

    % Control force
    force{ctrl_idx} = out{ctrl_idx}.logsout.get('force').Values.Data;
    t_force{ctrl_idx} = out{ctrl_idx}.logsout.get('force').Values.Time;
    
    % Slope disturbance
    theta{ctrl_idx} = out{ctrl_idx}.logsout.get('theta').Values.Data;
    t_theta{ctrl_idx} = out{ctrl_idx}.logsout.get('theta').Values.Time;
end

% Plot comparison of results between MPC and PID
figure;

% Speed tracking performance
subplot(3, 1, 1);
hold on;
colors = {'b', 'r'};
for ctrl_idx = 1:length(controllers)
    plot(t_speed{ctrl_idx}, speed{ctrl_idx}, colors{ctrl_idx}, 'LineWidth', 2);
end
stairs(t_speed_ref{1}, speed_ref{1}, 'k--', 'LineWidth', 2);
hold off;
ylabel('Speed (m/s)');
legend([controller_names, 'Setpoint'], 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 12);
grid on;
title('Speed Tracking Performance');

% Control effort
subplot(3, 1, 2);
hold on;
for ctrl_idx = 1:length(controllers)
    stairs(t_force{ctrl_idx}, force{ctrl_idx}, colors{ctrl_idx}, 'LineWidth', 2);
end
hold off;
ylabel('Force (N)');
legend(controller_names, 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 12);
grid on;
title('Control Effort');

% Slope disturbance (only need to plot once as it's the same)
subplot(3, 1, 3);
stairs(t_theta{1}, theta{1}, 'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (deg)');
set(gca, 'FontSize', 12);
grid on;
title('Slope Disturbance');

% Plot comparison of results between MPC and PID (ref test)
figure;

% Speed tracking performance (error)
subplot(2, 1, 1);
hold on;
colors = {'b', 'r'};
for ctrl_idx = 1:length(controllers)
    plot(t_speed{ctrl_idx}, speed_ref{ctrl_idx} - speed{ctrl_idx}, colors{ctrl_idx}, 'LineWidth', 2);
end
% stairs(t_speed_ref{1}, speed_ref{1}, 'k--', 'LineWidth', 2);
hold off;
ylabel('Speed (m/s)');
legend([controller_names], 'FontSize', 10, 'Location', 'best');
set(gca, 'FontSize', 12);
grid on;
title('Speed Tracking Performance (Error)');

% Slope disturbance (only need to plot once as it's the same)
subplot(2, 1, 2);
stairs(t_theta{1}, theta{1}, 'k', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (deg)');
set(gca, 'FontSize', 12);
grid on;
title('Slope Disturbance');