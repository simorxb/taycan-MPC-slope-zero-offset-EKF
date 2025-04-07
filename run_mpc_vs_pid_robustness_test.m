% This scripts should be run after init.m
init;

% Select reference
ref = 1;

% Noise power
noise_power = 0;

% PID tuning
kp = 2200;
ki = 300;

% EKF tuning
cov_proc = diag([0 1e6]);
cov_meas = 1;

% Adjust MPC tuning to obtain a behaviour comparable to the PID
nlobj.Weights.ManipulatedVariablesRate = 0.0001;

% Define array of masses for uncertainty mapping (+/- 40%)
m_V = [0.6 1.4]*m;

% Road slope angle (degrees)
theta_V = 0;

% Reference speed trajectory points (m/s)
speed_ref_V = [10 50 60 10];

% Simulation time
t_Stop = 80;

% Controller types (1 = MPC, 2 = PID)
controllers = [1, 2];
controller_names = {'MPC', 'PID'};

% Number of parameter combinations
num_m = length(m_V);

% Initialize cell arrays to store simulation results for all combinations
out = cell(length(controllers), num_m);
speed = cell(length(controllers), num_m);
t_speed = cell(length(controllers), num_m);
speed_ref = cell(length(controllers), num_m);
t_speed_ref = cell(length(controllers), num_m);
force = cell(length(controllers), num_m);
t_force = cell(length(controllers), num_m);
theta = cell(length(controllers), num_m);
t_theta = cell(length(controllers), num_m);

% Run simulations for each controller type and parameter combination
for ctrl_idx = 1:length(controllers)
    for m_idx = 1:num_m
        % Set controller type
        controller = controllers(ctrl_idx);
        
        % Set parameters for the current combination
        m = m_V(m_idx);
        
        % Run simulation and store results
        fprintf('Running simulation with %s controller, m = %.2f...\n', controller_names{ctrl_idx}, m);
        out{ctrl_idx, m_idx} = sim('model_with_PID');
        
        % Extract time series data from simulation results
        % Vehicle speed
        speed{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('speed').Values.Data;
        t_speed{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('speed').Values.Time;
        
        % Reference speed trajectory
        speed_ref{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('speed_ref').Values.Data;
        t_speed_ref{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('speed_ref').Values.Time;
        
        % Control force
        force{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('force').Values.Data;
        t_force{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('force').Values.Time;
        
        % Slope disturbance
        theta{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('theta').Values.Data;
        t_theta{ctrl_idx, m_idx} = out{ctrl_idx, m_idx}.logsout.get('theta').Values.Time;
    end
end

%% Plot comparison of results between MPC and PID for all combinations
figure;

colors = {'b', 'r', 'g', 'm'}; % Define colors for different simulations

% Speed tracking performance
subplot(2, 1, 1);
hold on;
for ctrl_idx = 1:length(controllers)
    for m_idx = 1:num_m
        plot(t_speed{ctrl_idx, m_idx}, speed{ctrl_idx, m_idx}, colors{(ctrl_idx-1)*num_m + m_idx}, 'LineWidth', 2);
    end
end
stairs(t_speed_ref{1, 1}, speed_ref{1, 1}, 'k--', 'LineWidth', 2);
hold off;
ylabel('Speed (m/s)');
legend_entries = {};
for i = 1:length(controllers)
    for j = 1:num_m
        legend_entries{end+1} = sprintf('%s (m=%.2f)', controller_names{i}, m_V(j));
    end
end
legend_entries{end+1} = 'Setpoint';
legend(legend_entries, 'FontSize', 8, 'Location', 'best');
set(gca, 'FontSize', 12);
grid on;
title('Speed Tracking Performance');

% Control effort
subplot(2, 1, 2);
hold on;
for ctrl_idx = 1:length(controllers)
    for m_idx = 1:num_m
        stairs(t_force{ctrl_idx, m_idx}, force{ctrl_idx, m_idx}, colors{(ctrl_idx-1)*num_m + m_idx}, 'LineWidth', 2);
    end
end
hold off;
ylabel('Force (N)');
legend(legend_entries(1:end-1), 'FontSize', 8, 'Location', 'best');
set(gca, 'FontSize', 12);
grid on;
title('Control Effort');
xlabel('Time (s)');
