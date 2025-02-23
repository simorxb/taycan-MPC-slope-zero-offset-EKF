% This scripts should be run after init.m
init;

% Define array of masses for uncertainty mapping (+/- 25%)
m_V = [0.75 1 1.25]*m;

% Define array of drag for uncertainty mapping (+/- 25%)
b_V = [0.75 1 1.25]*b;

% Road slope angle (degrees)
theta_V = 0;

% Reference speed trajectory points (m/s)
speed_ref_V = [10 50 60 10];

% Initialize cell arrays to store simulation results
out = cell(length(m_V), length(b_V));

% Pre-allocate cell arrays for time series data
speed = cell(length(m_V), length(b_V));
t_speed = cell(length(m_V), length(b_V));
speed_ref = cell(length(m_V), length(b_V));
t_speed_ref = cell(length(m_V), length(b_V)); 
force = cell(length(m_V), length(b_V));
t_force = cell(length(m_V), length(b_V));
xhat = cell(length(m_V), length(b_V));
t_xhat = cell(length(m_V), length(b_V));

% Calculate total number of simulations
total_sims = length(m_V) * length(b_V);
sim_count = 0;

% Run simulations for each mass and drag value combination
for m_idx = 1:length(m_V)
    for b_idx = 1:length(b_V)
        % Update mass and drag
        m = m_V(m_idx);
        b = b_V(b_idx);

        % Run simulation and store results
        out{m_idx,b_idx} = sim('model');
        
        % Update progress counter
        sim_count = sim_count + 1;
        fprintf('Simulation progress: %.1f%% (%d/%d)\n', ...
            100*sim_count/total_sims, sim_count, total_sims);

        % Extract time series data from simulation results
        % Vehicle speed
        speed{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('speed').Values.Data;
        t_speed{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('speed').Values.Time;

        % Reference speed trajectory
        speed_ref{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('speed_ref').Values.Data;
        t_speed_ref{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('speed_ref').Values.Time;

        % Control force
        force{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('force').Values.Data;
        t_force{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('force').Values.Time;
        
        % EKF output
        xhat{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('xhat').Values.Data;
        t_xhat{m_idx,b_idx} = out{m_idx,b_idx}.logsout.get('xhat').Values.Time;
    end
end

% Plot comparison of results for different weights and drag values
figure;

% Speed tracking performance
subplot(3, 1, 1);
hold on;
for m_idx = 1:length(m_V)
    for b_idx = 1:length(b_V)
        plot(t_speed{m_idx,b_idx}, speed{m_idx,b_idx}, 'LineWidth', 2);
    end
end
stairs(t_speed_ref{1,1}, speed_ref{1,1}, '--', 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Speed (m/s)');
legend_labels = {};
for m_idx = 1:length(m_V)
    for b_idx = 1:length(b_V)
        legend_labels{end+1} = sprintf('Mass = %.0f kg, Drag = %.2f Ns/m', m_V(m_idx), b_V(b_idx));
    end
end
legend([legend_labels, 'Setpoint'], 'FontSize', 8);
set(gca, 'FontSize', 12);
grid on;

% Control effort
subplot(3, 1, 2);
hold on;
for m_idx = 1:length(m_V)
    for b_idx = 1:length(b_V)
        stairs(t_force{m_idx,b_idx}, force{m_idx,b_idx}, 'LineWidth', 2);
    end
end
hold off;
%xlabel('Time (s)');
ylabel('Force (N)');
legend(legend_labels, 'FontSize', 8);
set(gca, 'FontSize', 12);
grid on;

% Force disturbance estimation
subplot(3, 1, 3);
hold on;
for m_idx = 1:length(m_V)
    for b_idx = 1:length(b_V)
        stairs(t_xhat{m_idx,b_idx}, xhat{m_idx,b_idx}(:, 2), 'LineWidth', 2);
    end
end
hold off;
xlabel('Time (s)');
ylabel('Force Disturbance (N)');
legend(legend_labels, 'FontSize', 8);
set(gca, 'FontSize', 12);
grid on;