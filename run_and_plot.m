% Define array of weights for manipulated variable rate penalty
w_V = [0.0001 0.002 0.007];

% Road slope angle (degrees)
theta_V = 0;

% Reference speed trajectory points (m/s)
speed_ref_V = [10 50 60 10];

% Initialize cell arrays to store simulation results
out = cell(1, length(w_V));

% Pre-allocate cell arrays for time series data
speed = cell(1, length(w_V));
t_speed = cell(1, length(w_V));
speed_ref = cell(1, length(w_V));
t_speed_ref = cell(1, length(w_V)); 
force = cell(1, length(w_V));
t_force = cell(1, length(w_V));

% Run simulations for each weight value
for sim_idx = 1:length(w_V)

    % Update MPC controller weight
    nlobj.Weights.ManipulatedVariablesRate = w_V(sim_idx);

    % Run simulation and store results
    out{sim_idx} = sim('model');

    % Extract time series data from simulation results
    % Vehicle speed
    speed{sim_idx} = out{sim_idx}.logsout.get('speed').Values.Data;
    t_speed{sim_idx} = out{sim_idx}.logsout.get('speed').Values.Time;

    % Reference speed trajectory
    speed_ref{sim_idx} = out{sim_idx}.logsout.get('speed_ref').Values.Data;
    t_speed_ref{sim_idx} = out{sim_idx}.logsout.get('speed_ref').Values.Time;

    % Control force
    force{sim_idx} = out{sim_idx}.logsout.get('force').Values.Data;
    t_force{sim_idx} = out{sim_idx}.logsout.get('force').Values.Time;

end

% Plot comparison of results for different weights
figure;

% Top subplot: Speed tracking performance
subplot(2, 1, 1);
hold on;
for sim_idx = 1:length(w_V)
    plot(t_speed{sim_idx}, speed{sim_idx}, 'LineWidth', 2);
end
stairs(t_speed_ref{1}, speed_ref{1}, '--', 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Speed (m/s)');
legend([arrayfun(@(x) sprintf('MV Rate Weight = %.4f', w_V(x)), 1:length(w_V), 'UniformOutput', false), 'Setpoint'], 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Bottom subplot: Control effort
subplot(2, 1, 2);
hold on;
for sim_idx = 1:length(w_V)
    stairs(t_force{sim_idx}, force{sim_idx}, 'LineWidth', 2);
end
hold off;
%xlabel('Time (s)');
ylabel('Force (N)');
legend(arrayfun(@(x) sprintf('MV Rate Weight = %.4f', w_V(x)), 1:length(w_V), 'UniformOutput', false), 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;