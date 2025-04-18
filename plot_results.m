% Access the signals from out.logsout

speed = out.logsout.get('speed').Values.Data;
t_speed = out.logsout.get('speed').Values.Time;

speed_ref = out.logsout.get('speed_ref').Values.Data;
t_speed_ref = out.logsout.get('speed_ref').Values.Time;

speed_meas = out.logsout.get('speed_meas').Values.Data;
t_speed_meas = out.logsout.get('speed_meas').Values.Time;

cost = out.logsout.get('cost').Values.Data;
t_cost = out.logsout.get('cost').Values.Time;

force = out.logsout.get('force').Values.Data;
t_force = out.logsout.get('force').Values.Time;

theta = out.logsout.get('theta').Values.Data;
t_theta = out.logsout.get('theta').Values.Time;

xhat = out.logsout.get('xhat').Values.Data;
t_xhat = out.logsout.get('xhat').Values.Time;

% Create the first figure
figure;

% Subplot for speed
subplot(3, 1, 1);
plot(t_speed, speed, 'LineWidth', 2);
hold on;
plot(t_speed_meas, speed_meas, 'LineWidth', 2);
stairs(t_speed_ref, speed_ref, '--', 'LineWidth', 2);
stairs(t_xhat, xhat(:, 1), 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Speed (m/s)');
legend({'Actual', 'Measured', 'Setpoint', 'Estimated - EKF'}, 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Subplot for force
subplot(3, 1, 2);
stairs(t_force, force, 'LineWidth', 2);
hold on;
stairs(t_xhat, xhat(:, 2), 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Force (N)');
legend({'Control Input', 'Estimated Disturbance - EKF'}, 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Subplot for theta
subplot(3, 1, 3);
stairs(t_theta, theta, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (deg)');
set(gca, 'FontSize', 12);
grid on;