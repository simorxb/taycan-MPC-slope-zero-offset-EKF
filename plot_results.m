% Access the signals from out.logsout

speed = out.logsout.get('speed').Values.Data;
t_speed = out.logsout.get('speed').Values.Time;

speed_ref = out.logsout.get('speed_ref').Values.Data;
t_speed_ref = out.logsout.get('speed_ref').Values.Time;

cost = out.logsout.get('cost').Values.Data;
t_cost = out.logsout.get('cost').Values.Time;

force = out.logsout.get('force').Values.Data;
t_force = out.logsout.get('force').Values.Time;

theta = out.logsout.get('theta').Values.Data;
t_theta = out.logsout.get('theta').Values.Time;

% Create the first figure
figure;

% Subplot for speed
subplot(3, 1, 1);
plot(t_speed, speed, 'LineWidth', 2);
hold on;
stairs(t_speed_ref, speed_ref, '--', 'LineWidth', 2);
hold off;
%xlabel('Time (s)');
ylabel('Speed (m/s)');
legend({'Measured', 'Setpoint'}, 'FontSize', 12);
set(gca, 'FontSize', 12);
grid on;

% Subplot for force
subplot(3, 1, 2);
stairs(t_force, force, 'LineWidth', 2);
%xlabel('Time (s)');
ylabel('Force (N)');
set(gca, 'FontSize', 12);
grid on;

% Subplot for theta
subplot(3, 1, 3);
stairs(t_theta, theta, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (deg)');
set(gca, 'FontSize', 12);
grid on;