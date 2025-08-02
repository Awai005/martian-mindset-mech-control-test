function plot_theta_vs_time(t, theta, tau, params)
% Plots theta (angle) and torque over time
figure;

% Subplot for theta
subplot(2, 1, 1);
plot(t, theta, 'LineWidth', 2);
hold on;
if strcmp(params.mode, 'swing_up')
    plot(t, pi * ones(size(t)), 'r--', 'LineWidth', 1.5); % Target theta = pi
end
xlabel('Time (s)');
ylabel('\theta (rad)');
if strcmp(params.mode, 'free_fall')
    title('Pendulum Free-Fall with Damping (b = 0.1, \tau = 0)');
    legend('\theta');
else
    title('Pendulum Swing-Up with Damping (b = 0.1, |\tau| \leq 1 Nm)');
    legend('\theta', 'Target (\pi)');
end
grid on;

% Subplot for torque
subplot(2, 1, 2);
plot(t, tau, 'LineWidth', 2, 'Color', [0, 0.5, 0]);
hold on;
if strcmp(params.mode, 'swing_up')
    plot(t, 1 * ones(size(t)), 'k--', 'LineWidth', 1); % Upper torque limit
    plot(t, -1 * ones(size(t)), 'k--', 'LineWidth', 1); % Lower torque limit
end
xlabel('Time (s)');
ylabel('\tau (Nm)');
title('Control Torque vs Time');
if strcmp(params.mode, 'swing_up')
    legend('\tau', 'Torque Limits (\pm 1 Nm)');
else
    legend('\tau');
end
grid on;

% Adjust layout
sgtitle('Pendulum Simulation Results');
end
