% pendulum_freefall.m
% Simulate pendulum free-fall using Euler integration and visualize

clear; clc;

%% Parameters
g = 9.81;       % gravitational acceleration (m/s^2)
l = 1.0;        % length of the pendulum (m)
m = 1.0;        % mass (not used since tau = 0)
tau = 0.0;      % torque = 0 for free fall

% Simulation settings
dt = 0.01;      % time step (s)
T = 20;         % total simulation time (s)
N = round(T/dt);% number of steps
t = linspace(0, T, N);

% Initial conditions
theta = pi/4;       % initial angle (45 degrees)
theta_dot = 0;      % initial angular velocity

% Data storage
theta_history = zeros(1, N);
theta_dot_history = zeros(1, N);

%% Euler Integration Loop
for i = 1:N
    % Store values
    theta_history(i) = theta;
    theta_dot_history(i) = theta_dot;

    % Compute angular acceleration using forward dynamics
    theta_ddot = - (g / l) * sin(theta);   % τ = 0

    % Update using Euler integration
    theta_dot = theta_dot + theta_ddot * dt;
    theta = theta + theta_dot * dt;
end

%% Plot Angle vs Time
figure;
plot(t, theta_history, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Pendulum Free-Fall (τ = 0)');
grid on;

%% Simple Animation
figure;
for i = 1:20:N
    clf;
    x = l * sin(theta_history(i));
    y = -l * cos(theta_history(i));
    plot([0 x], [0 y], 'k-', 'LineWidth', 2); hold on;
    plot(x, y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    axis equal;
    axis([-l l -l l]);
    title(sprintf('Time: %.2f s', t(i)));
    drawnow;
end
