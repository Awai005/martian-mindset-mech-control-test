function render_pendulum(t_vec, theta_hist, theta_dot_hist, params, tau_hist)
% Renders pendulum animation with torque value display

l = params.l;
N = length(t_vec);

figure;
for i = 1:N
    clf;

    theta = theta_hist(i);
    x = l * sin(theta);
    y = -l * cos(theta);

    % Pendulum rod and bob
    plot([0 x], [0 y], 'k-', 'LineWidth', 2); hold on;
    plot(x, y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');

    % Velocity vector
    vx = l * theta_dot_hist(i) * cos(theta);
    vy = l * theta_dot_hist(i) * sin(theta);
    quiver(x, y, vx * 0.2, vy * 0.2, 0, 'b', 'LineWidth', 1.5);

    % Display torque value
    text(-1.1*l, 1.1*l, sprintf('Torque: %.3f Nm', tau_hist(i)), ...
         'FontSize', 10, 'Color', 'black');

    % Set axis limits to show full pendulum motion
    axis equal;
    axis([-1.2*l, 1.2*l, -1.2*l, 1.2*l]);
    title(sprintf('Time: %.2f s', t_vec(i)));
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;

    pause(params.dt);
end
end