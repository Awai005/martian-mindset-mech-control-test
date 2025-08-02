function [t_vec, theta_hist, theta_dot_hist, tau_hist] = euler_integrator(dynamics_fn, state0, params)
    % Perform explicit Euler integration, store torque history.
    %
    % dynamics_fn should have signature:
    %    [theta_ddot, tau] = dynamics_fn(state, params)
    %
    % state0 = [theta0; theta_dot0]

    % Time vector with correct spacing
    t_vec = 0:params.dt:(params.T - params.dt);
    N     = numel(t_vec);

    theta_hist     = zeros(1, N);
    theta_dot_hist = zeros(1, N);
    tau_hist       = zeros(1, N);

    state = state0;  % state = [theta; theta_dot]

    for i = 1:N
        theta     = state(1);
        theta_dot = state(2);

        % Store current state
        theta_hist(i)     = theta;
        theta_dot_hist(i) = theta_dot;

        % Compute acceleration and torque
        [theta_ddot, tau] = dynamics_fn(state, params);
        tau_hist(i)       = tau;

        % Euler update (explicit)
        theta_dot_new = theta_dot + theta_ddot * params.dt;
        theta_new     = theta     + theta_dot     * params.dt;

        state = [theta_new; theta_dot_new];
    end
end
