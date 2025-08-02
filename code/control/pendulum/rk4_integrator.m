function [t_vec, theta_hist, theta_dot_hist, tau_hist] = rk4_integrator(dynamics_fn, state0, params)
    % 4th-order Runge-Kutta for pendulum, logs torque at each step

    % 1) Build time vector with exact dt
    t_vec = 0:params.dt:(params.T - params.dt);
    N     = numel(t_vec);

    % 2) Preallocate
    theta_hist     = zeros(1, N);
    theta_dot_hist = zeros(1, N);
    tau_hist       = zeros(1, N);

    state = state0;  % [theta; theta_dot]

    for i = 1:N
        % Log current state
        theta_hist(i)     = state(1);
        theta_dot_hist(i) = state(2);

        % Evaluate torque at the beginning of the step
        [theta_ddot1, tau1] = dynamics_fn(state, params);
        tau_hist(i) = tau1;

        % RK4 coefficients
        h = params.dt;
        k1 = [state(2);               theta_ddot1];

        x2 = state + 0.5*h*k1;
        [theta_ddot2, ~] = dynamics_fn(x2, params);
        k2 = [x2(2); theta_ddot2];

        x3 = state + 0.5*h*k2;
        [theta_ddot3, ~] = dynamics_fn(x3, params);
        k3 = [x3(2); theta_ddot3];

        x4 = state +     h*k3;
        [theta_ddot4, ~] = dynamics_fn(x4, params);
        k4 = [x4(2); theta_ddot4];

        % Update state
        state = state + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
    end
end
