function [theta_ddot, tau] = pendulum_dynamics(state, params)
% Computes angular acceleration and torque with damping and optional controller
theta = state(1);
theta_dot = state(2);

% Parameters
m = params.m;
l = params.l;
g = params.g;
b = params.b; % Damping coefficient

% Determine torque based on mode
if strcmp(params.mode, 'free_fall')
    tau = 0; % Free fall (no control)
    b = 0;
else % swing_up
    % Energy-based controller
    E_d = m * g * l * (1 - cos(pi)); % Desired energy at theta = pi
    E = 0.5 * m * l^2 * theta_dot^2 + m * g * l * (1 - cos(theta)); % Current energy
    E_error = E - E_d;

    % Controller parameters
    k_e = params.k_e;
    E_switch = params.E_switch;

    % Apply energy-based or stabilization control
    if E < E_switch * E_d
        % Apply kick-start torque if theta_dot is small
        if abs(theta_dot) < 0.01
            tau = 1; % Maximum torque to initiate motion
        else
            tau = -k_e * E_error * theta_dot; % Energy pumping
        end
    else
        if strcmp(params.controller, 'pd')
            % PD control
            k_p = params.k_p;
            k_d = params.k_d;
            tau = -k_p * (theta - pi) - k_d * theta_dot;
        else % lqr
            % LQR control: u = -K * [theta - pi; theta_dot]
            K_lqr = params.K_lqr;
            state_dev = [theta - pi; theta_dot];
            tau = -K_lqr * state_dev;
        end
    end

    % Apply torque limit
    tau = max(-1, min(1, tau));
end

% Dynamics with damping
theta_ddot = (tau - b * theta_dot - m * g * l * sin(theta)) / (m * l^2);
end