% MATLAB script to derive and verify joint space dynamics of a planar 2R robot arm
% Exercise 8, Part 2: Symbolic derivation using MATLAB Symbolic Math Toolbox

clear all;
clc;

% Define symbolic variables
syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 m1 m2 l1 l2 I1 I2 g real positive;

% Define joint angles, velocities, and accelerations
q = [theta1; theta2];
dq = [dtheta1; dtheta2];
ddq = [ddtheta1; ddtheta2];

% COM positions
x_c1 = (l1/2) * cos(theta1);
y_c1 = (l1/2) * sin(theta1);
x_c2 = l1 * cos(theta1) + (l2/2) * cos(theta1 + theta2);
y_c2 = l1 * sin(theta1) + (l2/2) * sin(theta1 + theta2);

% Velocities
dx_c1 = jacobian(x_c1, [theta1, theta2]) * dq;
dy_c1 = jacobian(y_c1, [theta1, theta2]) * dq;
dx_c2 = jacobian(x_c2, [theta1, theta2]) * dq;
dy_c2 = jacobian(y_c2, [theta1, theta2]) * dq;

% Kinetic energy
T1 = (1/2) * m1 * (dx_c1^2 + dy_c1^2) + (1/2) * I1 * dtheta1^2;
T2 = (1/2) * m2 * (dx_c2^2 + dy_c2^2) + (1/2) * I2 * (dtheta1 + dtheta2)^2;
T = simplify(T1 + T2);

% Potential energy
U = m1 * g * y_c1 + m2 * g * y_c2;
U = simplify(U);

% Lagrangian
L = T - U;

% Euler-Lagrange equations
tau = sym(zeros(2, 1));
for i = 1:2
    dL_dqidot = diff(L, dq(i));
    % Substitute time derivatives as symbolic variables
    d_dt_dL_dqidot = jacobian(dL_dqidot, [theta1, theta2]) * dq + jacobian(dL_dqidot, [dtheta1, dtheta2]) * ddq;
    dL_dqi = diff(L, q(i));
    tau(i) = simplify(d_dt_dL_dqidot - dL_dqi);
end

% Extract M, C, G
% tau = M * ddq + C * dq + G
M = sym(zeros(2, 2));
C = sym(zeros(2, 2));
G = sym(zeros(2, 1));

% Compute G (gravity terms when ddq = 0, dq = 0)
G = simplify(subs(tau, {ddtheta1, ddtheta2, dtheta1, dtheta2}, {0, 0, 0, 0}));

% Compute M (coefficients of ddtheta1, ddtheta2)
for i = 1:2
    for j = 1:2
        M(i,j) = simplify(diff(tau(i), ddq(j)));
    end
end

% Compute C (coefficients of dq after removing acceleration terms)
tau_noaccel = subs(tau, {ddtheta1, ddtheta2}, {0, 0});
for i = 1:2
    for j = 1:2
        C(i,j) = simplify(diff(tau_noaccel(i), dq(j)));
    end
end

% Display results
disp('Inertia Matrix M(q):');
pretty(M)
disp('Coriolis-Centrifugal Matrix C(q, dq):');
pretty(C)
disp('Gravity Vector G(q):');
pretty(G)

% Verification with numerical values
disp('Numerical Verification:');
params = {m1, m2, l1, l2, I1, I2, g, theta1, theta2, dtheta1, dtheta2};
values = {1, 1, 1, 1, 1/12, 1/12, 9.81, pi/4, pi/4, 0.1, 0.1};
M_num = double(subs(M, params, values));
C_num = double(subs(C, params, values));
G_num = double(subs(G, params, values));

disp('M(q) with m1=m2=1 kg, l1=l2=1 m, I1=I2=1/12 kg·m², theta1=theta2=pi/4, g=9.81 m/s²:');
disp(M_num);
disp('C(q, dq) with above parameters and dtheta1=dtheta2=0.1 rad/s:');
disp(C_num);
disp('G(q) with above parameters:');
disp(G_num);

% Check symmetry of M
is_symmetric = isequal(M, M.');
disp(['Is M symmetric? ', num2str(is_symmetric)]);

% Check positive definiteness of M
eigenvalues_M = eig(M_num);
disp('Eigenvalues of M_num (should be positive for positive definiteness):');
disp(eigenvalues_M);