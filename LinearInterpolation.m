clear;
clc;

% Define the robot model
l1 = 0; l2 = 0.50; l3 = 0.50; l4 = 0.15; % Link lengths

% Define the links using DH parameters
L(1) = Link([0, l1, 0, pi/2], 'standard');
L(2) = Link([0, 0, l2, 0], 'standard');
L(3) = Link([0, 0, l3, 0], 'standard');
L(4) = Link([0, 0, 0, -pi/2], 'standard');
L(5) = Link([0, l4, 0, 0], 'standard');

% Create the robot model
robot = SerialLink(L, 'name', '5-DOF RRRRR Manipulator');

% Ask user for initial and final joint angles (in degrees)
disp('Enter the initial joint angles in degrees (e.g., [0 0 0 0 0]):');
q0_deg = input('Initial joint angles (q0): '); % User input for initial joint angles
disp('Enter the final joint angles in degrees (e.g., [45 30 60 90 22.5]):');
qf_deg = input('Final joint angles (qf): '); % User input for final joint angles

% Convert angles from degrees to radians
q0 = deg2rad(q0_deg); % Initial joint angles in radians
qf = deg2rad(qf_deg); % Final joint angles in radians

% Define initial and final velocities (assumed zero)
qd0 = zeros(5, 1); % Initial velocities (rad/s)
qdf = zeros(5, 1); % Final velocities (rad/s)

% Define initial and final accelerations (for quintic, assumed zero)
qdd0 = zeros(5, 1); % Initial accelerations (rad/s^2)
qddf = zeros(5, 1); % Final accelerations (rad/s^2)

% Define time parameters
t0 = 0; % Start time
tf = 1; % End time (seconds) - reduced to 1 second
N = 100; % Number of trajectory points

% Time vector
t = linspace(t0, tf, N);

%% Cubic Polynomial Trajectory Planning
q_cubic = zeros(5, N); % Store joint positions for cubic polynomial
for i = 1:5
    % Calculate cubic polynomial coefficients
    a0 = q0(i);
    a1 = qd0(i);
    a2 = (3*(qf(i) - q0(i)) - (2*qd0(i) + qdf(i)) * (tf - t0)) / (tf - t0)^2;
    a3 = (2*(q0(i) - qf(i)) + (qd0(i) + qdf(i)) * (tf - t0)) / (tf - t0)^3;

    % Compute cubic polynomial for each joint over time
    q_cubic(i, :) = a0 + a1*t + a2*t.^2 + a3*t.^3;
end

%% Quintic Polynomial Trajectory Planning
q_quintic = zeros(5, N); % Store joint positions for quintic polynomial
for i = 1:5
    % Calculate quintic polynomial coefficients
    A = [1, t0, t0^2, t0^3, t0^4, t0^5;
         0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
         0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
         1, tf, tf^2, tf^3, tf^4, tf^5;
         0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
         0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
    
    B = [q0(i); qd0(i); qdd0(i); qf(i); qdf(i); qddf(i)];
    coeffs = A \ B; % Solve for the coefficients [a0; a1; a2; a3; a4; a5]
    
    % Compute quintic polynomial for each joint over time
    q_quintic(i, :) = coeffs(1) + coeffs(2)*t + coeffs(3)*t.^2 + coeffs(4)*t.^3 + coeffs(5)*t.^4 + coeffs(6)*t.^5;
end

%% Plot Joint Trajectories
figure;
subplot(2, 1, 1);
hold on;
for i = 1:5
    plot(t, q_cubic(i, :), 'DisplayName', sprintf('Joint %d', i), 'LineWidth', 1.5);
    yline(qf(i), '--', 'Color', [0.5, 0.5, 0.5], 'DisplayName', sprintf('Final Desired for Joint %d', i)); % Dashed line for final desired
end
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Cubic Polynomial Trajectory');
legend show;
grid on;

subplot(2, 1, 2);
hold on;
for i = 1:5
    plot(t, q_quintic(i, :), 'DisplayName', sprintf('Joint %d', i), 'LineWidth', 1.5);
    yline(qf(i), '--', 'Color', [0.5, 0.5, 0.5], 'DisplayName', sprintf('Final Desired for Joint %d', i)); % Dashed line for final desired
end
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Quintic Polynomial Trajectory');
legend show;
grid on;

