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

% Define time parameters
t0 = 0; % Start time
tf = 10; % End time (seconds)
N = 50; % Reduce the number of trajectory points for faster animation

% Time vector
t = linspace(t0, tf, N);

% Linear interpolation for joint angles
q = zeros(5, N); % Initialize matrix for joint trajectories
for i = 1:5
    q(i, :) = linspace(q0(i), qf(i), N);
end

% Calculate the end-effector path in Cartesian space
end_effector_positions = zeros(N, 3); % Store XYZ positions

for i = 1:N
    T = robot.fkine(q(:, i)'); % Forward kinematics for each configuration
    end_effector_positions(i, :) = T.t'; % Extract position part from transformation matrix
end

%% 1. 3D Animated Simulation of End-Effector Movement with Path
figure;
hold on;
plot3(end_effector_positions(:, 1), end_effector_positions(:, 2), end_effector_positions(:, 3), 'r--', 'LineWidth', 1.5); % Plot the path in 3D
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Animation of End-Effector Movement with Path');
grid on;

% Animate the robot and show the path
for i = 1:N
    robot.plot(q(:, i)', 'workspace', [-1 1 -1 1 0 1], 'perspective', 'delay', 0, 'trail', 'b'); % 3D Animation with blue trail
    plot3(end_effector_positions(i, 1), end_effector_positions(i, 2), end_effector_positions(i, 3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b'); % Current position
    drawnow;
    pause(0.001); % Adjust the pause time for faster animation
end
hold off;

%% 2. 2D Animation of End-Effector Trajectory in X-Y Plane
figure;
hold on;
plot(end_effector_positions(:, 1), end_effector_positions(:, 2), 'r--'); % Plot static X-Y trajectory
for i = 1:N
    plot(end_effector_positions(i, 1), end_effector_positions(i, 2), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b'); % Current position
    xlabel('X (m)');
    ylabel('Y (m)');
    title('2D Animation of End-Effector Trajectory in X-Y Plane');
    grid on;
    pause(0.001); % Adjust the pause time for faster animation
end
hold off;

%% 3. Plot of Joint Angles (Theta) vs. Time
figure;
hold on;
for i = 1:5
    plot(t, q(i, :), 'DisplayName', sprintf('Joint %d', i), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angles (Theta) vs. Time');
legend show;
grid on;


