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

% Define initial and final Cartesian positions for the end-effector
disp('Enter the initial position of the end-effector in Cartesian space [x y z] (e.g., [0.2 0.1 0.3]):');
p0 = input('Initial position (p0): '); % User input for initial Cartesian position
disp('Enter the final position of the end-effector in Cartesian space [x y z] (e.g., [0.4 0.2 0.5]):');
pf = input('Final position (pf): '); % User input for final Cartesian position

% Define time parameters
t0 = 0; % Start time
tf = 10; % End time (seconds)
N = 50; % Reduced number of trajectory points for faster animation

% Time vector
t = linspace(t0, tf, N);

% Linear interpolation for the Cartesian path
p_traj = zeros(3, N); % Initialize Cartesian path
for i = 1:3
    p_traj(i, :) = linspace(p0(i), pf(i), N); % Linear interpolation for x, y, z
end

% Initialize joint angles for storing IK solutions
q_traj = zeros(5, N);
q_initial_guess = zeros(1, 5); % Initial guess for IK

% Solve Inverse Kinematics for each point in the Cartesian trajectory
for i = 1:N
    % Define the target transformation matrix for the end-effector
    T = transl(p_traj(1, i), p_traj(2, i), p_traj(3, i)); % Translation matrix for the end-effector position
    
    % Use inverse kinematics to find the joint angles that achieve the desired end-effector position
    q = robot.ikine(T, 'q0', q_initial_guess, 'mask', [1 1 1 0 0 0]); % Solve IK with an initial guess and position constraints
    
    % Check if a valid solution is found
    if isempty(q)
        warning('No solution found for point %d, skipping...', i);
    else
        q_traj(:, i) = q'; % Store the joint angles
        q_initial_guess = q; % Update the initial guess for the next step
    end
end

%% 3D Animation of the Task Space Trajectory
figure;
hold on;
plot3(p_traj(1, :), p_traj(2, :), p_traj(3, :), 'r--', 'LineWidth', 1.5); % Plot the Cartesian path
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Animation of End-Effector Movement with Path');
grid on;
axis equal; % Keep axis scaling equal for better visual understanding
xlim([-1 1]); % Set X-axis limits
ylim([-1 1]); % Set Y-axis limits
zlim([-1 1]); % Set Z-axis limits
view(3); % Set a 3D view

% Animate the robot
for i = 1:N
    if any(q_traj(:, i) ~= 0) % Only plot if a valid solution was found
        robot.plot(q_traj(:, i)', 'workspace', [-1 1 -1 1 -0.5 1], 'perspective', 'delay', 0, 'trail', 'b'); % 3D Animation
        drawnow limitrate; % Faster drawing rate
        pause(0.05); % Increased pause for faster animation
    end
end
hold off;

%% Plot Joint Angles vs Time
figure;
hold on;
for i = 1:5
    plot(t, q_traj(i, :), 'DisplayName', sprintf('Joint %d', i), 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
title('Joint Angles vs Time (From Inverse Kinematics)');
legend show;
grid on;

