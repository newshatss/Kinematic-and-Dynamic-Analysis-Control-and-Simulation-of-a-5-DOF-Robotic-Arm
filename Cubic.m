% Define the robot model
l1 = 0; l2 = 0.50; l3 = 0.50; l4 = 0.15; % Link lengths

% Define the links using DH parameters
L(1) = Link([0, l1, 0, pi/2], 'standard');
L(2) = Link([0, 0, l2, 0], 'standard');
L(3) = Link([0, 0, l3, 0], 'standard');
L(4) = Link([0, 0, 0, pi/2], 'standard');
L(5) = Link([0, l4, 0, 0], 'standard');

% Create the robot model
robot = SerialLink(L, 'name', '5-DOF RRRRR Manipulator');

% Time parameters
t0 = 0; % Initial time
tf = 2; % Final time (2 seconds)
t = linspace(t0, tf, 100); % Time vector

% Get user input for initial and final joint positions (in degrees)
disp('Enter the initial joint positions in degrees:');
qi0_deg = input('Enter [q1_initial, q2_initial, q3_initial, q4_initial, q5_initial]: ');

disp('Enter the final joint positions in degrees:');
qif_deg = input('Enter [q1_final, q2_final, q3_final, q4_final, q5_final]: ');

% Convert the initial and final joint positions from degrees to radians
qi0 = deg2rad(qi0_deg); % Initial joint angles in radians
qif = deg2rad(qif_deg); % Final joint angles in radians

% Initial and final velocities (set to zero)
wi0 = [0; 0; 0; 0; 0]; % Initial velocities
wif = [0; 0; 0; 0; 0]; % Final velocities

% Cubic polynomial coefficients for each joint
A = [1 t0 t0^2 t0^3;
     1 tf tf^2 tf^3;
     0 1 2*t0 3*t0^2;
     0 1 2*tf 3*tf^2];

q = zeros(5, length(t)); % Preallocate for joint positions
qd = zeros(5, length(t)); % Preallocate for joint velocities
qdd = zeros(5, length(t)); % Preallocate for joint accelerations

for i = 1:5
    % Define the boundary conditions for joint i
    B = [qi0(i); qif(i); wi0(i); wif(i)];
    
    % Solve for polynomial coefficients a0, a1, a2, a3
    a = A\B;
    
    % Calculate position, velocity, and acceleration
    q(i, :) = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3;  % Position
    qd(i, :) = a(2) + 2*a(3)*t + 3*a(4)*t.^2;         % Velocity
    qdd(i, :) = 2*a(3) + 6*a(4)*t;                    % Acceleration
end

% Plot position, velocity, and acceleration for all joints in one figure
figure;

for i = 1:5
    % Position for all joints
    subplot(3, 1, 1);
    plot(t, q(i, :), 'LineWidth', 2);
    hold on;
    title('Joint Positions');
    xlabel('Time (s)');
    ylabel('Position (rad)');
    grid on;
    
    % Velocity for all joints
    subplot(3, 1, 2);
    plot(t, qd(i, :), 'LineWidth', 2);
    hold on;
    title('Joint Velocities');
    xlabel('Time (s)');
    ylabel('Velocity (rad/s)');
    grid on;
    
    % Acceleration for all joints
    subplot(3, 1, 3);
    plot(t, qdd(i, :), 'LineWidth', 2);
    hold on;
    title('Joint Accelerations');
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2)');
    grid on;
end

% Add legends for each joint
subplot(3, 1, 1); legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
subplot(3, 1, 2); legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
subplot(3, 1, 3); legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
% 
% % Simulate the robot motion
% figure;
% for k = 1:length(t)
%     robot.plot(q(:, k)');
%     pause(0.05); % Pause for visualization
% end

