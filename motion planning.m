% Define the link lengths
l1 = 0; l2 = 0.50; l3 = 0.50; l4 = 0.15; 

% Define the links using DH parameters
L(1) = Link([0, l1, 0, pi/2], 'standard');
L(2) = Link([0, 0, l2, 0], 'standard');
L(3) = Link([0, 0, l3, 0], 'standard');
L(4) = Link([0, 0, 0, -pi/2], 'standard');
L(5) = Link([0, l4, 0, 0], 'standard');

% Create the robot using the SerialLink function
robot = SerialLink(L, 'name', '5-DOF Manipulator');

% Define the lower and upper bounds for the joint angles (in radians)
lb = deg2rad([-120, 10, 20, -30, -150]);
ub = deg2rad([120, 180, 180, 90, 150]);

% Generate a grid of possible configurations within joint limits
n = 5; % Resolution of the grid for each joint
q1 = linspace(lb(1), ub(1), n);
q2 = linspace(lb(2), ub(2), n);
q3 = linspace(lb(3), ub(3), n);
q4 = linspace(lb(4), ub(4), n);
q5 = linspace(lb(5), ub(5), n);

% Initialize an empty array to store configurations
configurations = [];

% Loop through all possible configurations
for i = 1:n
    for j = 1:n
        for k = 1:n
            for m = 1:n
                for p = 1:n
                    % Define a configuration
                    q = [q1(i), q2(j), q3(k), q4(m), q5(p)];
                    
                    % Check for collisions or constraints here (custom function)
                    if ~check_collision(robot, q)
                        % If no collision, store the configuration
                        configurations = [configurations; q];
                    end
                end
            end
        end
    end
end

% Display the number of valid configurations
disp(['Total valid configurations: ', num2str(size(configurations, 1))]);

% Plot the workspace of the manipulator
figure;
robot.plot(configurations);
title('C-space Motion Planning for 5-DOF Manipulator');

% Define a custom collision-checking function (example)
function collision = check_collision(robot, q)
    % For simplicity, this example function always returns false (no collision).
    % In practice, you should implement checks based on the environment.
    collision = false;
end
