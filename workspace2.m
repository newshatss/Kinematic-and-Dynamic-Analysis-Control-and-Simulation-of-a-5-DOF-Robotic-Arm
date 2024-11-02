% Define the DH parameters and joint limits
l1 = 0;
l2 = 0.50; 
l3 = 0.50; 
l4 = 0.15;

% Define the links using DH parameters (standard)
L(1) = Link([0, l1, 0, pi/2], 'standard');
L(2) = Link([0, 0, l2, 0], 'standard');
L(3) = Link([0, 0, l3, 0], 'standard');
L(4) = Link([0, 0, 0, -pi/2], 'standard');
L(5) = Link([0, l4, 0, 0], 'standard');

% Set the joint limits
L(1).qlim = [deg2rad(-120), deg2rad(120)];
L(2).qlim = [deg2rad(10), deg2rad(180)];
L(3).qlim = [deg2rad(20), deg2rad(180)];
L(4).qlim = [deg2rad(-30), deg2rad(90)];
L(5).qlim = [deg2rad(-150), deg2rad(150)];

% Create the robot using SerialLink
robot = SerialLink(L, 'name', '5-DOF Robot');

% Define the resolution for sweeping the joint angles
n = 30; % Number of points to sample in each joint

% Create a meshgrid for joint angles within their limits
theta1_vals = linspace(L(1).qlim(1), L(1).qlim(2), n);
theta2_vals = linspace(L(2).qlim(1), L(2).qlim(2), n);
theta3_vals = linspace(L(3).qlim(1), L(3).qlim(2), n);
theta4_vals = linspace(L(4).qlim(1), L(4).qlim(2), n);
theta5_vals = linspace(L(5).qlim(1), L(5).qlim(2), n);

% Initialize arrays to hold the workspace points
XY_workspace = [];
XZ_workspace = [];
YZ_workspace = [];

% Perform the sweep for different joint combinations
for theta1 = theta1_vals
    for theta2 = theta2_vals
        for theta3 = theta3_vals
            % For each combination of theta1, theta2, theta3, calculate forward kinematics
            T = robot.fkine([theta1, theta2, theta3, 0, 0]); % Fix theta4 and theta5 for 2D workspace
            
            % Get the end-effector position
            Px = T.t(1);
            Py = T.t(2);
            Pz = T.t(3);
            
            % Store the points for different planes
            XY_workspace = [XY_workspace; Px, Py];
            XZ_workspace = [XZ_workspace; Px, Pz];
            YZ_workspace = [YZ_workspace; Py, Pz];
        end
    end
end

% Plot the workspace in the XY plane
figure;
scatter(XY_workspace(:, 1), XY_workspace(:, 2), '.');
xlabel('X');
ylabel('Y');
title('2D Workspace in XY Plane');
axis equal;

% Plot the workspace in the XZ plane
figure;
scatter(XZ_workspace(:, 1), XZ_workspace(:, 2), '.');
xlabel('X');
ylabel('Z');
title('2D Workspace in XZ Plane');
axis equal;

% Plot the workspace in the YZ plane
figure;
scatter(YZ_workspace(:, 1), YZ_workspace(:, 2), '.');
xlabel('Y');
ylabel('Z');
title('2D Workspace in YZ Plane');
axis equal;
%%
% % Define DH parameters for 5-DOF manipulator "Alisha"
% l1 = 0;
% l2 = 0.50; 
% l3 = 0.50; 
% l4 = 0.15;
% 
% % Define the links using DH parameters (standard)
% L(1) = Link([0, l1, 0, pi/2], 'standard');
% L(2) = Link([0, 0, l2, 0], 'standard');
% L(3) = Link([0, 0, l3, 0], 'standard');
% L(4) = Link([0, 0, 0, -pi/2], 'standard');
% L(5) = Link([0, l4, 0, 0], 'standard');
% 
% % Set the joint limits based on correct ranges
% L(1).qlim = [deg2rad(-120), deg2rad(120)];
% L(2).qlim = [deg2rad(10), deg2rad(180)];
% L(3).qlim = [deg2rad(20), deg2rad(180)];
% L(4).qlim = [deg2rad(-30), deg2rad(90)];
% L(5).qlim = [deg2rad(-150), deg2rad(150)];
% 
% % Define the robot as 'Alisha'
% Alisha = SerialLink(L, 'name', 'Alisha');
% 
% % Define the resolution for sweeping the joint angles
% n = 30; % Increase resolution to get smoother plots
% 
% % Generate joint angle values based on their limits
% theta1_vals = linspace(L(1).qlim(1), L(1).qlim(2), n);
% theta2_vals = linspace(L(2).qlim(1), L(2).qlim(2), n);
% theta3_vals = linspace(L(3).qlim(1), L(3).qlim(2), n);
% 
% % Initialize arrays to hold the workspace points
% XY_workspace = [];
% XZ_workspace = [];
% YZ_workspace = [];
% 
% % Perform joint sweeps and calculate forward kinematics for smoother workspace
% for theta1 = theta1_vals
%     for theta2 = theta2_vals
%         for theta3 = theta3_vals
%             % Forward kinematics to calculate end-effector position
%             T = Alisha.fkine([theta1, theta2, theta3, 0, 0]);  % Fix theta4 and theta5 for 2D workspace
%             
%             % Extract position of the end effector
%             Px = T.t(1);  % X-coordinate
%             Py = T.t(2);  % Y-coordinate
%             Pz = T.t(3);  % Z-coordinate
%             
%             % Store the points for different workspace planes
%             XY_workspace = [XY_workspace; Px, Py];  % For XY plane
%             XZ_workspace = [XZ_workspace; Px, Pz];  % For XZ plane
%             YZ_workspace = [YZ_workspace; Py, Pz];  % For YZ plane
%         end
%     end
% end
% 
% % Plot the workspace in the XY plane
% figure;
% plot(XY_workspace(:, 1), XY_workspace(:, 2), 'LineWidth', 1.5);
% xlabel('X');
% ylabel('Y');
% title('Smooth Workspace in XY Plane');
% axis equal;
% grid on;
% 
% % Plot the workspace in the XZ plane
% figure;
% plot(XZ_workspace(:, 1), XZ_workspace(:, 2), 'LineWidth', 1.5);
% xlabel('X');
% ylabel('Z');
% title('Smooth Workspace in XZ Plane');
% axis equal;
% grid on;
% 
% % Plot the workspace in the YZ plane
% figure;
% plot(YZ_workspace(:, 1), YZ_workspace(:, 2), 'LineWidth', 1.5);
% xlabel('Y');
% ylabel('Z');
% title('Smooth Workspace in YZ Plane');
% axis equal;
% grid on;
