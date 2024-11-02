%% Verification of Hand Calculations
% Set specific joint angles, velocities, and accelerations
theta1 = pi/4; % 45 degrees
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;

thetad1 = 1; % 1 rad/s for joint 1
thetad2 = 0;
thetad3 = 0;
thetad4 = 0;
thetad5 = 0;

thetadd1 = 0; % No acceleration for any joint
thetadd2 = 0;
thetadd3 = 0;
thetadd4 = 0;
thetadd5 = 0;

% Substitute values into the transformation matrices
T01_num = double(subs(T01));
T12_num = double(subs(T12));
T23_num = double(subs(T23));
T34_num = double(subs(T34));
T45_num = double(subs(T45));
T5e_num = double(subs(T5e));

% Compute angular velocity w1
w00 = [0; 0; 0]; % Base link has no angular velocity
w11 = R10 * w00 + [0; 0; thetad1];
w11_num = double(subs(w11));

disp('Angular velocity w11 (Link 1):');
disp(w11_num);

% Expected Result: [0; 0; 1] rad/s

% Compute linear velocity v1
P01_num = T01_num(1:3, 4);
v00 = [0; 0; 0]; % Base link has no linear velocity
v11 = R10 * (v00 + cross(w00, P01_num));
v11_num = double(subs(v11));

disp('Linear velocity v11 (Link 1):');
disp(v11_num);

% Expected Result: [0; 0; 0] m/s

% Compute angular acceleration w1_dot
w11_d = R10 * [0; 0; 0] + R10 * cross([0; 0; 0], [0; 0; thetad1]) + [0; 0; thetadd1];
w11_d_num = double(subs(w11_d));

disp('Angular acceleration w11_dot (Link 1):');
disp(w11_d_num);

% Expected Result: [0; 0; 0] rad/s^2

% gravity 
g_vec = [0; 0; 9.81];

% Correct linear acceleration calculation for link 1
v11_d = R10 * (cross([0; 0; 0], P01_num) + cross([0; 0; 0], cross([0; 0; 0], P01_num))) + g_vec;
v11_d_num = double(subs(v11_d));

disp('Linear acceleration v11_dot (Link 1):');
disp(v11_d_num);

%% Mass matrix validation
isSymmetricM = isequal(vpa(M, 3), vpa(transpose(M), 3));
disp('Is Mass Matrix Symmetric:');
disp(isSymmetricM);
%% G matrix validation
thetadd1 = 0; thetadd2 = 0; thetadd3 = 0; thetadd4 = 0; thetadd5 = 0;
thetad1 = 0; thetad2 = 0; thetad3 = 0; thetad4 = 0; thetad5 = 0;
g_check = [eval(taw1); eval(taw2); eval(taw3); eval(taw4); eval(taw5)];

disp('Gravitational Vector G:');
disp(vpa(G, 3));
disp('Computed Gravitational Vector (g_check):');
disp(vpa(g_check, 3));
isGCorrect = isequal(vpa(G, 3), vpa(g_check, 3));
disp('Is Gravitational Vector Correct:');
disp(isGCorrect);

%% Torques (Joint Forces) vs Time
% Define time vector and symbolic functions for joint angles
t = linspace(0, 10, 100); % Time from 0 to 10 seconds, 100 points
theta_vals = [pi/2*sin(t); pi/4*sin(t); pi/3*sin(t); pi/6*sin(t); pi/8*sin(t)]; % Joint angles as sine functions for demonstration
thetad_vals = [pi/2*cos(t); pi/4*cos(t); pi/3*cos(t); pi/6*cos(t); pi/8*cos(t)]; % Joint velocities
thetadd_vals = -[pi/2*sin(t); pi/4*sin(t); pi/3*sin(t); pi/6*sin(t); pi/8*sin(t)]; % Joint accelerations

% Preallocate space for torque values
torques = zeros(5, length(t));

% Calculate torques for each time step
for i = 1:length(t)
    % Substitute joint variables into symbolic expressions
    theta1 = theta_vals(1, i);
    theta2 = theta_vals(2, i);
    theta3 = theta_vals(3, i);
    theta4 = theta_vals(4, i);
    theta5 = theta_vals(5, i);
    
    thetad1 = thetad_vals(1, i);
    thetad2 = thetad_vals(2, i);
    thetad3 = thetad_vals(3, i);
    thetad4 = thetad_vals(4, i);
    thetad5 = thetad_vals(5, i);
    
    thetadd1 = thetadd_vals(1, i);
    thetadd2 = thetadd_vals(2, i);
    thetadd3 = thetadd_vals(3, i);
    thetadd4 = thetadd_vals(4, i);
    thetadd5 = thetadd_vals(5, i);
    
    % Evaluate torques
    torques(:, i) = double([eval(taw1); eval(taw2); eval(taw3); eval(taw4); eval(taw5)]);
end

% Plot the torques vs. time
figure;
for j = 1:5
    subplot(5, 1, j);
    plot(t, torques(j, :));
    title(['Torque \tau_' num2str(j) ' vs Time']);
    xlabel('Time [s]');
    ylabel(['\tau_' num2str(j) ' [Nm]']);
    grid on;
end
%% Joint Velocities and Accelerations
% Define time vector and symbolic functions for joint angles
t = linspace(0, 10, 100); % Time from 0 to 10 seconds, 100 points
theta_vals = [pi/2*sin(t); pi/4*sin(t); pi/3*sin(t); pi/6*sin(t); pi/8*sin(t)]; % Joint angles as sine functions for demonstration
thetad_vals = [pi/2*cos(t); pi/4*cos(t); pi/3*cos(t); pi/6*cos(t); pi/8*cos(t)]; % Joint velocities
thetadd_vals = -[pi/2*sin(t); pi/4*sin(t); pi/3*sin(t); pi/6*sin(t); pi/8*sin(t)]; % Joint accelerations

% Plot the angular velocities and accelerations vs. time for each joint
figure;
for j = 1:5
    % Plot joint angular velocity
    subplot(5, 2, 2*j-1);
    plot(t, thetad_vals(j, :));
    title(['Joint Velocity \theta_d_' num2str(j) ' vs Time']);
    xlabel('Time [s]');
    ylabel(['Velocity [rad/s]']);
    grid on;

    % Plot joint angular acceleration
    subplot(5, 2, 2*j);
    plot(t, thetadd_vals(j, :));
    title(['Joint Acceleration \theta_dd_' num2str(j) ' vs Time']);
    xlabel('Time [s]');
    ylabel(['Acceleration [rad/s^2]']);
    grid on;
end

%% eigen values
% Define a new set of specific values for joint angles (try different configuration)
theta1_val = 0;
theta2_val = pi/3;
theta3_val = pi/4;
theta4_val = pi/6;
theta5_val = pi/2;

% Substitute the new values into the symbolic mass matrix M
M_numeric = subs(M, {theta1, theta2, theta3, theta4, theta5}, ...
                 {theta1_val, theta2_val, theta3_val, theta4_val, theta5_val});

% Convert the symbolic matrix to a numeric matrix
M_numeric = double(M_numeric);

% Calculate the eigenvalues
eigenvalues = eig(M_numeric);

% Display the eigenvalues
disp('Eigenvalues of M for the new configuration:');
disp(eigenvalues);




