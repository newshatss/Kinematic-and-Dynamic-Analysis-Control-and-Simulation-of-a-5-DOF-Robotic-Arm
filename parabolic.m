% Define time parameters
t0 = 0; % Initial time
tf = 2; % Final time (2 seconds)
tb = 0.5; % Time for parabolic blending

% Ask user for initial and final joint positions (in degrees)
disp('Enter the initial joint positions (in degrees) for all 5 joints:');
q0_deg = input('[q1_0, q2_0, q3_0, q4_0, q5_0]: ');

disp('Enter the final joint positions (in degrees) for all 5 joints:');
qf_deg = input('[q1_f, q2_f, q3_f, q4_f, q5_f]: ');

% Convert initial and final positions to radians
q0 = deg2rad(q0_deg);
qf = deg2rad(qf_deg);

% Define the velocity in the linear segment
Omega = 10; % Velocity for the linear segment

% Calculate acceleration at the parabolic blend
alpha = Omega / tb; % Acceleration during the parabolic blend

% Time vector for plotting
t = linspace(t0, tf, 1000);

% Preallocate arrays for position, velocity, and acceleration for 5 joints
q = zeros(5, length(t));   % Position
qd = zeros(5, length(t));  % Velocity
qdd = zeros(5, length(t)); % Acceleration

for j = 1:5
    % Calculate trajectory for each joint
    for i = 1:length(t)
        if t(i) < tb
            % First parabolic segment (0 <= t <= tb)
            q(j, i) = q0(j) + (Omega / (2 * tb)) * t(i)^2;
            qd(j, i) = (Omega / tb) * t(i);
            qdd(j, i) = alpha;
            
        elseif t(i) < tf - tb
            % Linear segment (tb <= t <= tf - tb)
            qtb = 0.5 * (q0(j) + qf(j) - Omega * tf); % Calculate q(tb) for joint j
            q(j, i) = qtb + Omega * (t(i) - tb);
            qd(j, i) = Omega;
            qdd(j, i) = 0;
            
        else
            % Final parabolic deceleration (tf - tb <= t <= tf)
            q(j, i) = qf(j) - 0.5 * alpha * tf^2 + alpha * tf * t(i) - 0.5 * alpha * t(i)^2;
            qd(j, i) = alpha * (tf - t(i));
            qdd(j, i) = -alpha;
        end
    end
end

% Plot position, velocity, and acceleration for all joints in one figure
figure;

% Position
subplot(3, 1, 1);
plot(t, rad2deg(q(1, :)), 'LineWidth', 2); hold on;
plot(t, rad2deg(q(2, :)), 'LineWidth', 2);
plot(t, rad2deg(q(3, :)), 'LineWidth', 2);
plot(t, rad2deg(q(4, :)), 'LineWidth', 2);
plot(t, rad2deg(q(5, :)), 'LineWidth', 2);
title('Joint Positions');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
grid on;

% Velocity
subplot(3, 1, 2);
plot(t, rad2deg(qd(1, :)), 'LineWidth', 2); hold on;
plot(t, rad2deg(qd(2, :)), 'LineWidth', 2);
plot(t, rad2deg(qd(3, :)), 'LineWidth', 2);
plot(t, rad2deg(qd(4, :)), 'LineWidth', 2);
plot(t, rad2deg(qd(5, :)), 'LineWidth', 2);
title('Joint Velocities');
xlabel('Time (s)');
ylabel('Velocity (deg/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
grid on;

% Acceleration
subplot(3, 1, 3);
plot(t, rad2deg(qdd(1, :)), 'LineWidth', 2); hold on;
plot(t, rad2deg(qdd(2, :)), 'LineWidth', 2);
plot(t, rad2deg(qdd(3, :)), 'LineWidth', 2);
plot(t, rad2deg(qdd(4, :)), 'LineWidth', 2);
plot(t, rad2deg(qdd(5, :)), 'LineWidth', 2);
title('Joint Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (deg/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5');
grid on;
