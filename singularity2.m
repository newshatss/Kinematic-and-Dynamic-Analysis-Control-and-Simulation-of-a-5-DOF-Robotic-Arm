% Define symbolic variables for joint angles
syms theta1 theta2 theta3 theta4 theta5 real

% Jacobian computation as in your previous code
T01 = [cos(theta1),0,sin(theta1),0;sin(theta1),0,-cos(theta1),0;0,1,0,0;0,0,0,1];
T12 = [cos(theta2),-sin(theta2),0,0.50*cos(theta2);sin(theta2),cos(theta2),0,50*sin(theta2);0,0,1,0;0,0,0,1];
T23 = [cos(theta3),-sin(theta3),0,0.50*cos(theta3);sin(theta3),cos(theta3),0,50*sin(theta3);0,0,1,0;0,0,0,1];
T34 = [cos(theta4),0,-sin(theta4),0;sin(theta4),0,cos(theta4),0;0,-1,0,0;0,0,0,1];
T45 = [cos(theta5),-sin(theta5),0,0;sin(theta5),cos(theta5),0,0;0,0,1,0.15;0,0,0,1];

T02 = T01 * T12;
T03 = T01 * T12 * T23;
T04 = T01 * T12 * T23 * T34;
T0e = T01 * T12 * T23 * T34 * T45;

Z1 = T01(1:3, 3);
Z2 = T02(1:3, 3);
Z3 = T03(1:3, 3);
Z4 = T04(1:3, 3);
Z5 = T0e(1:3, 3);

O1 = T01(1:3, 4);
O2 = T02(1:3, 4);
O3 = T03(1:3, 4);
O4 = T04(1:3, 4);
O5 = T0e(1:3, 4);

% Jacobian for the end effector
J1 = [cross(Z1, (O5 - O1)); Z1];
J2 = [cross(Z2, (O5 - O2)); Z2];
J3 = [cross(Z3, (O5 - O3)); Z3];
J4 = [cross(Z4, (O5 - O4)); Z4];
J5 = [cross(Z5, (O5 - O5)); Z5];
J = [J1 J2 J3 J4 J5];

% Convert Jacobian to a MATLAB function
J_func = matlabFunction(J, 'Vars', [theta1 theta2 theta3 theta4 theta5]);

% Reduce the number of joint angle values for faster computation
theta1_vals = linspace(-pi, pi, 20);  % Reduced resolution
theta2_vals = linspace(0, pi, 20);    % Reduced resolution
theta3_vals = linspace(0, pi, 20);    % Reduced resolution
theta4_vals = linspace(-pi/2, pi/2, 20); % Reduced resolution
theta5_vals = linspace(-pi, pi, 20);  % Reduced resolution

% Open a file to save the singularities
fileID = fopen('singularities.txt', 'w');
fprintf(fileID, 'Singularities Detected at:\n\n');

% Loop through joint values to check for singularities
for th1 = theta1_vals
    for th2 = theta2_vals
        for th3 = theta3_vals
            for th4 = theta4_vals
                for th5 = theta5_vals
                    % Evaluate the Jacobian matrix
                    J_eval = J_func(th1, th2, th3, th4, th5);
                    [is_singular, rankJ] = check_singularity(J_eval);
                    
                    if is_singular
                        fprintf(fileID, 'Singularity at [theta1, theta2, theta3, theta4, theta5] = [%.2f, %.2f, %.2f, %.2f, %.2f], Rank = %d\n', ...
                                th1, th2, th3, th4, th5, rankJ);
                    end
                end
            end
        end
    end
end

% Close the file
fclose(fileID);

disp('Singularity check completed. Results saved to singularities.txt.');

% Local function definition at the end of the script
function [is_singular, rankJ] = check_singularity(J)
    tol = 1e-6; % Tolerance for detecting rank deficiency
    rankJ = rank(J, tol);
    is_singular = (rankJ < size(J, 2)); % Check if the rank is less than 5 (number of DOFs)
end

