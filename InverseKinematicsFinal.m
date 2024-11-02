function forward_kinematics_custom_robot()
    % Define the DH parameters and transformation matrices
    syms theta1 theta2 theta3 theta4 theta5

    % Define the transformation matrices for each joint
    T01 = [cos(theta1),0,sin(theta1),0;
          sin(theta1),0,-cos(theta1),0;
          0,1,0,0;
          0,0,0,1];

    T12 = [cos(theta2),-sin(theta2),0,50*cos(theta2);
           sin(theta2),cos(theta2),0,50*sin(theta2);
           0,0,1,0;
           0,0,0,1];

    T23 = [cos(theta3),-sin(theta3),0,50*cos(theta3);
          sin(theta3),cos(theta3),0,50*sin(theta3);
          0,0,1,0;
          0,0,0,1];

    T34 = [cos(theta4),0,-sin(theta4),0;
          sin(theta4),0,cos(theta4),0;
          0,-1,0,0;
          0,0,0,1];

    T45 = [cos(theta5),-sin(theta5),0,0;
           sin(theta5),cos(theta5),0,0;
           0,0,1,15;
           0,0,0,1];
    

    % Compute the overall transformation matrix symbolically
    T0e = T01 * T12 * T23 * T34 * T45;

    
    
    % Define the joint angles (60 degrees for each joint) in radians
    joint_angles_deg = 60 * ones(1, 5);
    joint_angles_rad = deg2rad(joint_angles_deg);

    % Substitute the joint angles into the transformation matrix
    T0e_subs = subs(T0e, {theta1, theta2, theta3, theta4, theta5}, num2cell(joint_angles_rad));

    % Evaluate the transformation matrix numerically
    T0e_eval = double(T0e_subs);

    disp('Forward kinematics (60 degrees for all joints):');
    disp(T0e_eval);

    % Define the desired end-effector transformation matrix based on the forward kinematics result
    xd = T0e_eval(1, 4);
    yd = T0e_eval(2, 4);
    zd = T0e_eval(3, 4);
    Rd = T0e_eval(1:3, 1:3);

    % Create the desired transformation matrix
    Td = [Rd, [xd; yd; zd]; 0, 0, 0, 1];

    % Numerical IK solution using nonlinear optimization

    % Define the error function for the optimization
    error_function = @(angles) sum(ik_error_function(angles, Td).^2);

    % Initial guess for the joint angles (radians)
    initial_guess = deg2rad(60 * ones(1, 5));  % Start with the 60 degrees as the initial guess

    % Define bounds for the joint angles (in radians)
    lb = deg2rad([-120, 10, 20, -30, -150]);
    ub = deg2rad([120, 180, 180, 90, 150]);

    % Solve the inverse kinematics problem using fmincon
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    [solution, fval, exitflag] = fmincon(error_function, initial_guess, [], [], [], [], lb, ub, [], options);

    % Convert the solution to degrees
    solution_deg = rad2deg(solution);

    disp('Inverse kinematics solution (radians):');
    disp(solution);

    disp('Inverse kinematics solution (degrees):');
    disp(solution_deg);
end

function error = ik_error_function(angles, Td)
    % Define the DH parameters and transformation matrices
    theta1 = angles(1);
    theta2 = angles(2);
    theta3 = angles(3);
    theta4 = angles(4);
    theta5 = angles(5);

    T01 = [cos(theta1),0,sin(theta1),0;
          sin(theta1),0,-cos(theta1),0;
          0,1,0,0;
          0,0,0,1];

    T12 = [cos(theta2),-sin(theta2),0,50*cos(theta2);
           sin(theta2),cos(theta2),0,50*sin(theta2);
           0,0,1,0;
           0,0,0,1];

    T23 = [cos(theta3),-sin(theta3),0,50*cos(theta3);
          sin(theta3),cos(theta3),0,50*sin(theta3);
          0,0,1,0;
          0,0,0,1];

    T34 = [cos(theta4),0,-sin(theta4),0;
          sin(theta4),0,cos(theta4),0;
          0,-1,0,0;
          0,0,0,1];

    T45 = [cos(theta5),-sin(theta5),0,0;
           sin(theta5),cos(theta5),0,0;
           0,0,1,15;
           0,0,0,1];
    


    % Compute the overall transformation matrix numerically
    T0e_num = T01 * T12 * T23 * T34 * T45;

    % Compute the position and orientation error
    position_error = T0e_num(1:3, 4) - Td(1:3, 4);
    orientation_error = T0e_num(1:3, 1:3) - Td(1:3, 1:3);

    % Combine the errors into a single error metric
    error = [position_error; reshape(orientation_error, 9, 1)];
end
