function forwardKinematics()
    % Define the DH parameters
    l1 = 0;
    l2 = 50;
    l3 = 50;
    l4 = 15;

    % Define the links using DH parameters (standard)
    L(1) = Link([0, l1, 0, pi/2], 'standard');
    L(2) = Link([0, 0, l2, 0], 'standard');
    L(3) = Link([0, 0, l3, 0], 'standard');
    L(4) = Link([0, 0, 0, -pi/2], 'standard');
    L(5) = Link([0, l4, 0, 0], 'standard');

    % Set the new joint limits
    L(1).qlim = [deg2rad(-120), deg2rad(120)];
    L(2).qlim = [deg2rad(10), deg2rad(180)];
    L(3).qlim = [deg2rad(20), deg2rad(180)];
    L(4).qlim = [deg2rad(-30), deg2rad(90)];
    L(5).qlim = [deg2rad(-150), deg2rad(150)];

    % Define the robot as 'Alisha'
    R = SerialLink(L, 'name', 'Alisha');

    % Input desired joint angles
    theta1 = input('Enter theta1 (in degrees): ');
    theta2 = input('Enter theta2 (in degrees): ');
    theta3 = input('Enter theta3 (in degrees): ');
    theta4 = input('Enter theta4 (in degrees): ');
    theta5 = input('Enter theta5 (in degrees): ');

    % Convert input angles to radians
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    theta4 = deg2rad(theta4);
    theta5 = deg2rad(theta5);

    % Calculate forward kinematics
    T = R.fkine([theta1 theta2 theta3 theta4 theta5]);
    disp('Transformation Matrix T0e:');
    disp(T);

    % Calculate Euler angles
    eul = tr2eul(T);
    disp('Euler Angles [phi, theta, psi]:');
    disp(eul);
end
