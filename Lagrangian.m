% clc
% 
% %% Transformation Matrix
% syms theta1 theta2 theta3 theta4 theta5
% 
% T01 = [cos(theta1),0,sin(theta1),0;sin(theta1),0,-cos(theta1),0;0,1,0,0;0,0,0,1];
% T12 = [cos(theta2),-sin(theta2),0,0.50*cos(theta2);sin(theta2),cos(theta2),0,0.50*sin(theta2);0,0,1,0;0,0,0,1];
% T23 = [cos(theta3),-sin(theta3),0,0.50*cos(theta3);sin(theta3),cos(theta3),0,0.50*sin(theta3);0,0,1,0;0,0,0,1];
% T34 = [cos(theta4),0,-sin(theta4),0;sin(theta4),0,cos(theta4),0;0,-1,0,0;0,0,0,1];
% T45 = [cos(theta5),-sin(theta5),0,0;sin(theta5),cos(theta5),0,0;0,0,1,0.15;0,0,0,1];
% 
% T02 = T01 * T12 ;
% T03 = T01 * T12 * T23 ; 
% T04 = T01 * T12 * T23 * T34 ;
% T0e=  T01 * T12 * T23 * T34 * T45 ;
% %% Lagrangian
% % Mass of each link
% m1 = 0.33;
% m2 = 1.33;
% m3 = 0.89;
% m4 = 1.86;
% m5 = 0.27;
% 
% 
% % Inertia tensors in the center of mass for each link
% I1 = [474862.68,10.87,3812.88;10.87,963857.85,10.91;3812.88,10.91,1051823.40].*1e-9;
% I2 = [2273563.60, 1527184.20, -3294838.30;1527184.20, 95421453.60, -179976.76;-3294838.30, -179976.76, 95626327.64].*1e-9;
% I3 = [1059667.14,1824954.06,-37980.79;1824954.06,43565913.89,71512.94;-37980.79,71512.94,43856469.41].*1e-9;
% I4 = [7673736.40, 12723624.85, 436835.23; 12723624.85, 163621941.46, 66216.10; 436835.23, 66216.10, 166527424.84].*1e-9;
% I5 = [166527424.84, -66216.10, -436835.23; -66216.10, 163621941.46, 12723624.85; -436835.23, 12723624.85, 7673736.40].*1e-9;
% 
% % Centers of mass for each link (relative positions)
% Tc11 = [1, 0, 0, -0.02; 0, 1, 0, 0; 0, 0, 1, 0.02; 0, 0, 0, 1];
% Tc22 = [1, 0, 0, 0.27; 0, 1, 0, 0.02; 0, 0, 1, -0.06; 0, 0, 0, 1];
% Tc33 = [1, 0, 0, 0.37; 0, 1, 0, 0.02; 0, 0, 1, -0.05; 0, 0, 0, 1];
% Tc44 = [1, 0, 0, -0.45; 0, 1, 0, -0.02; 0, 0, 1, -0.09; 0, 0, 0, 1];
% 
% T0C1 = T01 * Tc11;
% T02 =T01 * T12; 
% T0C2 = T02 * Tc22;
% T03=T01 * T12 * T23; 
% T0C3 = T03 * Tc33;
% T04 = T01 * T12 * T23 * T34;
% T0C4 = T04 * Tc44;
% T05= T01 * T12 * T23 * T34 * T45;
% T0C5 = T0e;
% 
% Z1 = T01(1:3, 3);
% Z2 = T02(1:3, 3);
% Z3 = T03(1:3, 3);
% Z4 = T04(1:3, 3);
% Z5 = T0e(1:3, 3);
% 
% O1 = T01(1:3, 4);
% Oc1 = T0C1(1:3, 4);
% O2 = T02(1:3, 4);
% Oc2 = T0C2(1:3, 4);
% O3 = T03(1:3, 4);
% Oc3 = T0C3(1:3, 4);
% O4 = T04(1:3, 4);
% Oc4 = T0C4(1:3, 4);
% O5 = T0e(1:3, 4);
% Oc5 = O5;
% 
% % Jacobians
% J1 = [cross(Z1, (O5 - O1)); Z1];
% J2 = [cross(Z2, (O5 - O2)); Z2];
% J3 = [cross(Z3, (O5 - O3)); Z3];
% J4 = [cross(Z4, (O5 - O4)); Z4];
% J5 = [cross(Z5, (O5 - O5)); Z5];
% J = [J1 J2 J3 J4 J5];
% 
% % Jc - Jacobians for each center of mass
% zero = sym(zeros(6,1));  % A 6x1 zero vector for padding
% 
% % Jc5 (Jacobian for the 5th link)
% Jc5 = J;
% Jv5 = Jc5(1:3, :);
% Jw5 = Jc5(4:6, :);
% 
% % Jc4 (Jacobian for the 4th link)
% Jc41 = [cross(Z1, (Oc4 - O1)); Z1];
% Jc42 = [cross(Z2, (Oc4 - O2)); Z2];
% Jc43 = [cross(Z3, (Oc4 - O3)); Z3];
% Jc44 = [cross(Z4, (Oc4 - O4)); Z4];
% Jc4 = [Jc41 Jc42 Jc43 Jc44 zero];
% Jv4 = Jc4(1:3, :);
% Jw4 = Jc4(4:6, :);
% 
% % Jc3 (Jacobian for the 3rd link)
% Jc31 = [cross(Z1, (Oc3 - O1)); Z1];
% Jc32 = [cross(Z2, (Oc3 - O2)); Z2];
% Jc33 = [cross(Z3, (Oc3 - O3)); Z3];
% Jc3 = [Jc31 Jc32 Jc33 zero zero];
% Jv3 = Jc3(1:3, :);
% Jw3 = Jc3(4:6, :);
% 
% % Jc2 (Jacobian for the 2nd link)
% Jc21 = [cross(Z1, (Oc2 - O1)); Z1];
% Jc22 = [cross(Z2, (Oc2 - O2)); Z2];
% Jc2 = [Jc21 Jc22 zero zero zero];
% Jv2 = Jc2(1:3, :);
% Jw2 = Jc2(4:6, :);
% 
% % Jc1 (Jacobian for the 1st link)
% Jc11 = [cross(Z1, (Oc1 - O1)); Z1];
% Jc1 = [Jc11 zero zero zero zero];
% Jv1 = Jc1(1:3, :);
% Jw1 = Jc1(4:6, :);
% 
% 
% %% Mass Matrix
% R01 = T01(1:3, 1:3);
% R02 = T02(1:3, 1:3);
% R03 = T03(1:3, 1:3);
% R04 = T04(1:3, 1:3);
% R05 = T0e(1:3, 1:3);
% 
% M_theta1 = m1 * transpose(Jv1) * Jv1 + transpose(Jw1) * R01 * I1 * transpose(R01) * Jw1;
% M_theta2 = m2 * transpose(Jv2) * Jv2 + transpose(Jw2) * R02 * I2 * transpose(R02) * Jw2;
% M_theta3 = m3 * transpose(Jv3) * Jv3 + transpose(Jw3) * R03 * I3 * transpose(R03) * Jw3;
% M_theta4 = m4 * transpose(Jv4) * Jv4 + transpose(Jw4) * R04 * I4 * transpose(R04) * Jw4;
% M_theta5 = m5 * transpose(Jv5) * Jv5 + transpose(Jw5) * R05 * I5 * transpose(R05) * Jw5;
% 
% M_theta = M_theta1 + M_theta2 + M_theta3 + M_theta4 + M_theta5;
% %% C 
% 
% syms thetadd1 thetadd2 thetadd3 thetadd4 thetadd5
% syms thetad1 thetad2 thetad3 thetad4 thetad5
% C = sym(zeros(5));
% X = sym(zeros(5, 1));
% theta = [theta1; theta2; theta3; theta4; theta5];
% thetad = [thetad1; thetad2; thetad3; thetad4; thetad5];
% 
% for k = 1:5
%     for j = 1:5
%         for i = 1:5
%             X(i) = (0.5 * (diff(M_theta(k,j), theta(i)) + diff(M_theta(k,i), theta(j)) - diff(M_theta(i,j), theta(k)))) * thetad(i);
%         end
%         C(k,j) = sum(X);
%     end
% end
% %% G
% rc1 = T0C1(1:3, 4);
% rc2 = T0C2(1:3, 4);
% rc3 = T0C3(1:3, 4);
% rc4 = T0C4(1:3, 4);
% rc5 = T0C5(1:3, 4);
% G = [0; 0; -9.81];
% 
% P1 = -m1 * transpose(G) * rc1;
% P2 = -m2 * transpose(G) * rc2;
% P3 = -m3 * transpose(G) * rc3;
% P4 = -m4 * transpose(G) * rc4;
% P5 = -m5 * transpose(G) * rc5;
% P = P1 + P2 + P3 + P4 + P5;
% 
% % Compute the gravitational torques
% g1 = diff(P, theta1);
% g2 = diff(P, theta2);
% g3 = diff(P, theta3);
% g4 = diff(P, theta4);
% g5 = diff(P, theta5);
% g = [g1; g2; g3; g4; g5];
% 
% %% taw
% % taw 
% thetad=[thetad1;thetad2;thetad3;thetad4;thetad5];
% thetadd=[thetadd1;thetadd2;thetadd3;thetadd4;thetadd5];
% taw=M_theta*thetadd+C*thetad+g;
%% numerical
theta1=deg2rad(0);
theta2=deg2rad(0);
theta3=deg2rad(0);
theta4=deg2rad(0);
theta5=deg2rad(0);
% theta1=1;
% theta2=1;
% theta3=1;
% theta4=1;
% theta5=1;
thetad1=0;
thetad2=0;
thetad3=0;
thetad4=0;
thetad5=0;
thetadd1=0;
thetadd2=0;
thetadd3=0;
thetadd4=0;
thetadd5=0;
thetad=[thetad1;thetad2;thetad3;thetad4;thetad5];
thetadd=[thetadd1;thetadd2;thetadd3;thetadd4;thetadd5];

M=zeros(5);
M(1,1) = 1.8393*cos(theta3) - 0.837*cos(theta4) - 0.0178*sin(theta3) + 0.1269*sin(theta4) - 0.837*cos(theta3)*cos(theta4) + 0.1269*cos(theta3)*sin(theta4) + 0.1269*cos(theta4)*sin(theta3) - 0.000132432*cos(theta5)*sin(theta5) + 0.837*sin(theta3)*sin(theta4) - 0.00290548*cos(theta5)^2 + 3.61919;
M(1,2) = 0.91965*cos(theta3) - 0.837*cos(theta4) - 0.0089*sin(theta3) + 0.1269*sin(theta4) - 0.4185*cos(theta3)*cos(theta4) + 0.06345*cos(theta3)*sin(theta4) + 0.06345*cos(theta4)*sin(theta3) - 0.000132432*cos(theta5)*sin(theta5) + 0.4185*sin(theta3)*sin(theta4) - 0.00290548*cos(theta5)^2 + 2.35096;
M(1,3) = 0.16465*cos(theta3) - 0.4185*cos(theta4) - 0.0089*sin(theta3) + 0.06345*sin(theta4) - 0.4185*cos(theta3)*cos(theta4) + 0.06345*cos(theta3)*sin(theta4) + 0.06345*cos(theta4)*sin(theta3) - 0.000132432*cos(theta5)*sin(theta5) + 0.4185*sin(theta3)*sin(theta4) - 0.00290548*cos(theta5)^2 + 1.05864;
M(1,4) = 0.0186*sin(theta4) - 0.0127236*cos(theta5) + 0.000436835*sin(theta5) + 0.0186*cos(theta3)*sin(theta4) + 0.0186*cos(theta4)*sin(theta3) + 0.00328178;
M(1,5) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(2,1) = 0.91965*cos(theta3) - 0.837*cos(theta4) - 0.0089*sin(theta3) + 0.1269*sin(theta4) - 0.4185*cos(theta3)*cos(theta4) + 0.06345*cos(theta3)*sin(theta4) + 0.06345*cos(theta4)*sin(theta3) - 0.000132432*cos(theta5)*sin(theta5) + 0.4185*sin(theta3)*sin(theta4) - 0.00290548*cos(theta5)^2 + 2.35096;
M(2,2) = 0.1269*sin(theta4) - 0.837*cos(theta4) - 0.000132432*cos(theta5)*sin(theta5) - 0.00290548*cos(theta5)^2 + 2.17141;
M(2,3) = 0.06345*sin(theta4) - 0.4185*cos(theta4) - 0.000132432*cos(theta5)*sin(theta5) - 0.00290548*cos(theta5)^2 + 1.05864;
M(2,4) = 0.0186*sin(theta4) - 0.0127236*cos(theta5) + 0.000436835*sin(theta5) + 0.00328178;
M(2,5) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(3,1) = 0.16465*cos(theta3) - 0.4185*cos(theta4) - 0.0089*sin(theta3) + 0.06345*sin(theta4) - 0.4185*cos(theta3)*cos(theta4) + 0.06345*cos(theta3)*sin(theta4) + 0.06345*cos(theta4)*sin(theta3) - 0.000132432*cos(theta5)*sin(theta5) + 0.4185*sin(theta3)*sin(theta4) - 0.00290548*cos(theta5)^2 + 1.05864;
M(3,2) = 0.06345*sin(theta4) - 0.4185*cos(theta4) - 0.000132432*cos(theta5)*sin(theta5) - 0.00290548*cos(theta5)^2 + 1.05864;
M(3,3) = 0.893994 - 0.00290548*cos(theta5)^2 - 0.000132432*cos(theta5)*sin(theta5);
M(3,4) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5) + 0.00328178;
M(3,5) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(4,1) = 0.0186*sin(theta4) - 0.0127236*cos(theta5) + 0.000436835*sin(theta5) + 0.0186*cos(theta3)*sin(theta4) + 0.0186*cos(theta4)*sin(theta3) + 0.00328178;
M(4,2) = 0.0186*sin(theta4) - 0.0127236*cos(theta5) + 0.000436835*sin(theta5) + 0.00328178;
M(4,3) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5) + 0.00328178;
M(4,4) = 0.551595;
M(4,5) = 0.00767374;
M(5,1) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(5,2) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(5,3) = 0.000436835*sin(theta5) - 0.0127236*cos(theta5);
M(5,4) = 0.00767374;
M(5,5) = 0.00767374;


C=zeros(5);
C(1,1) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 - 0.0089*thetad3*cos(theta3) + 0.06345*thetad4*cos(theta4) - 0.91965*thetad3*sin(theta3) + 0.4185*thetad4*sin(theta4) + 0.06345*thetad3*cos(theta3)*cos(theta4) + 0.06345*thetad4*cos(theta3)*cos(theta4) + 0.4185*thetad3*cos(theta3)*sin(theta4) + 0.4185*thetad3*cos(theta4)*sin(theta3) + 0.4185*thetad4*cos(theta3)*sin(theta4) + 0.4185*thetad4*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) - 0.06345*thetad3*sin(theta3)*sin(theta4) - 0.06345*thetad4*sin(theta3)*sin(theta4);
C(1,2) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 - 0.00445*thetad3*cos(theta3) + 0.06345*thetad4*cos(theta4) - 0.459825*thetad3*sin(theta3) + 0.4185*thetad4*sin(theta4) + 0.031725*thetad3*cos(theta3)*cos(theta4) + 0.031725*thetad4*cos(theta3)*cos(theta4) + 0.20925*thetad3*cos(theta3)*sin(theta4) + 0.20925*thetad3*cos(theta4)*sin(theta3) + 0.20925*thetad4*cos(theta3)*sin(theta4) + 0.20925*thetad4*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) - 0.031725*thetad3*sin(theta3)*sin(theta4) - 0.031725*thetad4*sin(theta3)*sin(theta4);
C(1,3) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 - 0.0089*thetad1*cos(theta3) - 0.00445*thetad2*cos(theta3) - 0.0089*thetad3*cos(theta3) + 0.031725*thetad4*cos(theta4) - 0.91965*thetad1*sin(theta3) - 0.459825*thetad2*sin(theta3) - 0.16465*thetad3*sin(theta3) + 0.20925*thetad4*sin(theta4) + 0.06345*thetad1*cos(theta3)*cos(theta4) + 0.031725*thetad2*cos(theta3)*cos(theta4) + 0.06345*thetad3*cos(theta3)*cos(theta4) + 0.041025*thetad4*cos(theta3)*cos(theta4) + 0.4185*thetad1*cos(theta3)*sin(theta4) + 0.4185*thetad1*cos(theta4)*sin(theta3) + 0.20925*thetad2*cos(theta3)*sin(theta4) + 0.20925*thetad2*cos(theta4)*sin(theta3) + 0.4185*thetad3*cos(theta3)*sin(theta4) + 0.4185*thetad3*cos(theta4)*sin(theta3) + 0.20925*thetad4*cos(theta3)*sin(theta4) + 0.20925*thetad4*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) - 0.06345*thetad1*sin(theta3)*sin(theta4) - 0.031725*thetad2*sin(theta3)*sin(theta4) - 0.06345*thetad3*sin(theta3)*sin(theta4) - 0.041025*thetad4*sin(theta3)*sin(theta4);
C(1,4) = 0.06345*thetad1*cos(theta4) + 0.06345*thetad2*cos(theta4) + 0.031725*thetad3*cos(theta4) + 0.0186*thetad4*cos(theta4) + 0.000218418*thetad5*cos(theta5) + 0.4185*thetad1*sin(theta4) + 0.4185*thetad2*sin(theta4) + 0.20925*thetad3*sin(theta4) + 0.00636181*thetad5*sin(theta5) + 0.06345*thetad1*cos(theta3)*cos(theta4) + 0.031725*thetad2*cos(theta3)*cos(theta4) + 0.041025*thetad3*cos(theta3)*cos(theta4) + 0.0186*thetad4*cos(theta3)*cos(theta4) + 0.4185*thetad1*cos(theta3)*sin(theta4) + 0.4185*thetad1*cos(theta4)*sin(theta3) + 0.20925*thetad2*cos(theta3)*sin(theta4) + 0.20925*thetad2*cos(theta4)*sin(theta3) + 0.20925*thetad3*cos(theta3)*sin(theta4) + 0.20925*thetad3*cos(theta4)*sin(theta3) - 0.06345*thetad1*sin(theta3)*sin(theta4) - 0.031725*thetad2*sin(theta3)*sin(theta4) - 0.041025*thetad3*sin(theta3)*sin(theta4) - 0.0186*thetad4*sin(theta3)*sin(theta4);
C(1,5) = 0.0000662161*thetad1 + 0.0000662161*thetad2 + 0.0000662161*thetad3 - 0.000132432*thetad1*cos(theta5)^2 - 0.000132432*thetad2*cos(theta5)^2 - 0.000132432*thetad3*cos(theta5)^2 + 0.000218418*thetad4*cos(theta5) + 0.000436835*thetad5*cos(theta5) + 0.00636181*thetad4*sin(theta5) + 0.0127236*thetad5*sin(theta5) + 0.00290548*thetad1*cos(theta5)*sin(theta5) + 0.00290548*thetad2*cos(theta5)*sin(theta5) + 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(2,1) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 - 0.00445*thetad3*cos(theta3) + 0.06345*thetad4*cos(theta4) - 0.459825*thetad3*sin(theta3) + 0.4185*thetad4*sin(theta4) + 0.031725*thetad3*cos(theta3)*cos(theta4) + 0.031725*thetad4*cos(theta3)*cos(theta4) + 0.20925*thetad3*cos(theta3)*sin(theta4) + 0.20925*thetad3*cos(theta4)*sin(theta3) + 0.20925*thetad4*cos(theta3)*sin(theta4) + 0.20925*thetad4*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) - 0.031725*thetad3*sin(theta3)*sin(theta4) - 0.031725*thetad4*sin(theta3)*sin(theta4);
C(2,2) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 + 0.06345*thetad4*cos(theta4) + 0.4185*thetad4*sin(theta4) + 0.00290548*thetad5*cos(theta5)*sin(theta5);
C(2,3) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 - 0.00445*thetad1*cos(theta3) + 0.031725*thetad4*cos(theta4) - 0.459825*thetad1*sin(theta3) + 0.20925*thetad4*sin(theta4) + 0.031725*thetad1*cos(theta3)*cos(theta4) + 0.20925*thetad1*cos(theta3)*sin(theta4) + 0.20925*thetad1*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) - 0.031725*thetad1*sin(theta3)*sin(theta4);
C(2,4) = 0.06345*thetad1*cos(theta4) + 0.06345*thetad2*cos(theta4) + 0.031725*thetad3*cos(theta4) + 0.0186*thetad4*cos(theta4) + 0.000218418*thetad5*cos(theta5) + 0.4185*thetad1*sin(theta4) + 0.4185*thetad2*sin(theta4) + 0.20925*thetad3*sin(theta4) + 0.00636181*thetad5*sin(theta5) + 0.031725*thetad1*cos(theta3)*cos(theta4) + 0.20925*thetad1*cos(theta3)*sin(theta4) + 0.20925*thetad1*cos(theta4)*sin(theta3) - 0.031725*thetad1*sin(theta3)*sin(theta4);
C(2,5) = 0.0000662161*thetad1 + 0.0000662161*thetad2 + 0.0000662161*thetad3 - 0.000132432*thetad1*cos(theta5)^2 - 0.000132432*thetad2*cos(theta5)^2 - 0.000132432*thetad3*cos(theta5)^2 + 0.000218418*thetad4*cos(theta5) + 0.000436835*thetad5*cos(theta5) + 0.00636181*thetad4*sin(theta5) + 0.0127236*thetad5*sin(theta5) + 0.00290548*thetad1*cos(theta5)*sin(theta5) + 0.00290548*thetad2*cos(theta5)*sin(theta5) + 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(3,1) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 + 0.0089*thetad1*cos(theta3) + 0.00445*thetad2*cos(theta3) + 0.031725*thetad4*cos(theta4) + 0.91965*thetad1*sin(theta3) + 0.459825*thetad2*sin(theta3) + 0.20925*thetad4*sin(theta4) - 0.06345*thetad1*cos(theta3)*cos(theta4) - 0.031725*thetad2*cos(theta3)*cos(theta4) + 0.022425*thetad4*cos(theta3)*cos(theta4) - 0.4185*thetad1*cos(theta3)*sin(theta4) - 0.4185*thetad1*cos(theta4)*sin(theta3) - 0.20925*thetad2*cos(theta3)*sin(theta4) - 0.20925*thetad2*cos(theta4)*sin(theta3) + 0.20925*thetad4*cos(theta3)*sin(theta4) + 0.20925*thetad4*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) + 0.06345*thetad1*sin(theta3)*sin(theta4) + 0.031725*thetad2*sin(theta3)*sin(theta4) - 0.022425*thetad4*sin(theta3)*sin(theta4);
C(3,2) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 + 0.00445*thetad1*cos(theta3) + 0.031725*thetad4*cos(theta4) + 0.459825*thetad1*sin(theta3) + 0.20925*thetad4*sin(theta4) - 0.031725*thetad1*cos(theta3)*cos(theta4) - 0.20925*thetad1*cos(theta3)*sin(theta4) - 0.20925*thetad1*cos(theta4)*sin(theta3) + 0.00290548*thetad5*cos(theta5)*sin(theta5) + 0.031725*thetad1*sin(theta3)*sin(theta4);
C(3,3) = 0.0000662161*thetad5 - 0.000132432*thetad5*cos(theta5)^2 + 0.00290548*thetad5*cos(theta5)*sin(theta5);
C(3,4) = 0.031725*thetad1*cos(theta4) + 0.031725*thetad2*cos(theta4) + 0.000218418*thetad5*cos(theta5) + 0.20925*thetad1*sin(theta4) + 0.20925*thetad2*sin(theta4) + 0.00636181*thetad5*sin(theta5) + 0.022425*thetad1*cos(theta3)*cos(theta4) + 0.20925*thetad1*cos(theta3)*sin(theta4) + 0.20925*thetad1*cos(theta4)*sin(theta3) - 0.022425*thetad1*sin(theta3)*sin(theta4);
C(3,5) = 0.0000662161*thetad1 + 0.0000662161*thetad2 + 0.0000662161*thetad3 - 0.000132432*thetad1*cos(theta5)^2 - 0.000132432*thetad2*cos(theta5)^2 - 0.000132432*thetad3*cos(theta5)^2 + 0.000218418*thetad4*cos(theta5) + 0.000436835*thetad5*cos(theta5) + 0.00636181*thetad4*sin(theta5) + 0.0127236*thetad5*sin(theta5) + 0.00290548*thetad1*cos(theta5)*sin(theta5) + 0.00290548*thetad2*cos(theta5)*sin(theta5) + 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(4,1) = 0.000218418*thetad5*cos(theta5) - 0.06345*thetad2*cos(theta4) - 0.031725*thetad3*cos(theta4) - 0.06345*thetad1*cos(theta4) - 0.4185*thetad1*sin(theta4) - 0.4185*thetad2*sin(theta4) - 0.20925*thetad3*sin(theta4) + 0.00636181*thetad5*sin(theta5) - 0.06345*thetad1*cos(theta3)*cos(theta4) - 0.031725*thetad2*cos(theta3)*cos(theta4) - 0.022425*thetad3*cos(theta3)*cos(theta4) - 0.4185*thetad1*cos(theta3)*sin(theta4) - 0.4185*thetad1*cos(theta4)*sin(theta3) - 0.20925*thetad2*cos(theta3)*sin(theta4) - 0.20925*thetad2*cos(theta4)*sin(theta3) - 0.20925*thetad3*cos(theta3)*sin(theta4) - 0.20925*thetad3*cos(theta4)*sin(theta3) + 0.06345*thetad1*sin(theta3)*sin(theta4) + 0.031725*thetad2*sin(theta3)*sin(theta4) + 0.022425*thetad3*sin(theta3)*sin(theta4);
C(4,2) = 0.000218418*thetad5*cos(theta5) - 0.06345*thetad2*cos(theta4) - 0.031725*thetad3*cos(theta4) - 0.06345*thetad1*cos(theta4) - 0.4185*thetad1*sin(theta4) - 0.4185*thetad2*sin(theta4) - 0.20925*thetad3*sin(theta4) + 0.00636181*thetad5*sin(theta5) - 0.031725*thetad1*cos(theta3)*cos(theta4) - 0.20925*thetad1*cos(theta3)*sin(theta4) - 0.20925*thetad1*cos(theta4)*sin(theta3) + 0.031725*thetad1*sin(theta3)*sin(theta4);
C(4,3) = 0.000218418*thetad5*cos(theta5) - 0.031725*thetad2*cos(theta4) - 0.031725*thetad1*cos(theta4) - 0.20925*thetad1*sin(theta4) - 0.20925*thetad2*sin(theta4) + 0.00636181*thetad5*sin(theta5) - 0.022425*thetad1*cos(theta3)*cos(theta4) - 0.20925*thetad1*cos(theta3)*sin(theta4) - 0.20925*thetad1*cos(theta4)*sin(theta3) + 0.022425*thetad1*sin(theta3)*sin(theta4);
C(4,4) = 0.0;
C(4,5) = 0.000218418*thetad1*cos(theta5) + 0.000218418*thetad2*cos(theta5) + 0.000218418*thetad3*cos(theta5) + 0.00636181*thetad1*sin(theta5) + 0.00636181*thetad2*sin(theta5) + 0.00636181*thetad3*sin(theta5);
C(5,1) = 0.000132432*thetad1*cos(theta5)^2 - 0.0000662161*thetad2 - 0.0000662161*thetad3 - 0.0000662161*thetad1 + 0.000132432*thetad2*cos(theta5)^2 + 0.000132432*thetad3*cos(theta5)^2 - 0.000218418*thetad4*cos(theta5) - 0.00636181*thetad4*sin(theta5) - 0.00290548*thetad1*cos(theta5)*sin(theta5) - 0.00290548*thetad2*cos(theta5)*sin(theta5) - 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(5,2) = 0.000132432*thetad1*cos(theta5)^2 - 0.0000662161*thetad2 - 0.0000662161*thetad3 - 0.0000662161*thetad1 + 0.000132432*thetad2*cos(theta5)^2 + 0.000132432*thetad3*cos(theta5)^2 - 0.000218418*thetad4*cos(theta5) - 0.00636181*thetad4*sin(theta5) - 0.00290548*thetad1*cos(theta5)*sin(theta5) - 0.00290548*thetad2*cos(theta5)*sin(theta5) - 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(5,3) = 0.000132432*thetad1*cos(theta5)^2 - 0.0000662161*thetad2 - 0.0000662161*thetad3 - 0.0000662161*thetad1 + 0.000132432*thetad2*cos(theta5)^2 + 0.000132432*thetad3*cos(theta5)^2 - 0.000218418*thetad4*cos(theta5) - 0.00636181*thetad4*sin(theta5) - 0.00290548*thetad1*cos(theta5)*sin(theta5) - 0.00290548*thetad2*cos(theta5)*sin(theta5) - 0.00290548*thetad3*cos(theta5)*sin(theta5);
C(5,4) = - 0.000218418*thetad1*cos(theta5) - 0.000218418*thetad2*cos(theta5) - 0.000218418*thetad3*cos(theta5) - 0.00636181*thetad1*sin(theta5) - 0.00636181*thetad2*sin(theta5) - 0.00636181*thetad3*sin(theta5);
C(5,5) = 0.0;

G=zeros(5,1);
G(1) = 0.0;
G(2) = 24.8595*cos(theta2) - 0.260946*sin(theta2) + 18.0435*cos(theta2)*cos(theta3) - 0.174618*cos(theta2)*sin(theta3) - 0.174618*cos(theta3)*sin(theta2) - 18.0435*sin(theta2)*sin(theta3) + 8.21097*cos(theta2)*sin(theta3)*sin(theta4) + 8.21097*cos(theta3)*sin(theta2)*sin(theta4) + 8.21097*cos(theta4)*sin(theta2)*sin(theta3) - 1.24489*sin(theta2)*sin(theta3)*sin(theta4) - 8.21097*cos(theta2)*cos(theta3)*cos(theta4) + 1.24489*cos(theta2)*cos(theta3)*sin(theta4) + 1.24489*cos(theta2)*cos(theta4)*sin(theta3) + 1.24489*cos(theta3)*cos(theta4)*sin(theta2);
G(3) = 18.0435*cos(theta2)*cos(theta3) - 0.174618*cos(theta2)*sin(theta3) - 0.174618*cos(theta3)*sin(theta2) - 18.0435*sin(theta2)*sin(theta3) + 8.21097*cos(theta2)*sin(theta3)*sin(theta4) + 8.21097*cos(theta3)*sin(theta2)*sin(theta4) + 8.21097*cos(theta4)*sin(theta2)*sin(theta3) - 1.24489*sin(theta2)*sin(theta3)*sin(theta4) - 8.21097*cos(theta2)*cos(theta3)*cos(theta4) + 1.24489*cos(theta2)*cos(theta3)*sin(theta4) + 1.24489*cos(theta2)*cos(theta4)*sin(theta3) + 1.24489*cos(theta3)*cos(theta4)*sin(theta2);
G(4) = 8.21097*cos(theta2)*sin(theta3)*sin(theta4) + 8.21097*cos(theta3)*sin(theta2)*sin(theta4) + 8.21097*cos(theta4)*sin(theta2)*sin(theta3) - 1.24489*sin(theta2)*sin(theta3)*sin(theta4) - 8.21097*cos(theta2)*cos(theta3)*cos(theta4) + 1.24489*cos(theta2)*cos(theta3)*sin(theta4) + 1.24489*cos(theta2)*cos(theta4)*sin(theta3) + 1.24489*cos(theta3)*cos(theta4)*sin(theta2);
G(5) = 0.0;


taw_Lagrangian_num=(M*thetadd+C*thetad+G)

