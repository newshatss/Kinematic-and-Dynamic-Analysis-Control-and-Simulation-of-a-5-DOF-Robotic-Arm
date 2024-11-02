%% transformation matrix
syms theta1 theta2 theta3 theta4 theta5

T01 = [cos(theta1),0,sin(theta1),0;sin(theta1),0,-cos(theta1),0;0,1,0,0;0,0,0,1];
T12 = [cos(theta2),-sin(theta2),0,0.50*cos(theta2);sin(theta2),cos(theta2),0,0.50*sin(theta2);0,0,1,0;0,0,0,1];
T23 = [cos(theta3),-sin(theta3),0,0.50*cos(theta3);sin(theta3),cos(theta3),0,0.50*sin(theta3);0,0,1,0;0,0,0,1];
T34 =[cos(theta4),0,-sin(theta4),0;sin(theta4),0,cos(theta4),0;0,-1,0,0;0,0,0,1];%-pi/2
T45 = [cos(theta5),-sin(theta5),0,0;sin(theta5),cos(theta5),0,0;0,0,1,0.15;0,0,0,1];

T02 = T01 * T12;
T03 = T01 * T12 * T23; 
T04 = T01 * T12 * T23 * T34;
T05 = T01 * T12 * T23 * T34 * T45;

%% velocity & acceleration propagation(Newton-Euler)
%p
P01=T01(1:3,4);
P12=T12(1:3,4);
P23=T23(1:3,4);
P34=T34(1:3,4);
P45=T45(1:3,4);

%R
R01=T01(1:3,1:3);
R12=T12(1:3,1:3);
R23=T23(1:3,1:3);
R34=T34(1:3,1:3);
R45=T45(1:3,1:3);


R10=transpose(R01);
R21=transpose(R12);
R32=transpose(R23);
R43=transpose(R34);
R54=transpose(R45);


syms thetad1 thetad2 thetad3 thetad4 thetad5 thetadd1 thetadd2 thetadd3 thetadd4 thetadd5
% w
w00=[0;0;0];
w11=R10*w00+[0;0;thetad1];
w22=R21*w11+[0;0;thetad2];
w33=R32*w22+[0;0;thetad3];
w44=R43*w33+[0;0;thetad4];
w55=R54*w44+[0;0;thetad5];

% v
v00=[0;0;0];
v11=R10*(v00+cross(w00,P01));
v22=R21*(v11+cross(w11,P12));
v33=R32*(v22+cross(w22,P23));
v44=R43*(v33+cross(w33,P34));
v55=R54*(v44+cross(w44,P45));


% wee&vee
R05=R01 * R12 * R23 * R34 * R45;
v05= R05 * v55;
w05= R05 * w55;


% wdot
w11_d = simplify(R10*[0;0;0]+R10*(cross([0;0;0],[0;0;thetad1]))+[0;0;thetadd1]);
w22_d = simplify(R21*w11+R21*(cross(w11,[0;0;thetad2]))+[0;0;thetadd2]);
w33_d = simplify(R32*w22+R32*(cross(w22,[0;0;thetad3]))+[0;0;thetadd3]);
w44_d = simplify(R43*w33+R43*(cross(w33,[0;0;thetad4]))+[0;0;thetadd4]);
w55_d = simplify(R54*w44+R54*(cross(w44,[0;0;thetad5]))+[0;0;thetadd5]);

% v dot
v11_d=R10*(cross([0;0;0],P01)+cross([0;0;0],cross([0;0;0],P01))+[0;0;9.81]);
v22_d=R21*(cross(w11_d,P12)+cross(w11,cross(w11,P12))+v11_d);
v33_d=R32*(cross(w22_d,P23)+cross(w22,cross(w22,P23))+v22_d);
v44_d=R43*(cross(w33_d,P34)+cross(w33,cross(w33,P34))+v33_d);
v55_d=R54*(cross(w44_d,P45)+cross(w44,cross(w44,P45))+v44_d);

%v dot center of mass
v1c1_d=cross(w11_d,[-0.02;0;0.02])+cross(w11,cross(w11,[-0.02;0;0.02]))+v11_d;
v2c2_d=cross(w22_d,[0.27;0.02;-0.06])+cross(w22,cross(w22,[0.27;0.02;-0.06]))+v22_d;
v3c3_d=cross(w33_d,[0.37;0.02;-0.05])+cross(w33,cross(w33,[0.37;0.02;-0.05]))+v33_d;
v4c4_d=cross(w44_d,[-0.45;-0.02;-0.09])+cross(w44,cross(w44,[-0.45;-0.02;-0.09]))+v44_d;
v5c5_d=cross(w55_d,[0;-0.02;-0.53])+cross(w55,cross(w55,[0;-0.02;-0.53]))+v55_d;

%% mass properties,Inertia,F,f,N,n,taw
%m [kg]
m1 = 0.33;
m2 = 1.33;
m3 = 0.89;
m4 = 1.86;
m5 = 0.27;
%F
F11 = v1c1_d * m1;
F22 = v2c2_d * m2;
F33 = v3c3_d * m3;
F44 = v4c4_d * m4;
F55 = v5c5_d * m5;
%inertia
I11 = [474862.68,10.87,3812.88;10.87,963857.85,10.91;3812.88,10.91,1051823.40].*1e-9;
I22 = [2273563.60, 1527184.20, -3294838.30;1527184.20, 95421453.60, -179976.76;-3294838.30, -179976.76, 95626327.64].*1e-9;
I33 = [1059667.14,1824954.06,-37980.79;1824954.06,43565913.89,71512.94;-37980.79,71512.94,43856469.41].*1e-9;
I44 = [7673736.40, 12723624.85, 436835.23; 12723624.85, 163621941.46, 66216.10; 436835.23, 66216.10, 166527424.84].*1e-9;
I55 = [166527424.84, -66216.10, -436835.23; -66216.10, 163621941.46, 12723624.85; -436835.23, 12723624.85, 7673736.40].*1e-9;

%N
N11 = I11 * w11_d + cross(w11,I11*w11);
N22 = I22 * w22_d + cross(w22,I22*w22);
N33 = I33 * w33_d + cross(w33,I33*w33);
N44 = I44 * w44_d + cross(w44,I44*w44);
N55 = I55 * w55_d + cross(w55,I55*w55);
%f
f55=F55;
f44=R45*f55+F44;
f33=R34*f44+F33;
f22=R23*f33+F22;
f11=R12*f22+F11;
%n
n55=N55;
n44=N44+R45*n55+cross([-0.45;-0.02;-0.09],F44)+cross(P45,R45*f55);
n33=N33+R34*n44+cross([0.37;0.02;-0.05],F33)+cross(P34,R34*f44);
n22=N22+R23*n33+cross([0.27;0.02;-0.06],F22)+cross(P23,R23*f33);
n11=N11+R12*n22+cross([-0.02;0;0.02],F11)+cross(P12,R12*f22);

% n55=N55;
% n44=N44+R45*n55+cross([-0.45;-0.02;-0.09],F44)+cross(-P45,R45*f55);
% n33=N33+R34*n44+cross([0.37;0.02;-0.05],F33)+cross(-P34,R34*f44);
% n22=N22+R23*n33+cross([0.27;0.02;-0.06],F22)+cross(-P23,R23*f33);
% n11=N11+R12*n22+cross([-0.02;0;0.02],F11)+cross(-P12,R12*f22);
%% taw

taw5=transpose(n55)*[0;0;1];
taw4=transpose(n44)*[0;0;1];
taw3=transpose(n33)*[0;0;1];
taw2=transpose(n22)*[0;0;1];
taw1=transpose(n11)*[0;0;1];
% taw matrix
taw =[vpa(simplify(taw1),3) ;vpa(simplify(taw2),3) ;vpa(simplify(taw3),3) ;vpa(simplify(taw4),3) ;vpa(simplify(taw5),3)];
%% G
thetadd1=0;thetadd2=0;thetadd3=0;thetadd4=0;thetadd5=0;
thetad1=0;thetad2=0;thetad3=0;thetad4=0;thetad5=0;
g(1)=eval(taw1);
g(2)=eval(taw2);
g(3)=eval(taw3);
g(4)=eval(taw4);
g(5)=eval(taw5);
G=transpose(g);
%% M

% % M?
% for i = 1:5
%     thetadd = zeros(5, 1);
%     thetadd(i) = 1; % Set one joint acceleration to 1
%     % Evaluate torques and subtract gravity contribution
%     M(:, i) = [eval(taw1) - G(1);
%                eval(taw2) - G(2);
%                eval(taw3) - G(3);
%                eval(taw4) - G(4);
%                eval(taw5) - G(5)];
% end
% 
% %For First column
thetadd1=1;thetadd2=0;thetadd3=0;
thetadd4=0;thetadd5=0;
thetad1=0;thetad2=0;thetad3=0;
thetad4=0;thetad5=0;

M(1,1)=eval(taw1)-G(1);
M(2,1)=eval(taw2)-G(2);
M(3,1)=eval(taw3)-G(3);
M(4,1)=eval(taw4)-G(4);
M(5,1)=eval(taw5)-G(5);

% For column 2
thetadd1=0;thetadd2=1;thetadd3=0;
thetadd4=0;thetadd5=0;

M(1,2)=eval(taw1)-G(1);
M(2,2)=eval(taw2)-G(2);
M(3,2)=eval(taw3)-G(3);
M(4,2)=eval(taw4)-G(4);
M(5,2)=eval(taw5)-G(5);

% For column 3
thetadd1=0;thetadd2=0;thetadd3=1;
thetadd4=0;thetadd5=0;

M(1,3)=eval(taw1)-G(1);
M(2,3)=eval(taw2)-G(2);
M(3,3)=eval(taw3)-G(3);
M(4,3)=eval(taw4)-G(4);
M(5,3)=eval(taw5)-G(5);


%For column 4
thetadd1=0;thetadd2=0;thetadd3=0;
thetadd4=1;thetadd5=0;

M(1,4)=eval(taw1)-G(1);
M(2,4)=eval(taw2)-G(2);
M(3,4)=eval(taw3)-G(3);
M(4,4)=eval(taw4)-G(4);
M(5,4)=eval(taw5)-G(5);

% For column 5
thetadd1=0;thetadd2=0;thetadd3=0;
thetadd4=0;thetadd5=1;

M(1,5)=eval(taw1)-G(1);
M(2,5)=eval(taw2)-G(2);
M(3,5)=eval(taw3)-G(3);
M(4,5)=eval(taw4)-G(4);
M(5,5)=eval(taw5)-G(5);
%% C 
%column 1
thetadd1=0;thetadd2=0;thetadd3=0;thetadd4=0;thetadd5=0;
thetad1=1;thetad2=0;thetad3=0;thetad4=0;thetad5=0;

C(1,1)=eval(taw1)-G(1);
C(2,1)=eval(taw2)-G(2);
C(3,1)=eval(taw3)-G(3);
C(4,1)=eval(taw4)-G(4);
C(5,1)=eval(taw5)-G(5);

%column 2
thetad1=0;thetad2=1;thetad3=0;thetad4=0;thetad5=0;

C(1,2)=eval(taw1)-G(1);
C(2,2)=eval(taw2)-G(2);
C(3,2)=eval(taw3)-G(3);
C(4,2)=eval(taw4)-G(4);
C(5,2)=eval(taw5)-G(5);

%column 3
thetad1=0;thetad2=0;thetad3=1;thetad4=0;thetad5=0;

C(1,3)=eval(taw1)-G(1);
C(2,3)=eval(taw2)-G(2);
C(3,3)=eval(taw3)-G(3);
C(4,3)=eval(taw4)-G(4);
C(5,3)=eval(taw5)-G(5);

%column 4
thetad1=0;thetad2=0;thetad3=0;thetad4=1;thetad5=0;

C(1,4)=eval(taw1)-G(1);
C(2,4)=eval(taw2)-G(2);
C(3,4)=eval(taw3)-G(3);
C(4,4)=eval(taw4)-G(4);
C(5,4)=eval(taw5)-G(5);

%column 5
thetad1=0;thetad2=0;thetad3=0;thetad4=0;thetad5=1;

C(1,5)=eval(taw1)-G(1);
C(2,5)=eval(taw2)-G(2);
C(3,5)=eval(taw3)-G(3);
C(4,5)=eval(taw4)-G(4);
C(5,5)=eval(taw5)-G(5);

clear thetad1 thetad2 thetad3 thetad4 thetad5 
syms thetad1 thetad2 thetad3 thetad4 thetad5

thetad=[thetad1;thetad2;thetad3;thetad4;thetad5];
V=C*thetad;

% % Define symbolic variables for angular velocities
% syms thetad1 thetad2 thetad3 thetad4 thetad5
% 
% % Initialize Coriolis matrix C
% C = sym(zeros(5,5));
% 
% Iterate over each column to compute C matrix
% for i = 1:5
%     % Set all angular velocities to zero
%     thetad1 = 0; thetad2 = 0; thetad3 = 0; thetad4 = 0; thetad5 = 0;
% 
%     % Set current angular velocity to 1
%     eval(sprintf('thetad%d = 1;', i));
% 
%     % Compute each element of the current column
%     C(1,i) = eval(taw1) - G(1);
%     C(2,i) = eval(taw2) - G(2);
%     C(3,i) = eval(taw3) - G(3);
%     C(4,i) = eval(taw4) - G(4);
%     C(5,i) = eval(taw5) - G(5);
% end

% Define angular velocities as symbolic variables for future use
% syms thetad1 thetad2 thetad3 thetad4 thetad5
% 
% % Define angular velocity vector
% thetad = [thetad1; thetad2; thetad3; thetad4; thetad5];
% 
% % Compute velocity vector
% V = C * thetad;

%% numerical
theta1=0;
theta2=0;
theta3=0;
theta4=0;
theta5=0;
% theta1=deg2rad(0);
% theta2=deg2rad(-90);
% theta3=deg2rad(0);
% theta4=deg2rad(0);
% theta5=deg2rad(0);
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
M_numerical=eval(M);
C_numerical=eval(C);
G_numerical=eval(G);
thetadd=[thetadd1;thetadd2;thetadd3;thetadd4;thetadd5];
taw_NewtonEuler_numerical=eval(M*thetadd + V + G)
%taw_answer_symbolic=M*thetadd + V + G;
% 
