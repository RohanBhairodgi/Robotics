close all;
clear all;
mm = 1/1000;
steps = 1000000;
Ti = eye(4);

T2=[0, 1, 0, 0; -1, 0, 0, -0.0662; 0, 0, 1, 0.0431; 0, 0, 0, 1];
roll_x =  -172.95718336855933;
pitch_y = -27.847557028831005;
yaw_z = 68.70697922863141;
angles = [yaw_z pitch_y roll_x];
rad = deg2rad(angles);
rot_ac = eul2rotm(rad,'ZYX');
T3(1:3,1:3) = rot_ac;
T3(4,:) = [0, 0, 0];
T3(:,4) = [-0.14195360128424114,-0.06062556383004704 ,0.3528046636209403,1];

T4=[1,0,0,0.103975 ; 0,1,0,-0.103975 ; 0,0,1,0 ; 0,0,0,1];
Tf=T2*T3*T4;
qc=[-77.26 -38.76 26.22 93.29 -56.69 -59.94 118];
the = deg2rad(qc);
a=zeros(7);
d= [0.34, 0, 0.4, 0, 0.4, 0, 0.126];
alpha=[-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0];

A1=[cos(the(1)), -sin(the(1))*cos(alpha(1)), sin(the(1))*sin(alpha(1)), a(1)*cos(the(1));
sin(the(1)), cos(the(1))*cos(alpha(1)), -cos(the(1))*sin(alpha(1)), a(1)*sin(the(1));
0, sin(alpha(1)), cos(alpha(1)), d(1);
0, 0, 0, 1;];
A2=[cos(the(2)), -sin(the(2))*cos(alpha(2)), sin(the(2))*sin(alpha(2)), a(2)*cos(the(2));
sin(the(2)), cos(the(2))*cos(alpha(2)), -cos(the(2))*sin(alpha(2)), a(2)*sin(the(2));
0, sin(alpha(2)), cos(alpha(2)), d(2);
0, 0, 0, 1;];
A3=[cos(the(3)), -sin(the(3))*cos(alpha(3)), sin(the(3))*sin(alpha(3)), a(3)*cos(the(3));
sin(the(3)), cos(the(3))*cos(alpha(3)), -cos(the(3))*sin(alpha(3)), a(3)*sin(the(3));
0, sin(alpha(3)), cos(alpha(3)), d(3);
0, 0, 0, 1;];
A4=[cos(the(4)), -sin(the(4))*cos(alpha(4)), sin(the(4))*sin(alpha(4)), a(4)*cos(the(4));
sin(the(4)), cos(the(4))*cos(alpha(4)), -cos(the(4))*sin(alpha(4)), a(4)*sin(the(4));
0, sin(alpha(4)), cos(alpha(4)), d(4);
0, 0, 0, 1;];
A5=[cos(the(5)), -sin(the(5))*cos(alpha(5)), sin(the(5))*sin(alpha(5)), a(5)*cos(the(5));
sin(the(5)), cos(the(5))*cos(alpha(5)), -cos(the(5))*sin(alpha(5)), a(5)*sin(the(5));
0, sin(alpha(5)), cos(alpha(5)), d(5);
0, 0, 0, 1;];
A6=[cos(the(6)), -sin(the(6))*cos(alpha(6)), sin(the(6))*sin(alpha(6)), a(6)*cos(the(6));
sin(the(6)), cos(the(6))*cos(alpha(6)), -cos(the(6))*sin(alpha(6)), a(6)*sin(the(6));
0, sin(alpha(6)), cos(alpha(6)), d(6);
0, 0, 0, 1;];
A7=[cos(the(7)), -sin(the(7))*cos(alpha(7)), sin(the(7))*sin(alpha(7)), a(7)*cos(the(7));
sin(the(7)), cos(the(7))*cos(alpha(7)), -cos(the(7))*sin(alpha(7)), a(7)*sin(the(7));
0, sin(alpha(7)), cos(alpha(7)), d(7);
0, 0, 0, 1;];
A17= A1*A2*A3*A4*A5*A6*A7;
Tbt=A17*Tf;
yef = 0.033+(0.025/2);
zef = 0.0600;
xef = 0.00;
Tef(1:3,1:3) = eye(3);

ef_rot_z = myrotmat(-pi/2,'z');
ef_rot_x = myrotmat(pi,'x');
ef_new_rot = ef_rot_z*ef_rot_x;
Tef(1:3,1:3) = ef_new_rot;
Tef(4,:) = [0 0 0];
Tef(:,4) = [xef yef zef 1];
Tbe= Tbt*inv(Tef);
d_pose = pose(Tbe);
phi=0.4069;
theta=3.0995;
shi=-2.1828;
Ta = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -sin(phi) cos(phi)*sin(theta);
      0 0 0 0 cos(phi) sin(phi)*sin(theta);
      0 0 0 1 0 cos(theta)];
  
Q = zeros(7, steps);
   Q(:,1) = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712  ]);
K = 10*eye(6,6);
e = zeros(6,steps);
for i = 1:steps 
    [T02e, A02e] = FK(d, transpose(Q(:,i)), a, alpha, Ti); 
    xe = pose(T02e);
    e(:,i) = d_pose - xe;
    Ja = Jg2Ja(A02e,Ta);
    qdot = pinv(Ja)*K*e(:,i);
    if  Q(1,i)>= deg2rad(-170) && Q(1,i) <= deg2rad(170) 
        Q(1,i+1) = Q(1,i) + qdot(1,1)*0.01; 
    else
        Q(1,i+1) = deg2rad(-170);
    end
    if  Q(2,i)>= deg2rad(-120) && Q(2,i) <= deg2rad(120)
        Q(2,i+1) = Q(2,i) + qdot(2,1)*0.01; 
    else
        Q(2,i+1) = deg2rad(-120);
    end
    if  Q(3,i)>= deg2rad(-170) && Q(3,i) <= deg2rad(170)
        Q(3,i+1) = Q(3,i) + qdot(3,1)*0.01; 
    else
        Q(3,i+1) = deg2rad(-170);
    end
    if  Q(4,i)>= deg2rad(-120) && Q(4,i) <= deg2rad(120)
        Q(4,i+1) = Q(4,i) + qdot(4,1)*0.01; 
    else
        Q(4,i+1) = deg2rad(-120);
    end
    if  Q(5,i)>= deg2rad(-170) && Q(5,i) <= deg2rad(170)
        Q(5,i+1) = Q(5,i) + qdot(5,1)*0.01; 
    else
        Q(5,i+1) = deg2rad(-170);
    end
    if  Q(6,i)>= deg2rad(-120) && Q(6,i) <= deg2rad(120)
        Q(6,i+1) = Q(6,i) + qdot(6,1)*0.01; 
    else
        Q(6,i+1) = deg2rad(-120);
    end
    if  Q(7,i)>= deg2rad(-175) && Q(7,i) <= deg2rad(175)
        Q(7,i+1) = Q(7,i) + qdot(7,1)*0.01; 
    else
        Q(7,i+1) = deg2rad(-120);
    end
    if (max(abs(e(:,i))) < 0.0001)
            qfinal = Q(:,i); 
            break;
    end
end
[T0des, A0des] = FK(d, transpose(qfinal), a, alpha, Ti);
qfin_deg =  rad2deg(qfinal(:,1));
a0 = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712 ]);
a1 = 0;
a2 = 0;
A = [10^3, 10^4, 10^5; 3*10^2, 4*10^3, 5*10^4; 6*10, 12*10^2, 20*10^3];
x = zeros(3,7);
b = zeros(3,7);

for i = 1:7
    b(:,i) = [qfinal(i,1) - a0(1,i); 0; 0];
    x(:,i) = A\b(:,i);
end
Tinitial = 0;
Tfinalt = 10;
t = transpose(linspace(Tinitial,Tfinalt,2000));
lt = length(t);

PoseG = zeros(7,lt);
VeloG = zeros(7,lt);

for i = 1:length(t)
    for j = 1:7
       PoseG(j,i) = x(3,j)*t(i)^5 + x(2,j)*t(i)^4 + x(1,j)*t(i)^3 + a2*t(i)^2 + a1*t(i) + a0(j); %Joint trajectory
       Qgraph=PoseG;
    end
end


for i = 1:length(t)
    for j = 1:7
        VeloG(j,i) = 5*x(3,j)*t(i)^4 + 4*x(2,j)*t(i)^3 + 3*x(1,j)*t(i)^2 + 2*a2*t(i) + a1; % Velocity Profile
        Qdgraph=rad2deg(VeloG);
    end
end
writematrix(round(Qgraph,4)','Bhairodagi_Rohan_file.txt','Delimiter','space')

% subplot(3,1,1);
% x = t;
% y1 = PoseG;
% plot(x,y1)
% title('Joint Trajectory')
% 
% subplot(3,1,2); 
% y2 = VeloG;
% plot(x,y2)
% title('Velocity Profile')

function [T, Tloop]  = FK(d, q, a, alpha, Tin)
    T = Tin;
    R = length(a);
    Tloop = cell(1,R);
    for j = 1:R
        t = [cos(q(1,j)) -sin(q(1,j))*cos(alpha(1,j))  sin(q(1,j))*sin(alpha(1,j)) a(1,j)*cos(q(1,j));
             sin(q(1,j))  cos(q(1,j))*cos(alpha(1,j)) -cos(q(1,j))*sin(alpha(1,j)) a(1,j)*sin(q(1,j));
                          0                   sin(alpha(1,j))                  cos(alpha(1,j))                 d(1,j);
                          0                                  0                              0                      1];
        T = T * t;
        Tloop{1,j} = T;
        
    end
end

function ps = pose(Td)
    pd  = [Td(1:3,4)];
    r23 = Td(2,3);
    r13 = Td(1,3);
    r33 = Td(3,3);
    r32 = Td(3,2);
    r31 = Td(3,1);
    %From Rotation to euler zyz
    phi = atan2(r23,r13);
    theta = atan2(sqrt(r13^2+r23^2),r33);
    psi = atan2(r32,-r31);
    phid = [phi; theta; psi];
    ps = [pd;phid];
end 

function Ja = Jg2Ja(Ac,Ta)
    %Performs Jacobian Here from Transformations
    z0 = [0; 0; 1]; 
    p0 = [0; 0; 0];
    pe = Ac{1,7}(1:3,3);
    
    z1 = Ac{1,1}(1:3,3); 
    p1 = Ac{1,1}(1:3,4);
    
    z2 = Ac{1,2}(1:3,3);
    p2 = Ac{1,2}(1:3,4);
    
    z3 = Ac{1,3}(1:3,3); 
    p3 = Ac{1,3}(1:3,4);
    
    z4 = Ac{1,4}(1:3,3); 
    p4 = Ac{1,4}(1:3,4);
    
    z5 = Ac{1,5}(1:3,3); 
    p5 = Ac{1,5}(1:3,4);
    
    z6 = Ac{1,6}(1:3,3); 
    p6 = Ac{1,6}(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
        z0 z1 z2 z3 z4 z5 z6];
    
    Ja = inv(Ta)*Jg; 
end
  
function R = myrotmat(theta, axis)
   switch axis
        case {'x','X'}
            R = [1 0 0 ; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
        case {'y','Y'}
            R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        case {'z','Z'}
            R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
        otherwise
            disp('Unknown axis. Please use x, y or z');
            R = [];
    end
end