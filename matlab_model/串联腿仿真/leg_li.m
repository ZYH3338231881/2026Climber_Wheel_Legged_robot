% 本程序用于腿部结构解算
clear;

%实车模型
l1=0.210;l4=l1;
l2=0.250;l3=l2;
l5=0.0; % 腿部杆长(单位：m)
%仿真模型
%l1=0.130;l4=l1;
%l2=0.240;l3=l2;
%l5=0.150; % 腿部杆长(单位：m)

% 逆运动学分析(Leg Inverse Kinematics)
syms l0 phi0;
Lca2 = l0^2+(l5/2)^2+l0*l5*cos(phi0);
Lce2 = l0^2+(l5/2)^2-l0*l5*cos(phi0);

cos_phi11 = ((l5/2)^2+Lca2-l0^2)/(l5*sqrt(Lca2));
cos_phi12 = (l1^2+Lca2-l2^2)/(2*l1*sqrt(Lca2));
cos_phi41 = ((l5/2)^2+Lce2-l0^2)/(l5*sqrt(Lce2));
cos_phi42 = (l4^2+Lce2-l3^2)/(2*l4*sqrt(Lce2));

phi11 = acos(cos_phi11);
phi12 = acos(cos_phi12);

phi41 = acos(cos_phi41);
phi42 = acos(cos_phi42);

phi1 = phi11 + phi12;
phi4 = pi-(phi41 + phi42);

joint_pos=[phi1; phi4];

matlabFunction(joint_pos,'File','D:\Desktop\2026RM\study\串联腿仿真\install\LegIKine.m');

disp("逆运动学分析完毕");

% 正运动学分析(Leg Forward Kinematics)
syms phi1 phi4 ;
%进行几何计算
xb=l1*cos(phi1);
yb=l1*sin(phi1);
xd=l5+l4*cos(phi4);
yd=l4*sin(phi4);

A0=2*l2*(xd-xb);
B0=2*l2*(yd-yb);
C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
phi2=2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));

xc=xb+l2*cos(phi2);
yc=yb+l2*sin(phi2);

l0=sqrt((xc-l5/2)^2+yc^2);
phi0=atan2(yc,(xc-l5/2));

leg_pos=[l0; phi0];

matlabFunction(leg_pos,'File','D:\Desktop\2026RM\study\串联腿仿真\install\LegFKine.m');

disp("正运动学分析完毕");


% 雅可比矩阵
J11=diff(l0,phi1);
J12=diff(l0,phi4);
J21=diff(phi0,phi1);
J22=diff(phi0,phi4);
J=[J11 J12; J21 J22];
matlabFunction(J,'File','D:\Desktop\2026RM\study\串联腿仿真\install\LegJacobian');

disp("雅可比矩阵计算完毕");

syms dphi1 dphi4;
speed=J*[dphi1; dphi4];

matlabFunction(speed,'File','D:\Desktop\2026RM\study\串联腿仿真\install\LegSpeed');

disp("腿部速度分析完毕");

% 动态静力学
syms F Tp;
T=J'*[F; Tp];

matlabFunction(T,'File','D:\Desktop\2026RM\study\串联腿仿真\install\LegTransform');

disp("动态静力学分析完毕");