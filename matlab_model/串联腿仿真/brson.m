%theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
%x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
%phi   : 机体与水平夹角                LM  : 摆杆重心到其转轴距离
%T     ：驱动轮输出力矩                 l   : 机体重心到其转轴距离
%Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
%N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
%P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量
%Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
%Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
%Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量

syms x(t) T Iw mw theta(t) phi(t) Tp 

m_p = 0.045; % 摆杆质量，这里指的是轮子除聚氨酯部分的质量
m_w = 0.6; % 驱动轮转子质量
M = 1.44;   % 机体质量

L = 0.5;   % 摆杆重心到驱动轮轴距离
L_m = 0.5; % 摆杆重心到机体转轴距离
l = 0.011;   % 机体重心到其转轴距离
R = 0.0603;   % 驱动轮半径
I_p = m_p*((L+L_m)^2+0.048^2)/12.0; % 摆杆绕质心转动惯量
I_m = M*(0.135^2+0.066^2)/12.0; % 机体绕质心转动惯量
I_w = 0.5*m_w*R^2; % 驱动轮转子转动惯量

% 状态变量一阶导数
dx = diff(x,t);
dtheta = diff(theta,t);
dphi = diff(phi,t);

% 状态变量二阶导数
ddx = diff(x,t,2);
ddtheta = diff(theta,t,2);
ddphi = diff(phi,t,2);

g = 9.8;

% 摆杆对机体力水平方向分量 
N_M = M * (ddx + ((L+L_m)*ddtheta*cos(theta) - (L+L_m)*dtheta*dtheta*sin(theta)) - (ddphi*l*cos(phi) - dphi*dphi*l*sin(phi)));
% 驱动轮对摆杆力的水平分量
N = N_M + m_p*(ddx + L*ddtheta*cos(theta) - L*dtheta*dtheta*sin(theta));
% 摆杆对机体力竖直方向分量
P_M = M*(-((L+L_m)*ddtheta*sin(theta) + (L+L_m)*dtheta*dtheta*cos(theta)) - (l*ddphi*sin(phi) + l*dphi*dphi*cos(phi))) + M*g;
% 驱动轮对摆杆力的竖直分量
P = P_M + m_p*g + m_p*(-L*ddtheta*sin(theta) - L*dtheta*dtheta*cos(theta));

eqn1 = ddx == (T -N*R)/((I_w/R) + m_w*R);
eqn2 = I_p*ddtheta == (P*L + P_M*L_m)*sin(theta)-(N*L+N_M*L_m)*cos(theta)-T+Tp;
eqn3 = I_m*ddphi == Tp +N_M*l*cos(phi)+P_M*l*sin(phi);

% 求解系统非线性模型符号表达式，也就是ddtheta,ddx,ddphi
[ddtheta,ddx,ddphi] = solve(eqn1,eqn2,eqn3,ddtheta,ddx,ddphi);

% 求解雅可比矩阵，带入系统平衡点【0，0，x，0，0，0】【0，0】
A_jac=jacobian([dtheta,ddtheta,dx,ddx,dphi,ddphi],[theta,dtheta,x,dx,phi,dphi]);
A = subs(A_jac,{theta,dtheta,dx,phi,dphi,T,Tp},{0,0,0,0,0,0,0});
A = double(A);

B_jac=jacobian([dtheta,ddtheta,dx,ddx,dphi,ddphi],[T,Tp]);
B = subs(B_jac,{theta,dtheta,dx,phi,dphi,T,Tp},{0,0,0,0,0,0,0});
B = double(B);
% B=subs(jacobian([dtheta,ddtheta,dx,ddx,dphi,ddphi],[T,Tp]),[theta,dtheta,dx,phi,dphi,T,Tp],[0,0,0,0,0,0,0]);   

%设置Q、R矩阵

Q=diag([1 0.07 10 5 300 0.6]);
R=[20 0;0,1];                %T Tp

K=lqr(A,B,Q,R);

fprintf('***************************\n'); 
fprintf('A矩阵为：\n');   
disp(A)
fprintf('B矩阵为：\n');   
disp(B)
fprintf('***************************\n'); 
fprintf('K增益矩阵为：\n');   
disp(K)
% fprintf(ddtheta);