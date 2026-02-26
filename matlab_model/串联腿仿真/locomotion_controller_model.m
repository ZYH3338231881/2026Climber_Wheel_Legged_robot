 clear;
%本仿真适用于2026Climber战队串联腿步兵
L0s=0.13:0.01:0.40; % L0变化范围应该对应串联腿的最长和最短腿长0.13到0.4  间隔0.01
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K 后期对其线性拟合

for step=1:length(L0s)
    %fprintf('step=%d\n', step);
    % 所需符号量
    syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
    syms x x1 x2;
    syms phi phi1 phi2;
    syms T Tp N P Nm Pm Nf t;
    
    % 机器人结构参数(国际单位制) 注意这里得输出我们串联腿的参数     32代码中是对单个腿进行计算
    R=0.06;                     %轮半径                 --->  solidwork m
    L=L0s(step)/2;              %摆杆重心到驱动轮轴的距离 --->   solidwork
    Lm=L0s(step)/2;             %摆杆重心到机体转轴距离   --->   solidwork
    l=0.03;                        %机体重心到其转轴距离     --->   solidwork
    mw=0.5;              %一个轮子质量     --->>记得称重
    mp=3;       %一摆杆质量     --->>记得称重
    M= 16;                %机体质量         --->>记得称重
    Iw=0.5*mw*R*R;       %驱动轮转子的转动惯量   --->>计算
    Ip=mp*((L+Lm)^2+0.05^2)/12;%摆杆绕质心的转动惯量   --->>计算
    Im=M*(0.21^2+0.58^2)/12;         %机体绕质心得转动惯量   --->>计算
    g=9.8;                      %重力加速度

    % 对机体建模后的数据进行物理分析：求解二阶微分方程
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));%机体的水平方向
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));%机体竖直方向
    % 对摆杆建模后的数据进行物理分析：求解二阶微分方程
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
    
    equ1=x2-(T-N*R)/(Iw/R+mw*R); %轮子
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;  %机子
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;     %棍子
    
    %求解前三个变量----->>系统非线性模型符号表达式
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);

    % 求得雅克比矩阵，然后得到状态空间方程
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));

    % 离散化
    [G,H]=c2d(double(A),double(B),0.001);%最后一个参数是离散化的步长(s)
    %G系统矩阵  H空间矩阵

     Q_theta     = 4000;
     Q_theta_dot = 150;%防止腿部关节过冲
     Q_x         = 400;
     Q_x_dot     = 1000;
     Q_phi       = 80000;
     Q_phi_dot   = 10;
     Q=diag([Q_theta Q_theta_dot Q_x Q_x_dot Q_phi Q_phi_dot]);
     R=diag([300 12]);  

     %步兵较软参数 起立参数
     % Q_theta     = 500;
     % Q_theta_dot = 100;%防止腿部关节过冲
     % Q_x         = 500;
     % Q_x_dot     = 50;
     % Q_phi       = 40000;
     % Q_phi_dot   = 10;
     % Q=diag([Q_theta Q_theta_dot Q_x Q_x_dot Q_phi Q_phi_dot]);
     % R=diag([240 12]);  




    % 求解反馈矩阵K
    Ks(:,:,step)=lqr(double(A),double(B),Q,R);
    
end
% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
for x=1:2 
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 也可以显示为更紧凑的形式
% fprintf('\n紧凑形式的K矩阵：\n');
% disp(vpa(K, 6));
% 打印K矩阵的C语言数组格式
fprintf('k[0][0] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,1), L0, 'All')));
fprintf('k[0][1] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,2), L0, 'All')));
fprintf('k[0][2] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,3), L0, 'All')));
fprintf('k[0][3] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,4), L0, 'All')));
fprintf('k[0][4] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,5), L0, 'All')));
fprintf('k[0][5] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(1,6), L0, 'All')));

fprintf('k[1][0] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,1), L0, 'All')));
fprintf('k[1][1] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,2), L0, 'All')));
fprintf('k[1][2] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,3), L0, 'All')));
fprintf('k[1][3] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,4), L0, 'All')));
fprintf('k[1][4] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,5), L0, 'All')));
fprintf('k[1][5] = %.4ff * t3 + %.4ff * t2 + %.4ff * t1 + %.4ff;\n', double(coeffs(K(2,6), L0, 'All')));

% 输出到m函数
matlabFunction(K, 'File', 'D:\Desktop\2026RM\study\串联腿仿真\install\L2K.m');


%% 计算特定腿长L0=0.18m时的K矩阵
fprintf('\n===========================================\n');
fprintf('腿长L0=0.18m时的K矩阵计算\n');
fprintf('===========================================\n');

% 计算L0=0.18m时的K矩阵
L0_target = 0.18;
K_target = zeros(2,6);

for x = 1:2
    for y = 1:6
        p = coeffs(K(x,y), L0, 'All');
        if length(p) == 4
            K_target(x,y) = p(1)*L0_target^3 + p(2)*L0_target^2 + ...
                           p(3)*L0_target + p(4);
        elseif length(p) == 3
            K_target(x,y) = p(1)*L0_target^2 + p(2)*L0_target + p(3);
        elseif length(p) == 2
            K_target(x,y) = p(1)*L0_target + p(2);
        else
            K_target(x,y) = p;
        end
    end
end

% 输出K矩阵
fprintf('\nK矩阵（2x6）：\n');
fprintf('第一行（对应轮子力矩T）：\n');
fprintf('[%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K_target(1,:));
fprintf('第二行（对应摆杆力矩Tp）：\n');
fprintf('[%8.4f, %8.4f, %8.4f, %8.4f, %8.4f, %8.4f]\n', K_target(2,:));

