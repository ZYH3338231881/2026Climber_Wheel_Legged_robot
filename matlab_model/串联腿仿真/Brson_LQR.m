clear;
% 本仿真适用于2026Climber战队串联腿步兵

% Bryson法则全局参数（可根据不同战术模式调整）
RHO_BALANCE = 1.0;  % 平衡因子：1.0=标准, 1.5=激进, 0.7=保守

L0s = 0.13:0.01:0.40;  % 腿长变化范围
Ks = zeros(2,6,length(L0s));

for step = 1:length(L0s)
    fprintf('计算腿长 L0 = %.2f m ...\n', L0s(step));
    
    % ===== 原有符号运算部分（保持不变）=====
    syms theta theta1 theta2 x x1 x2 phi phi1 phi2 T Tp N P Nm Pm
    R = 0.06; mw = 0.5; mp = 2.5; M = 13; g = 9.8;
    L  = L0s(step)/2;
    Lm = L0s(step)/2;
    l  = 0;
    Iw = 0.5*mw*R^2;
    Ip = mp*((L+Lm)^2+0.05^2)/12;
    Im = M*(0.21^2+0.58^2)/12;
    
    Nm = M*(x2 + (L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta)) - l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm = M*g + M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta)) - l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N  = Nm + mp*(x2 + L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P  = Pm + mp*g + mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));
    
    equ1 = x2 - (T - N*R)/(Iw/R + mw*R);
    equ2 = (P*L + Pm*Lm)*sin(theta) - (N*L + Nm*Lm)*cos(theta) - T + Tp - Ip*theta2;
    equ3 = Tp + Nm*l*cos(phi) + Pm*l*sin(phi) - Im*phi2;
    
    [x2,theta2,phi2] = solve(equ1,equ2,equ3,x2,theta2,phi2);
    
    Ja = jacobian([theta1;theta2;x1;x2;phi1;phi2], [theta theta1 x x1 phi phi1]);
    Jb = jacobian([theta1;theta2;x1;x2;phi1;phi2], [T Tp]);
    A  = vpa(subs(Ja, [theta theta1 x x1 phi phi1], [0 0 0 0 0 0]));
    B  = vpa(subs(Jb, [theta theta1 x x1 phi phi1], [0 0 0 0 0 0]));
    % ======================================
    
    % ===== Bryson法则自动整定（新增核心）=====
    % 按物理约束定义最大值（需根据实测数据调整）
    state_limits = [pi/6, 2*pi, 0.1, 1.0, pi/4, 2*pi];  % 更现实的限幅 % [theta,theta1,x,x1,phi,phi1]
    ctrl_limits  = [20, 5];                          % [T, Tp]
    
    Q_bryson = diag(1./state_limits.^2);
    R_bryson = diag(1./ctrl_limits.^2);
    
    Q = RHO_BALANCE * Q_bryson;
    R = R_bryson;
    % ======================================
    
    % 求解并存储
    [G,H] = c2d(double(A), double(B), 0.003);
    Ks(:,:,step) = lqr(G, H, Q, R);  % 离散LQR
    
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