%计算前馈

clear;

% F前馈
syms theta
% 机器人结构参数(国际单位制)
g=9.8;
M=13;
feedforward = M*g/cos(theta);
feedforward = feedforward/2;%2条腿的前馈各自独立

% 输出到m函数
matlabFunction(feedforward,'File','D:\Desktop\2026RM\study\串联腿仿真\install\FFdf');
disp("前馈模型分析完毕");