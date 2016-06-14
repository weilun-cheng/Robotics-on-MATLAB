clc;
clear;
%根据标准D-H参数建立机械臂link([Alpha A theta d],'standard')
L1 = link([1/2*pi 4 0 0],'standard');
L2 = link([1/2*pi 3 0 0],'standard');
L3 = link([0 2 0 0],'standard');
bot=robot({L1 L2 L3},'bot');
drivebot(bot)
%qz为机械臂的初始关节角
qz=[0 0 0];
%qr为机械臂目标关节角
qr=[1/3*pi 1/3*pi 1/2*pi];
%将时间分切，规划机械臂运动路径
t=0:0.009:1;
[q,qd,qdd]=jtraj(qz,qr,t);  %q为转过的角度，qd为机械臂速度，qdd为加速度
figure()
subplot(3,1,1);plot(q);xlabel('Time');ylabel('q');
subplot(3,1,2);plot(qd);xlabel('Time');ylabel('qd');
subplot(3,1,3);plot(qdd);xlabel('Time');ylabel('qdd');
T=fkine(bot,qr)   %前向运动学求出变换矩阵
qi=ikine(bot,T)   %逆运动学求出每个关节角转的角度
%求出机械臂末端轨迹
m=fkine(bot,q);
for i=1:1:length(t)
    n=m(:,:,i);
    x(1,i)=n(1,4); 
    y(1,i)=n(2,4);
    z(1,i)=n(3,4);
end
%绘制末端轨迹散点图
figure()
scatter3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
%绘制末端轨迹连线三维图
figure()
boxplot3(n(1,4)-0.15,n(2,4)-0.15,n(3,4)-0.15,0.3,0.3,0.3);axis equal;
hold on
plot3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
grid on;
%绘制机械臂运动循环动图
plot(bot,q,'loop');
