function qr=ni(bot,T)
t=0:0.009:1;
qz=[0 0 0];
qr=ikine(bot,T);
[q,qd,qdd]=jtraj(qz,qr,t);
m=fkine(bot,q);
for i=1:1:length(t)
    n=m(:,:,i);
    x(1,i)=n(1,4);
    y(1,i)=n(2,4);
    z(1,i)=n(3,4);
end
%»æÖÆÄ©¶Ë¹ì¼£Á¬ÏßÈýÎ¬Í¼
figure()
boxplot3(n(1,4)-0.15,n(2,4)-0.15,n(3,4)-0.15,0.3,0.3,0.3);axis equal
hold on
plot3(x,y,z);xlabel('X');ylabel('Y');zlabel('Z');
grid on;
