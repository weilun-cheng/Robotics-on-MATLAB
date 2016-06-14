%theta d a alpha
clear;
clc;
L(1) = Link([ 0 0 4 0],'standard');
L(2) = Link([ 0 0 3 pi/2],'standard');
L(3) = Link([ 0 0 2 0],'standard');
bot = SerialLink(L, 'name', 'bot');
bot.display();
qz = ([0 0 0]);
qr = ([pi/2 pi/4 -pi/2]);
t = 0:0.056:2;
[p,pd,pdd] = jtraj(qz,qr,t);
plot(bot,p);
bot.teach();
figure(2);
subplot(3,1,1);plot(p);xlabel('Time');ylabel('p');
subplot(3,1,2);plot(pd);xlabel('Time');ylabel('pd');
subplot(3,1,3);plot(pdd);xlabel('Time');ylabel('pdd');
