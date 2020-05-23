
function dy = PlantModel(t,y,flag,p1,p2)
M = 5.4;D = 10;R = 16.8;L = 17.4*0.001;Kf = 130;Ke = 123;Fc = 10;
Fs = 20;Fv = 10;Ar = 8.5;a = 314;phi = 0.05*pi;Flm = 100;s = 0.1;
b0 = Kf/(M*R);

ut=p1;
% time=p2;
dy=zeros(2,1);

Fload = 50;
Fripple = Ar*sin(a*y(1)+phi);%¹«Ê½£¨4£©
Ffric = (Fc + (Fs - Fc)*exp(-(y(2)/s)^2) + Fv*abs(y(2)))*sign(y(2));
Fd = Fload + Fripple + Ffric;%(2)
fd = Fd/M;
a1 = (D + Kf*Ke/R)/M;
f=- a1*y(2) - fd;   %Uunknown part
dy(1)=y(2);
dy(2)=f+b0*ut;