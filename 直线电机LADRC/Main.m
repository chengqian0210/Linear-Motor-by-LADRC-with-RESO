% -------------------------------------------------------------------------
% 物理模型:     Active Disturbance Rejection Control for Permanent Magnet Linear Motor的模型
% 算法:         常规线性ADRC     
% -------------------------------------------------------------------------
clear all;
close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h=0.001;  %Sampling time
%系统参数
M = 5.4;D = 10;R = 16.8;L = 17.4*0.001;Kf = 130;Ke = 123;Fc = 10;
Fs = 20;Fv = 10;Ar = 8.5;a = 314;phi = 0.05*pi;Flm = 100;s = 0.1;


%ESO Parameters
w = 600;
b0 = Kf/(M*R);
beta1=3*w;beta2=3*w^2;beta3=w^3;
%NPID Parameters
wc = 50;
kp=wc^2;kd=2*wc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xk=zeros(2,1);
u_1=0;
z1_1=0;z2_1=0;z3_1=0;
for k=1:1:8000
time(k) = k*h;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%参考信号
v1(k)=100*sin(time(k)); %位置参考信号
v2(k)=100*cos(time(k));%参考微分信号

%%%%%%%%%%%%%%%%%%%%%%%
%负载Fload
if mod(k,3000)<2500
    Fload(k) = 50;
else
    Fload(k) = 100;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%被控对象
p1=u_1;
p2=k*h;
tSpan=[0 h];
[tt,xx]=ode45('plant',tSpan,xk,[],p1,p2);%解被控对象微分方程   chap6_12plant对应公式（6）
xk = xx(length(xx),:);
y(k)=xk(1);
dy(k)=xk(2);

%%%%%%%%%%%%%%%%%%%%%%
%求扰动
Fripple(k) = Ar*sin(a*y(k)+phi);%公式（4）
Ffric(k) = (Fc + (Fs - Fc)*exp(-(dy(k)/s)^2) + Fv*abs(dy(k)))*sign(dy(k));
Fd = Fload(k) + Fripple(k) + Ffric(k);%(2)
fd = Fd/M;
a1 = (D + Kf*Ke/R)/M;
f(k)= - a1*dy(k) - fd;%扰动 公式（8）
x3(k)=f(k);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ESO
e=z1_1-y(k);
z1(k)=z1_1+h*(z2_1-beta1*e);
z2(k)=z2_1+h*(z3_1-beta2*e+b0*u_1);
z3(k)=z3_1-h*beta3*e;

z1_1=z1(k);
z2_1=z2(k);
z3_1=z3(k);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%SEF and disturbance compensation
e1(k)=v1(k)-z1(k);
e2(k)=v2(k)-z2(k);

    u0(k)=kp*e1(k)+kd*e2(k);
    u(k)=u0(k)-z3(k)/b0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z1_1=z1(k);z2_1=z2(k);z3_1=z3(k);
u_1=u(k);
end

    figure(1);
    plot(time,Fload,'k','linewidth',2);
    xlabel('time(s)'),ylabel('F signal');
    legend('Fload signal');
    title('负载Fload');
    figure(2);
    plot(time,v1,'r',time,y,'k:','linewidth',2);
    xlabel('time(s)'),ylabel('position signal');
    legend('ideal position signal','position tracking signal');
    title('位置跟踪');
    figure(3);
    plot(time,v1-y,'b-.','linewidth',2);
    xlabel('time(s)'),ylabel('position tracking error');
    title('位置跟踪误差');
    figure(4);
    subplot(311);
    plot(time,y,'r',time,z1,'k:','linewidth',2);
    xlabel('time(s)'),ylabel('z1,y');
    legend('practical position signal', 'position signal estimation');    
    subplot(312);
    plot(time,dy,'r',time,z2,'k:','linewidth',2);
    xlabel('time(s)'),ylabel('z2,dy');
    legend('practical speed signal', 'speed signal estimation');    
    subplot(313);
    plot(time,x3,'r',time,z3,'k:','linewidth',2);
    xlabel('time(s)'),ylabel('z3,x3');
    legend('practical uncertain part', 'uncertain part estimation'); 
    title('ESO跟踪');
    figure(5);
    subplot(311);
    plot(time,y-z1);
    xlabel('time(s)'),ylabel('e1 '); 
    subplot(312);
    plot(time,dy-z2);
    xlabel('time(s)'),ylabel('e2');
    subplot(313);
    plot(time,x3-z3);
    xlabel('time(s)'),ylabel('e3');
    title('ESO跟踪误差');