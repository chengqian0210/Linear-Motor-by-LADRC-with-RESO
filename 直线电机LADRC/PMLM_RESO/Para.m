%% PMLM ����
M = 5.4;
D = 10;
R = 16.8;
L = 17.4*0.001;
Kf = 130;
Ke = 123;
Fc = 10;
Fs = 20;
Fv = 10;
Ar = 8.5;
a = 314;
phi = 0.05*pi;
Flm = 100;
xs = 0.1;
a1 = (D + Kf*Ke/R)/M;

%% ����ϵ�� 
b0 = Kf/(M*R);

%% RESO����
wo = 1000;
beta1 = 2*wo;
beta2 = wo^2;

%% ����������
wc = 500;
kp = wc^2;
kd = 2*wc;

%%TD΢��������
wt = 50;