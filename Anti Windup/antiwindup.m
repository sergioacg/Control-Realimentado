%% Cálculo del Anti Windup de la acción integradora del PID
% Sergio Andrés Castaño Giraldo
% https://controlautomaticoeducacion.com/
clc
clear 
close all

%% Planta
K = 1;
tau = 10;
G = tf(K,[tau 1]);

%% Control PI
Kp = 1.85;
ti = 2.8;
C=tf(Kp*[ti 1],[ti 0]);

%% loop de control
nit = 2000;
Ts=0.05;
y(1:nit)=0;
u=y;
r=y;
e=y;
Ie=e;
r(200:end)=0.5;
 for k=2:nit
      t = 0:Ts:(k-1)*Ts;
      y=lsim(G,u(1:k),t)';
      
      e(k)=r(k)-y(k);
      Ie(k)=e(k)+e(k-1);
      
      u=lsim(C,e(1:k),t)'; 
      u = [u 0];
      u(find(u>0.8))=0.8;
 end
 H = feedback(C*G,1);
 Hu = feedback(C,G);
 y2=lsim(H,r,t)';
 u2=lsim(Hu,r,t)';
 
 subplot(211)
 plot(t,r,'r',t,y,'k',t,y2,'--b');
 subplot(212)
 plot(t,u(1:end-1),'k',t,Ie,'r',t,u2,'--b');
