%% Cálculo del Anti Windup de la acción integradora del PID
% Sergio Andrés Castaño Giraldo
% https://controlautomaticoeducacion.com/
clc
clear 
% close all

st = {'PI sin limites','PI sin anti windup',...
    'PI con anti windup', 'PI incremental anti windup'};
m = menu('Anti Windup',st);
%% Planta
K = 0.8;
tau = 10;
G = tf(K,[tau 1]);

%% Control PI
Kp = 1.85;
ti = 2.8;
td=0;
tw=1;
Ts=0.05;
C=tf(Kp*[ti 1],[ti 0]);

% Calculo do controle PID digital
q0=Kp*(1+Ts/(2*ti)+td/Ts);
q1=-Kp*(1-Ts/(2*ti)+(2*td)/Ts);
q2=(Kp*td)/Ts;

%% Limites máximos y minimos
umax = 0.8;
umin = 0.0;

%% loop de control
nit = 100/Ts;
y(1:nit)=0;
u=y;
ug=u;
deltaU=u;
r=y;
e=y;
Ie=e;
r(10/Ts:end)=0.5;
 for k=3:nit
     %Respuesta del proceso
      t = 0:Ts:(k-1)*Ts;
      y=lsim(G,ug(1:k),t)';
      
      %Error
      e(k)=r(k)-y(k);
      
      %Parte Integral del Controlador
      Ie(k) = u(k-1) + q1*e(k-1);
      
      %PID
     u(k) = u(k-1) + q0*e(k) + q1*e(k-1) + q2*e(k-2); 
     %Anti-Windup
     if m>2
         if (u(k) >= umax)     
            u(k) = umax;
         elseif (u(k) <= umin)
             u(k) = umin;
         end
     end
     
     %PID ley de control incremental
     if m==4
         deltaU(k) =  q0*e(k) + q1*e(k-1) + q2*e(k-2);
         if (u(k-1)+deltaU(k) >= umax || u(k-1)+deltaU(k) <= umin)     
            deltaU(k) = 0;
         end
         u(k) = u(k-1)+deltaU(k);
     end
    
     ug=u;
     
     %Saturación del elemento final de control
     if m>1
        ug(ug>umax)=umax;
     end
 end
 H = feedback(C*G,1);
 Hu = feedback(C,G);
 y2=lsim(H,r,t)';
 u2=lsim(Hu,r,t)';
 
 figure
 subplot(311)
 plot(t,r,'r',t,y,'k',t,y2,'--b');grid,
 title(st{m})
 ylabel('Salida')
 subplot(312)
 plot(t,ug,'k',t,u2,'--b');grid,
 ylabel('Control')
 subplot(313)
 plot(t,Ie,'r');grid,
 ylabel('Integral del error')
