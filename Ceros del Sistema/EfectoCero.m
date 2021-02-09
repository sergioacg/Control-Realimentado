%% Ejemplo de Ceros del Sistema
% Sergio Andres Castaño Giraldo
% Rio de Janeiro 2020
% https://controlautomaticoeducacion.com/
clc
clear all
close all

dt=0.001; %Diferencial
Z=6; %Cero del sistema

t=0:dt:2;
%Para Z=6:
%yt= 0.1714-0.6*exp(-5*t)+0.4286*exp(-7*t);
H=tf(Z,[1    12    35]);
H2=tf([1 Z],[1    12    35]);

yt=step(H,t)';
% hold on
% plot(t,yt,'--r')

%Graficar diagrama de bloques
dyt=diff(yt)/dt; %Derivada;
dyt=[0 dyt];
dyt2=1/Z*dyt;

figure
plot(t,yt,'linewidth',2);
hold on
plot(t,dyt,'--r',t,dyt2,'--k','linewidth',1.5);
legend('y_1','dy/dt',['1/' num2str(Z) ' dy/dt']);

%Sumar yt con dyt2
yt2=yt+dyt2;

%Comparar con la funcion de transferencia H2

figure
step(H2)
hold on
plot(t,yt2,'--r')

%% Frecuencia
%Se debe descargar la función asymp (para eso digitar en google y
%descargarla
figure
asymp(H2)


