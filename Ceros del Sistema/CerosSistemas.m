%% Ejemplo de Ceros del Sistema
% Sergio Andres Castaño Giraldo
% Rio de Janeiro 2020
% https://controlautomaticoeducacion.com/
clc
clear all
close all
%Frecuencia
w0=1;
%Sistema con w0
H=tf([1 0 w0^2],w0*[1    12    35]);

%entrada (tiempo)
t=0:0.001:10;
ut=sin(w0*t);


%Respuesta temporal
figure
subplot(2,1,1)
yt=lsim(H,ut,t);
subplot(2,1,1)
plot(t,ut)
ylabel('Entrada u')
subplot(2,1,2)
plot(t,yt)
ylabel('Salida y')

%Diagrama frecuencial (diagrama de bode)
figure
bode(H)