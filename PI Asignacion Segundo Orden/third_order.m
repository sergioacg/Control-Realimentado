clc
clear all
close all

%función de transferencia
num = 62.5*[1 2.5];
den = conv([1 6 25],[1 6.25]);
G=tf(num,den);

%Aprox función de transferencia
Wn=5;
ep=0.39;
num1 = Wn^2;
den1 = [1 2*ep*Wn Wn^2];
G1=tf(num1,den1);

step(G,G1)