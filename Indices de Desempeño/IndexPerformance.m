% Indice Performance
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com/
clc
clear 
close all

%control
Kc = 1;
ti = 1;
td = 0;
% C=tf(Kc*[ti*td ti 1],[td*ti ti 0]);
G = tf(1,[1 0]);
zeta = 0.1:0.01:2;
% zeta = 0.2;
t=0:0.001:15;
for i = 1:length(zeta)
    %Planta
    den = [1 2*zeta(i)];
    C = tf(1,den);
    E = feedback(1,G*C); %Error
    e = step(E,t);
    Ts = t(2);
   
    %% Opción 1 (manualmente)
%     e_1=[1;e(1:end-1)];% Error-1
%     TE =Ts.*((e+e_1)./2); %Trapecio del error
%     
%     sqe = e.^2; %Error cuadratico
%     e2_1=[0;sqe(1:end-1)]; %Error cuadratico -1     
%     TE2 = Ts.*((sqe+e2_1)./2); %Trapecio del error cuadratico
%     
%     ISE(i) = sum(TE2);
%     ITSE(i) = sum(TE2.*t');
%     IAE(i) = sum(abs(TE));
%     ITAE(i) = sum(abs(TE).*t');

    %% Opción 2 (función trapz)
    ISE(i) =  trapz(t,e.^2);
    ITSE(i) = trapz(t,t'.*e.^2);
    IAE(i) =  trapz(t,abs(e));
    ITAE(i) = trapz(t,t'.*abs(e));
end

plot(zeta,ISE,zeta,ITSE,zeta,IAE,zeta,ITAE,'linewidth',2),grid
axis([0 2 0 7])
xlabel('\zeta')
ylabel('J')
legend('ISE','ITSE','IAE','ITAE')