%% Sintonia del Controlador Ziegler y Nichols (Metodo 2)
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com
% Rio de Janeiro - 2019

close all
clear all
clc
%Funcion de transferencia (Válvula - Planta - Sensor)
G=tf(1,[1/6 1 11/6 1]);

%Animación del incremento de la acción proporcional del controlador

ts=0.1;
nit=10/ts;
t = 0:ts:(nit-1)*ts;
R=ones(1,length(t)); %Escalon unitario
set(figure,'Color', [1 1 1] );
for k=0:2:10
    %Lazo Cerrado
    Gc=feedback(k*G,1);
    y=lsim(Gc,R,t);
    for i=1:length(t)
        plot(t(1:i),y(1:i),'-r','Linewidth',4)
        xlabel('Time','interpreter','latex','FontSize',22);
        ylabel('Output','interpreter','latex','FontSize',22);
        title(strcat("K=", num2str(k)),'FontSize',22);
        set(gca,'XTick', [] ,'YTick', [] )
        box off
        axis([0 t(end) 0 max(y)+0.1])
        hold on
        pause(1e-5);
    end
    hold off
end


figure
[C1,Kc1,ti1,td1,Ku1,Tu1] = PID_Ku(G,'P',1,1); %Planta, control, tipo, graficar
title('Control P - Ganancia Limite')

figure
[C2,Kc2,ti2,td2,Ku2,Tu2] = PID_Ku(G,'PI',1,1); %Planta, control, tipo, graficar
title('Control PI - Ganancia Limite')

figure
[C3,Kc3,ti3,td3,Ku3,Tu3] = PID_Ku(G,'PID',1,1); %Planta, control, tipo, graficar
title('Control PID - Ganancia Limite')