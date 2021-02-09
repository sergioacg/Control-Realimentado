%% Sintonia del Controlador Ziegler y Nichols (Metodo 1)
% Sergio Andres Castaño Giraldo
% https://controlautomaticoeducacion.com
% Rio de Janeiro - 2019
clc
clear all
close all
%Modelo del sistema
G=tf([2],conv([2 1],[5 1]));
G.iodelay=1;
%tiempo
dt=0.01;
t=0:dt:35;


%Entrada
u(1:length(t))=1;
%Salida
y=lsim(G,u,t);
dy=diff(y)/dt; %Derivada
dy2=diff(dy)/dt;

%Encontrar el punto de inflexion y su derivada
% el punto donde la pendiente de la respuesta escalón tiene su valor máximo (punto de inflexión)
[m,p]=max(dy);
yp=y(p);
tp=t(p);
tm=0:20; 
ym=m*(tm-tp)+yp; %Ecuacion de la recta


%Parametros obtenidos por ZN
k=2;
L=2;
tau=9;

G1=tf([k],[tau 1]);
G1.iodelay=L;
y1=lsim(G1,u,t);

%Parametros obtenidos directamente de la grafica
L2=3;
tau2=20/4;
G2=tf([k],[tau2 1]);
G2.iodelay=L2;
y2=lsim(G2,u,t);


%Grafica
figure
plot(t(1:end-1),dy,'linewidth',3);
title('Derivada de la función de transferencia');
axis([0 35 0 0.25]); 
box off
ylabel('$$c(t)$$','FontSize',20,'Interpreter','latex')
xlabel('$$t$$','FontSize',20,'Interpreter','latex')
set(gca,'FontSize',(20) )

figure
plot([t(1) t(end)],[2 2],'--k',[11 11],[0 2],'--k','linewidth',2);
hold on
plot(t,y,tp,yp,'o',tm,ym,'-r','linewidth',3);
axis([0 35 0 3]); 
box off
ylabel('$$c(t)$$','FontSize',20,'Interpreter','latex')
xlabel('$$t$$','FontSize',20,'Interpreter','latex')
set(gca,'FontSize',(20) )

figure
plot([t(1) t(end)],[2 2],'--k','linewidth',2);
hold on
plot(t,y,t,y1,'--',t,y2,'--','linewidth',3);

%text(-1.5,2,'$$K$$','FontSize',20,'interpreter','latex')

axis([0 35 0 3]); 
box off
ylabel('$$c(t)$$','FontSize',20,'Interpreter','latex')
xlabel('$$t$$','FontSize',20,'Interpreter','latex')
set(gca,'FontSize',(20) )

% Grafica de Controladore
alfa=0; %Filtro do PID
Kc=tau/(k*L);
C=Kc;
[c1,tc1]=step((C*G)/(1+C*G));
figure
subplot(3,1,1)
plot([tc1(1) tc1(end)],[1 1],tc1,c1,'linewidth',3)
title('Control P')
set(gca,'FontSize',(20) )

%PI
Kc=(0.9*tau)/(k*L);
ti=3.33*L; td=0;
C=tf(Kc*[ti*td ti 1],[ti 0]);
[c1,tc1]=step((C*G)/(1+C*G));
subplot(3,1,2)
plot([tc1(1) tc1(end)],[1 1],tc1,c1,'linewidth',3)
title('Control PI')
set(gca,'FontSize',(20) )

%PID
Kc=(1.2*tau)/(k*L);
Kc=Kc/1;  
ti=2*L; td=0.5*L;
C=tf(Kc*[ti*td ti 1],[ti 0]);
[c1,tc1]=step((C*G)/(1+C*G));
subplot(3,1,3)
plot([tc1(1) tc1(end)],[1 1],tc1,c1,'linewidth',3)
title('Control PID')
set(gca,'FontSize',(20) )



Kc2=(1.2*tau2)/(k*L2);
Kc2=Kc2/1;  
ti2=2*L2; td2=0.5*L2;
alfa=0; %Filtro do PID
C2=tf(Kc2*[ti2*td2 ti2 1],[alfa*td2*ti2 ti2 0]);

figure
step((C*G)/(1+C*G),(C2*G)/(1+C2*G))
legend('PID por Inflexión','PID por Grafica')