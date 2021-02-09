% Predictor de Smith
% Sergio Andres Castaño Giraldo
% http:\\controlautomaticoeducacion.com
%_________________________________________________________________________

clc
clear all
close all

Ts=0.4;             %Tiempo de Muestreo
FS=18;
%% Proceso Real P(s)
nP=0.12;                %Numerador de la Planta
dP=[6 1];               %Denominador de la Planta
P=tf(nP,dP);            %Processo real P(s)
L=3.0;                  %Retardo de P(s)
P.iodelay=L;            %Aplico el retardo en P(s)
Pd=c2d(P,Ts);           %Discretizo el Proceso con retendedor de Orden Cero P(z)
[B,A]=tfdata(Pd,'v');   %Divido el proceso discreto en numerador y denominador
d=Pd.iodelay;          %Obtengo el retardo en discreto

%% Modelo Nominal Pn(s)
nPn=0.12;                %Numerador del Proceso Nominal
dPn=[6 1];               %Denominador del Proceso Nominal
Gn=tf(nP,dP);            %Modelo Nominal sin retardo Gn(s)
Ln=3.0;                  %Retardo de Pn(s)
Pn=Gn;
Pn.iodelay=Ln;           %Aplico el retardo en P(s)
Pdn=c2d(Pn,Ts);           %Discretizo el Proceso nominal con retendedor de Orden Cero Pn(z)
Gnd=Pdn;
Gnd.iodelay=0;          %Modelo Nominal sin retardo Gn(z)
[Bgn,Agn]=tfdata(Gnd,'v');  %Divido el modelo discreto en numerador y denominador
[Bn,An]=tfdata(Pd,'v');  %Divido el proceso discreto en numerador y denominador
dn=Pdn.iodelay;           %Obtengo el retardo en discreto

%% Perturbacion
nQ=0.3;                %Numerador del Proceso Nominal
dQ=[100 1];               %Denominador del Proceso Nominal
Gq=tf(nQ,dQ);            %Modelo Nominal sin retardo Gn(s)
Pq=c2d(Gq,Ts); 

%% Controlador Primario por cancelamiento de polos
Ti=dPn(1);      %Selecciono el parametro Ti=Tn (Parametro Integral)
Kn=nPn;         %Ganancia de la planta
Tr=dPn(1)*0.75; %Constante de tiempo que deseo en lazo cerrado (75% del tao de la planta)
Kc=Ti/(Kn*Tr);  %Parametro proporcional del controlador PI
C=tf(Kc*[Ti 1],[Ti 0]); %Controlador PI [C(s)]
Cd=c2d(C,Ts,'tustin');  %Discretizo el controlador
[Bc,Ac]=tfdata(Cd,'v');   %Divido el controlador discreto en numerador y denominador
% 
% %% Lazo Cerrado
% 
% Hz=minreal((Cd*Gnd)/(1+Cd*Gnd));
% 
% [nH,dH]=tfdata(Hz,'v');
% 
% %% Calculo do Filtro
% 
% radio=0.85;
% alfa=0.85;
% nk=3;
% 
% polos=pole(Pq);                  % Obtem os polos de Pq
% p_ind=polos(find(abs(polos)>=radio))';  % Obtem os polos indesejados de G(z)
% nm=length(p_ind);
% 
% lista_p=[1 p_ind];        % lista de polos indesejados
% 
% px=poly(lista_p);
% 
% %Denominador do filtro
% Dr=1;
% if length(alfa)==1
%     for i=1:nk
%         Dr=conv(Dr,[1 -alfa]);
%     end
% else
%     Dr=poly(alfa);
% end
% %Polinomio formado só pelo atraso
% Pold(d+1)=1;
% Pold=flip(Pold); %Polinomio  d(atraso)
% 
% Pes=conv(Dr,Pold);
% ordem=length(Pes)-1; %Maximo orden do termo da esquerda (Dr z^d)
% 
% pn=1; %posicion del polo en px
% lpx=length(px); %Longitud de los polos px
% lnH=length(nH); %Longitud de nH
% lDr=length(Dr)-1; %Orden del Polinomio Dr
% ldH=length(dH)-1; %Orden del Polinomio dH (Denominador de lazo cerrado)
% 
%   %Vou ter uma incognita a mais do ordem
% A=zeros(ordem+1,ordem+lDr-ldH); %Crea la matriz com os coeficientes das incognitas
% ip=1; %Utilizada para aumentar de a uma fila por coluna na matriz A (Sylvestre)
% for j=lDr-ldH+2:ordem+lDr-ldH  %Começo na coluna d+1
% 
%     for i=ip:ordem+1 %Começo colocando os valores dos polos que quero eliminar
%         if pn<=lpx   %de forma de uma matriz de silvestre
%             A(i,j)=px(pn);
%             pn=pn+1;
%         end
%     end
%     pn=1; %Reinicia o indice que quenta na matriz dos polos que quero elimnar
%     ip=ip+1; %Aumento de uma coluna cada que coloco o total de polos
% 
% end
% 
% %Encho as primeiras colunas da matriz, que contem os coeficientes do
% %numerador(Nr) do filtro (Dr z^d + Nr)
% i=d+1;
% for j=1:lDr-ldH+1
%        A(i:i+lnH-1,j)=nH'; 
%        i=i+1;
% end
% 
% %Encho Matriz B Com os polos desejados do filtro
% B=zeros(ordem+1,1);
% B(1)=1;
% B(2:length(Dr),1)=Dr(1,2:length(Dr));
% 
% X=A\B; %Solucion del sistema de ecuaciones
% 
% %Numerador do filtro
% Nr=[];
% for i=1:lDr-ldH+1
%     Nr=[Nr X(i)];
% end
% Nr=conv(dH,Nr);
% sum(Nr)/sum(Dr)
% Fr=tf(Nr,Dr,Ts);

% 
%% Filtro de Robustez
Tr=4.5;    %Dinamica en lazo Cerrado
Tf=3;       %Dinamica del Filtro
beta=(1-(1-Tf/Ti)^2*exp(-Ln/Ti))*Ti;
Frs=tf(conv([Tr 1],[beta 1]),conv([Tf 1],[Tf 1]));
Fr=c2d(Frs,Ts,'tustin');
%Fr=filtro_siso(Pdn,0.5,0.91,1); %Calculo del filtro de robustez del PSF
[Bf,Af]=tfdata(Fr,'v');   %Divido el filtro discreto en numerador y denominador

%% Simulacion
nit=600;        %Numero de Interacciones
ref(1:nit)=0;   %Vector de SetPoint
ref(27:end)=1;  %Escalon unitario en el Set Point
rq(1:nit)=0;
rq(220:nit)=10;   %Estimulo de perturbacion de carga
rn(1:nit)=0;
rn(450:nit)=0.3; %Estimulo da perturbacion de salida

%Variables de simulacion
y1(1:nit) = 0; y(1:nit) = 0; u(1:nit) = 0;e(1:nit) = 0;yp(1:nit) = 0;
yt(1:nit) = 0; yp(1:nit) = 0; ep(1:nit) = 0;yn(1:nit) = 0; up(1:nit) = 0;
yf(1:nit) = 0; %Salida Filtrada
%Loop de Control
for i=1:2
for k=d+5:nit
    
    %Salida del proceso real P(z) [y(t)]
    y1(k)= B(1)*up(k-1-d)+B(2)*up(k-2-d)-A(2)*y1(k-1);
    y(k)=y1(k)+rn(k);
    
    %Salida del proceso nominal Pn(z)
    yn(k)= Bn(1)*u(k-1-dn)+Bn(2)*u(k-2-dn)-An(2)*yn(k-1);
    
    %Diferencia entre proceso real y proceso nominal (error de prediccion)
    ep(k)=y(k)-yn(k);
    
    if i==2 %Si es igual a 2 utilice el filtro del PSF
       yf(k)= Bf(1)*ep(k-1)+Bf(2)*ep(k-2)+Bf(3)*ep(k-3)-Af(2)*yf(k-1)-Af(3)*yf(k-2);
    end
    
    %Salida del modelo rapido Gn(z)  [y(k+d)]
    yt(k)= Bgn(1)*u(k-1)+Bgn(2)*u(k-2)-Agn(2)*yt(k-1);
    
    %Suma entre error de prediccion y salida predicha
    if i==1
        yp(k)=yt(k)+ep(k);
    else
        yp(k)=yt(k)+yf(k);
    end
    
    %Error de la ley de control
    e(k)=ref(k)-yp(k);
    
    %ley de control
    u(k)=u(k-1)+ Bc(1)*e(k)+Bc(2)*e(k-1);  
    
    %Ingreso de perturbacion en la entrada
    up(k)=u(k)+rq(k);
end
if i==1
    ySP=y;
    uSP=u;
else
    ySPF=y;
    uSPF=u;
end
end

%Grafico
t = 0:Ts:(nit-1)*Ts;
figure(1)
subplot(3,1,1);
hold on;
plot(t,ref,t,ySP,t,ySPF,'Linewidth',3)
legend('Referencia','PS','PSF');
xlabel('Tiempo (s)','FontSize',FS);
ylabel('Salida','FontSize',FS);
set(gca,'fontsize',FS);
grid on;
hold on
subplot(3,1,2);
hold on;
plot(t,uSP,t,uSPF,'Linewidth',3)
xlabel('Tiempo (s)','FontSize',FS);
ylabel('Control','FontSize',FS);
set(gca,'fontsize',FS);
grid on;
subplot(3,1,3);
hold on;
plot(t,rq,t,rn,'Linewidth',3)
legend('Perturbacion de carga','perturbacion de salida');
xlabel('Tiempo (s)','FontSize',FS);
ylabel('Salida','FontSize',FS);
set(gca,'fontsize',FS);
grid on;
hold on