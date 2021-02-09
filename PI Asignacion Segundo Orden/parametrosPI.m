function y = parametrosPI(x,FT,beta)

n1=FT.num{1};
d1=FT.den{1};

P=tf(n1/d1(1),d1/d1(1)); %Normaliza la FT
k=dcgain(P);%Ganho Estatico
n=k*d1(3); %Numerador
d=P.den{1};%denominador

%Definición de las variables:

kc=x(1);
ki=x(2);
ep=x(3);
wn=x(4); %Polo insignificante

%Ecuaciones
f1=12*ep*wn/d(2);
f2=(((2*beta*ep^2+1)*wn^2-d(3)))/(kc*k);
f3=10*ep*wn^3/(k*ki);
f4=10*ep*wn*kc/ki;


y=[f1 f2 f3 f4];