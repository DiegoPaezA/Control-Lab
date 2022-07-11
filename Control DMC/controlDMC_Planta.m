%% Implementación controlador DMC
% Dinamic Matrix Control - Planta CONTROL(LAB)
clc
clear all
%close all
%% 1. Función de transferencia de la planta: Modelo del Sistema
st = 6; %Tiempo de muestreo relacionado con horizonte de predicción st = tao/hp
g = tf(1.20,[129.2, 1],'InputDelay',14.23); %TF Model

tempA = 27; %Temperatura ambiente

gz = c2d(g,st); % Funcion de transferencia discreta (FTD)
[num,den] = tfdata(gz,'v');
b = num; % Númerador FTD
a = [den 0]; % Denominador FTD
d=gz.iodelay;   % atraso de tiempo discreto
%% 2. Parametros del control DMC
gpi=stepinfo(g); %Parametros de la planta
hp = 20; %Horizonte de predicción (P)vería 180 segundos al futuro
hc = 5; %Horizonte de control    (N)
lambda = 0.1; %Factor de Supresión del control
delta = 0.1*eye(hp); %Factor de ponderación del error

%% 3. Vector de coeficientes Gi - respuesta al escalon
gi=step(gz);           %Respuesta al escalon
Nm=length(gi)-1;       %Valor limite para la predicción del modelo completo "M"

%% 4. Calculo G salida de predicción Y=Gu+f
G=zeros(hp,hc); %Matriz G
G(:,1)=gi(1+d:hp+d);  % Se llena la primera columna con los valores de respuesta al escalon
% Hasta el horizonte de prediccion, quitando los
% retardos del sistema.
% Se llena la matriz G con los coeficientes resultantes de la evaluación de j e i.
for i=2:hc
    for j=2:hp
        G(j,i)=G(j-1,i-1);
    end
end
%% 5. Calcula matriz k de función de costo minimizada se toma la primera fila
Mn=inv(G'*delta*G + lambda*eye(hc))*G'*delta;
%Calculo de k de la función de costo minimizada. Se calcula una sola vez.
K1=Mn(1,:);
%% 6. Lazo de Control
T = 1;
Num_Ite = 1200; % Número de iteraciones st = 1200s de simulación

inc_u=0;
u_ant(1:10) = 0;
u(1:Num_Ite) = 0; %Se inicializa el vector de accion de control del sistema
h1s(1:Num_Ite) = 0; %Heater 1



% Referencia
r(1:Num_Ite) = tempA;
r(50:Num_Ite) = 55; 
r(400:Num_Ite) = 70; 
r(800:Num_Ite) = 60; 

duf = zeros(1,Nm); % Se inicializa salida diferencial de control - Minimizado de la función de costo
w=0; %Termino para colocar referencias futuras

%% include tclab.m for initialization
tclab;
disp('Load tclab library')
figure(1)
t1s = [];
h1s = [];
% Valores iniciales de los heaters
ht1 = 0;
%ht2 = 0;
h1(ht1);
%h2(ht2);
%% %6.1 Loop de Control
for j = 1:Num_Ite-w
    %
    % %% 6.2 Salida del proceso de la planta CONTROL(LAB)
    tic
    tic
    h1(ht1);
    % read temperatures
    t1 = T1C();
    % LED brightness
    brightness1 = (t1 - 30)/50.0;  % <30degC off, >100degC full brightness
    led(brightness1);
    
    % plot heater and temperature data
    h1s = [h1s,ht1]; %Almace registro historico.
    t1s = [t1s,t1];  %Historico Temperatura
    
    n = length(t1s);
    %     time = linspace(0,T*(n+1),n);
    time = 0:T:T*(n-1);
    if mod(j,st)==0 %Se calcula acción de control cada st segundos
        %% 6.3 CALCULO DE LA RESPUESTA LIBRE
        f=zeros(hp,1); % Vetor f (free) Resposta libre
        for kk=1:hp
            % monta un vector con las gkk+i - gkk
            for i=1:Nm-hp
                vect_g(i)=gi(kk+i)-gi(i);
            end
            for i=Nm-hp+1:Nm
                vect_g(i)=gi(Nm)-gi(i);
            end
            f(kk)=t1s(j)+vect_g*duf';  %Calculo da resposta livre
            %f= Vector de respuesta libre con tamaño P
            %duf= (du libre) es la u correspondiente a la respuesta libre
            %ese vector siempre esta en el pasado. Es cero en el futuro
            %y vale unicamente en el pasado
            %ym= Salida de la planta
        end
        %Calculo del Controle
        %Projeto onde nao tenho as referencias futuras
        inc_u=K1*(r(j+w)*ones(1,hp)'-f);
    else
        inc_u=0;  %Si no se cumple st, no hay nuevo incremento de control
    end
    %% 6.4 Calculo de la ley de control
    % %No se conocen las referencias (w=0)
    if j==1
        u(j)=inc_u;
    else
        u(j)=u(j-1)+ inc_u;
    end
    %Restricciones
    if u(j)>100
        u(j)=100;
    end
    if u(j)<0
        u(j)=0;
    end
    ht1 = u(j); %actualiza variable de la acción de control
    
    %actualiza duf
    % duf= [du(j-1) du(j-2) ..... du(j-N)]
    if mod(j,st)==0
        aux_2=duf(1:Nm-1);
        duf=[inc_u aux_2];
    end
    
    clf
    subplot(2,1,1)
    plot(time,t1s,'r',time,r(1:j),'b','LineWidth',2);
    hold on
    
    ylabel('Temperature (degC)')
    legend('Temperature 1','Setpoint','Location','NorthWest')
    subplot(2,1,2)
    stairs(time,h1s,'r-','LineWidth',2);
    hold on
    ylabel('Heater (0-100%PWM)')
    xlabel('Time (sec)')
    legend('Heater 1','Location','NorthWest')
    drawnow;
    
    t = toc;
    pause(T-t)
    toc
end
%%
disp('Turn off heaters')
h1(0);
led(0); % OFF
disp('Heater Test Complete')