%% Implementación controlador DMC
% Dinamic Matrix Control - Modelo CONTROL(LAB)
clc
clear all
close all
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
hp = 20; %Horizonte de predicción (hp)vería 120 segundos al futuro
hc = 5; %Horizonte de control    (hc) 
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
Num_Ite = 200; % Número de iteraciones 200 * st = 1200s de simulación
duf = zeros(1,Nm); % Se inicializa salida diferencial de control - Minimizado de la función de costo
u = zeros(1,Num_Ite); %Se inicializa el vector de accion de control del sistema
ys(1:Num_Ite) = tempA; % Se inicializa la salida actual del sistema a temperatura ambiente

% Referencia
r(1:Num_Ite) = tempA;
r(8:Num_Ite) = 55; 
r(66:Num_Ite) = 70; 
r(133:Num_Ite) = 60; 

%Perturbación
q(1:Num_Ite) = 0;q(50:end) = 0;

w=0; %Termino para colocar referencias futuras 
% %%6.1 Loop de Control
 for j = 10:Num_Ite-w 
%     
% %% 6.2 Salida del proceso con DMC - Ecuación de diferencias transformada Z
     t = 0:st:(j-1)*st;
     ym=lsim(g,u(:,1:j),t,'zoh')';   %Salida de del modelo de la planta    
     ys(j)=ym(j)+tempA + q(j);         %Salida completa del sistema.
% 
     %% 6.3 CALCULO DE LA RESPUESTA LIBRE
    f=zeros(hp,1); % Vetor f (free) respuesta libre
     for kx = 1:hp 
         for i = 1:Nm-hp
            vect_g(i)=gi(kx+i)-gi(i);
         end
         for i=Nm-hp+1:Nm
             vect_g(i)=gi(Nm)-gi(i);
         end
         f(kx)=ys(j)+vect_g*duf';  %Calculo de la respuesta libre
     end
         %f= Vector de respuesta libre con tamaño hp
         %duf= (du libre) es la u correspondiente a la respuesta libre
               %ese vector siempre esta en el pasado. Es cero en el futuro
               %y vale unicamente en el pasado
         %ym= Salida de la planta
% 
%% 6.4 Calculo de la ley de control
% %No se conocen las referencias (w=0)
    inc_u=K1*(r(j+w)*ones(1,hp)'-f);
    
     if j==1
        u(j)=inc_u;
     else
        u(j)=u(j-1)+ inc_u;
     end

    %actualiza duf
    % duf= [du(k-1) du(k-2) ..... du(k-N)]
        aux_2=duf(1:Nm-1);
        duf=[inc_u aux_2];
 end    
%%
t = 0:st:(Num_Ite-1)*st;
figure(2)
subplot(2,1,1)
plot(t(1:Num_Ite-w),r(1:Num_Ite-w),'--k',t(1:Num_Ite-w),ys,'-r','Linewidth',2)
xlabel('Time(s)','FontSize', 18);
ylabel('Temperature (ºC)','FontSize', 18);
legend('Setpoint','Model Output','Location','SouthEast','FontSize', 18)
grid on;
hold
subplot(2,1,2)
stairs(t(1:Num_Ite-w),u,'b','Linewidth',3)
xlabel('Time (s)','FontSize', 18);
ylabel('Heater (0-100%PWM)','FontSize', 18);
legend('u Model','FontSize', 18)
grid on;

%Performance Evaluation
setpoint = r';
yout = ys';
time = t';
[ISE, IAE, ITAE] = performance_eval(setpoint,yout,time);