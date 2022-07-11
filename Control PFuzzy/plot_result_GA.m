%Plot plant Fuzzy PD+I GA result

clc;clear;

load('simuFuzzy_GA_Modelo.mat')
load('simuFuzzy_1_Planta_GA_1.mat')


tempModelo;              % Model Temperature
tempPlanta = out.tempPlanta; % Plant Temperature
setPoint;   % reference
uPlanta = out.uPlanta;    % Controller action
uModelo;    % Controller action

Ts=1; %sample time 1s
time = 1200; %simulation time
t = (0:Ts:time)';
subplot(2,1,1)
plot(t,setPoint,'--k',t,tempModelo,'-b',t,tempPlanta,'-r','Linewidth',2)
xlabel('Time(s)','FontSize', 18);
ylabel('Temperature (ºC)','FontSize', 18);
legend('Setpoint','Model Output','Plant Output','Location','SouthEast','FontSize', 18)
grid on;
subplot(2,1,2)
plot(t,uModelo,'b',t,uPlanta,'r','Linewidth',3)
xlabel('Time(s)','FontSize', 18);
ylabel('Heater (0-100%PWM)','FontSize', 18);
legend('u Model','u Plant' ,'FontSize', 18)
grid on;

[ISE_M, IAE_M, ITAE_M] = performance_eval(setPoint,tempModelo,t);
[ISE_P, IAE_P, ITAE_P] = performance_eval(setPoint,tempPlanta,t);
