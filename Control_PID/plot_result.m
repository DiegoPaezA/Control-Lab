clc;clear;
load('simuModelo.mat')
load('simuPlanta.mat')

tempModelo; % Model Temperature
tempPlanta; % Plant Temperature
setPoint;   % reference
uModelo;    % Controller action
uPlanta;    % Controller action

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