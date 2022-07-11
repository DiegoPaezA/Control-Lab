%Plot plant dmc result

clc;clear;

load('simuDMC_plant.mat')


tempPlanta = tempDMC; % Plant Temperature
setPoint = setpoint;   % reference
uPlanta = uDMC;    % Controller action

Ts=1; %sample time 1s
time = 1200; %simulation time
t = (0:Ts:time-1)';
subplot(2,1,1)
plot(t,setPoint,'--k',t,tempPlanta,'-r','Linewidth',2)
xlabel('Time(s)','FontSize', 18);
ylabel('Temperature (ºC)','FontSize', 18);
legend('Setpoint','Plant Output','Location','SouthEast','FontSize', 18)
grid on;
subplot(2,1,2)
plot(t,uPlanta,'b','Linewidth',3)
xlabel('Time(s)','FontSize', 18);
ylabel('Heater (0-100%PWM)','FontSize', 18);
legend('u Plant' ,'FontSize', 18)
grid on;

[ISE, IAE, ITAE] = performance_eval(setPoint,tempPlanta,t);