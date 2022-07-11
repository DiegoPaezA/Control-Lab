clc;clear;
load('simuData2.mat')

temp = stepTemp2; % Plant Temperature 
Ts=1; %sample time 1s
time = 600; %simulation time
t = (0:Ts:600)';


g = tf(1.20,[129.2, 1],'InputDelay',14.23); %TF Model
u = (40 + zeros( 1, length(t)))';
tempSimu = lsim(g,u,t);

tempSimu = tempSimu + 27;
plot(t,temp,t,tempSimu,'LineWidth',2);
% 
ylabel('T(C)')
xlabel('t(s)')
legend({'Planta','Modelo'},'FontSize', 14)
grid
