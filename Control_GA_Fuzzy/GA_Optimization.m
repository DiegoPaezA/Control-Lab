% Author: Diego R. Páez
clc;clear;
LB = 0.01*ones(1,4); % Lower bound
UB = 2*ones(1,4); % Upper bound

nvar = 4; %number of variables in the objective functions

opt = optimoptions("ga", "Display","iter", ...
    "MaxGenerations", 25*nvar, ...
    "PopulationSize", 100, ...
    PlotFcn="gaplotbestindiv");

% Optimization

[X, best_obj] = ga(@costs, nvar, [],[], [],[],LB,UB,[],opt);
best_obj;
disp(['The value of Kp is:' num2str(X(1))])
disp(['The value of Ki is:' num2str(X(2))])
disp(['The value of Kd is:' num2str(X(3))])
disp(['The value of Ku is:' num2str(X(4))])
assignin('base','Kp',X(1))
assignin('base','Ki',X(2))
assignin('base','Kd',X(3))
assignin('base','Ku',X(4))
sim('ControlFuzzy_Modelo_GA');
 
function [cost_value] = costs(x)
% Assign Kp, Ki, Kd and Ku
% Kp = x(1);
% Ki = x(2);
% Kd = x(3);
% Ku = x(4);
assignin('base','Kp',x(1))
assignin('base','Ki',x(2))
assignin('base','Kd',x(3))
assignin('base','Ku',x(4))
% Run Model
sim('ControlFuzzy_Modelo_GA');
err=ref-tempModelo; 
[n,~]=size(err);
cost_value=0;
for i=1:n %
    %cost_value=cost_value+(err(i))^2 ; % ISE
    %cost_value=cost_value+abs(err(i)); % IAE
    %
    cost_value=cost_value+time(i)*abs(err(i)); % ITAE
    %cost_value=cost_value+t(i)*(err(i))^2; % ITSE
    %cost_value=cost_value/t(n); % MSE
    %
end
end