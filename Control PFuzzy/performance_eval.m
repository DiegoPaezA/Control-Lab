function [ISE,IAE, ITAE] = performance_eval(setpoint,yout,time)
% the time-integral performance criteria, based on the entire response of 
% the process, was used to evaluate the error behavior. 
%   The integral of the square error (ISE), 
%   Integral of the absolute value of the error (IAE), 
%   Integral of the time-weighted absolute error (ITAE) 
% expressed by (Sthephanopoulos, 1985).
% Inputs: setpoint, plant output, time vector   
err=setpoint-yout; 
[n,~]=size(err);
ISE=0;
IAE=0;
ITAE=0;
for i=1:n %
    ISE=ISE+(err(i))^2 ; % ISE
    IAE=IAE+abs(err(i)); % IAE
    ITAE=ITAE+time(i)*abs(err(i)); % ITAE
end
end
