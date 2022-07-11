clc
clear
close
%% Now tune the FLC controller I/O scale factor disp('Tuning the FLC')
LB = 0.01*ones(1,4); % Lower bound
UB = 2*ones(1,4); % Upper bound
X_min = LB; X_max = UB;
[X,best_obj]=GA(X_min,X_max,@costs);
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

function [X_best , obj_optimal] = GA(X_min,X_max,CostFun)
Para_No=size(X_min,2);
costs = CostFun; % Cost function
%Algorithm Parameters
Ngmax=100; %Maximum number of generations
Tourn_Prob_f =0.8;%Tournament propability factor
Cro_Prob=0.7;%Crossover propability
alpha=0.5; %Crossover coefficient 
Mut_Prob=0.01;
b=2; %Mutation propability
N=200; %Mutation constant %Number of solutions (Population size) %
% t=1;
Nps = N/4 ; %Generation counter % Number of selected population

%*****Initialize population **** %
disp('->Initialize population')
old_pop=zeros(N,Para_No);
i=1;
h = waitbar(0,'Initializing population...');

while (i<=N)
    waitbar(i/N,h)
    for j=1:Para_No
        old_pop(i,j) = X_min(j) + (X_max(j)-X_min(j))*rand;
    end
    i=i+1;
end
delete(h);
%***** Objective function evaluation ***** %
disp('->Calculating the cost function of the initial population')
h = waitbar(0,'Cost function evaluation of the initial population...');
fitness = zeros(N,1);
for i =1:size(old_pop,1)
    waitbar(i/size(old_pop,1),h)
    fitness(i) = costs(old_pop(i,:));
end
delete(h);
Jbest = min(fitness);
Best_index = find(Jbest==fitness);
Xbest = old_pop(Best_index(1),:);

%%%%%%%%%%%%%%%% The below section is repeated for differentgenerations %%%%%%%%%%%%%%%%%%%%%%%
obj_opt = zeros(Ngmax,1);
best_sol_gen = zeros(Ngmax,Para_No);

h = waitbar(0,'Please wait...');
for t=1:Ngmax
    t
    waitbar(t/Ngmax,h)
    %Tournament selection
    selected_parent_index = Tourn_Select(N,fitness,Nps,Tourn_Prob_f);

    %**** BLX-a Crossover ********%
    child1= zeros(Nps,Para_No);
    child2= zeros(Nps,Para_No);
    i=1;
    while i<=Nps
        for j =1:Para_No
            p1=old_pop(selected_parent_index(i,1),j);
            p2=old_pop(selected_parent_index(i,2),j);
            if rand <= Cro_Prob
                cmax=max(p1,p2);
                cmin=min(p1,p2);
                I=cmax-cmin;
                child1(i,j)=(cmin-I*alpha)+(rand*((cmax+I*alpha)-(cmin-I*alpha)));
                child2(i,j)=(cmin-I*alpha)+(rand*((cmax+I*alpha)-(cmin-I*alpha)));
            else
                child1(i,j)=p1;
                child2(i,j)=p2;
            end

            if (child1(i,j)>X_max(j))
                child1(i,j)=X_max(j);
            end
            if (child2(i,j)<X_min(j))
                child2(i,j)=X_min(j);
            end
        end

        i = i+1;
    end
    children=[child1 ;child2];

    %Non-Uniform Mutation
    para_min=X_min; para_max=X_max;
    childrenm = zeros(N/2,Para_No);
    i = 1;
    while (i<=N/2)
        for j=1:Para_No
            if rand <= Mut_Prob
                deltap=(para_max(j)-children(i,j))*(1-rand^((1-(t-1/Ngmax))^b));
                deltan=(children(i,j)-para_min(j))*(1-rand^((1-(t-1/Ngmax))^b));
                if rand < 0.5
                    childrenm(i,j)=children(i,j)+deltap;
                else
                    childrenm(i,j)=children(i,j)-deltan;
                end
            else
                childrenm(i,j)=children(i,j);
            end
            if (childrenm(i,j)>para_max(j))
                childrenm(i,j)=para_max(j);
            end
            if (childrenm(i,j)<para_min(j))
                childrenm(i,j)=para_min(j);
            end
        end
        i = i+1;
    end

    %combining parents with new children solutions to form the new generation
    parents=zeros(N/2,Para_No);
    for i=1:N/2
        parents(i,:)=old_pop(selected_parent_index(i),:);
    end
    new_pop=[parents;childrenm];
    % Objective function evaluation
    for i =1:size(new_pop,1)
        fitness(i) = costs(new_pop(i,:));
    end
    Jmin = min(fitness);
    Min_index = find(Jmin==fitness);
    Xmi = new_pop(Min_index(1),:);
    if (Jmin<=Jbest)
        Jbest = Jmin;
        Xbest = Xmi;
    end
    obj_opt(t)=Jbest;

    %best_sol_index=find(fitness==obj_opt(t));
    best_sol_gen(t,:)=Xbest;
    old_pop=new_pop;
end
delete(h);
obj_optm = obj_opt;
obj_optimal=min(obj_optm);
op_sol_index=find(obj_optm==obj_optimal);
X_best=best_sol_gen(op_sol_index(1),:);
return
end

% This function is responsible for the tournament selection operation it
% receives the following arguments:
% The number of solution (population) (N)
% The fitness value of each solution (fitness)
% The number of selected solution (Nps)
% The tournament selection probability (Tourn_Prob_f)
% Then it produces the selected parant index (Selected_Parent_index)

function Selected_Parent_index = Tourn_Select(N,fitness,Nps,Tourn_Prob_f)
Selected_Parent_index=zeros(Nps,2);
for i=1:Nps
    ti1=randi([1 N],1); %Random Tournament index 1
    ti2=randi([1 N],1); %Random Tournament index 2
    if rand <= Tourn_Prob_f
        if fitness(ti1)<fitness(ti2)
            pi1=ti1; %Parent index 1
        else
            pi1=ti2;
        end
    else
        if fitness(ti1)<fitness(ti2)
            pi1=ti2;
        else
            pi1=ti1;
        end
    end
    ti1=randi([1 N],1); %Random Tournament index 1
    ti2=randi([1 N],1); %Random Tournament index 2
    if rand()<= Tourn_Prob_f
        if fitness(ti1)<fitness(ti2)
            pi2=ti1; %Parent index 2
        else
            pi2=ti2;
        end
    else
        if fitness(ti1)<fitness(ti2)
            pi2=ti2;
        else
            pi2=ti1;
        end
    end
    Selected_Parent_index(i,:)=[pi1 pi2]; %Parent index vector
end
return
end
