function optConfig = frankaOptimalConfiguration(stateBounds,initialGuess)
% This function optimize the initial guess obtained from the random
% sampling
%--------------------------------------------------------------------------
%                SETTING THE OPTIMIZATION PROBLEM
%--------------------------------------------------------------------------

lb = stateBounds(:,1); % state lower bounds
ub = stateBounds(:,2); % state upper bounds
g = 9.81;              % gravity
  
% Setting solver options
fminconOpts = optimoptions('fmincon', ...
    'Display','iter-detailed',...
    'Algorithm','sqp',...
    'MaxFunctionEvaluations', 1e5, ...
    'StepTolerance',          1e-8, ...
    'ConstraintTolerance',    1e-8,...
    'OptimalityTolerance',    1e-5,...
    'MaxIterations',          3000);

%--------------------------------------------------------------------------
%                  SOLVING THE OPTIMIZATION PROBLEM
%--------------------------------------------------------------------------
disp(repmat('-',1,70));
disp('Starting optimization')
disp(repmat('-',1,70));
optConfig = fmincon(@costFunction, initialGuess, [], [], [], [], lb, ub,...
               [], fminconOpts);
           
disp('Optimization completed! The optimal throwing configuration is:')
disp(strcat('q = [',num2str(optConfig(1:8)'),']'))
disp(strcat('dq = [',num2str(optConfig(9:16)'),']'))
disp(repmat('-',1,70));

%--------------------------------------------------------------------------
%                        PROBLEM FUNCTIONS
%--------------------------------------------------------------------------           
function cost = costFunction(x)
%Computing final position and velocity
qf = x(1:8);
dqf = x(9:16);
pf = frankaDirKin(qf);
pzf = pf(3);
vf = frankaVel(qf,dqf);
vzf = vf(3);

%Computing fligth time
tFly = (1/g)*(vzf + sqrt(vzf^2 + 2*g*pzf));
if vzf^2 + 2*g*pzf < 0
    tFly = 0;
end
%Computing landing position
xLand = pf(1) + vf(1)*tFly;
yLand = pf(2) + vf(2)*tFly;
cost = -sqrt(xLand*xLand + yLand*yLand);
end
            
end

