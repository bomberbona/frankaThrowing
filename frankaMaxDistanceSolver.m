% This function is the optimal control solver for throwing at the maximum
% distance using the franka manipulator with the elastic wrist.
% INUPUT: 1)initialState = it is the robot initial state [q0,dq0]
%         2)option = structure containing the solver options, as the robot
%           state bounds, the solver method, final time, discretization
%           step...
% OUTPUT: out = optimal control sequence vector out = [u(0)',...,u(N-1)']'
%
%
% TIPS: for speeding up the computation time of the solver is necessary
% creating the mex functions of the 'sysIntergration' , 'frankaVel',
% 'frankaDirKin' functions.

function out = frankaMaxDistanceSolver(initialState,options)
%--------------------------------------------------------------------------
%                           Setting the OCP
%--------------------------------------------------------------------------
% Unpackaging the solver options
Tf = options.finalTime;                % optimization final time
Ts = options.samplingTime;             % integration step
stateBounds = options.stateBounds;     % state constraints
controlBounds = options.controlLimits; % control constraints
c = options.controlsNumber;            % number of active joints
Ks = options.springValue;              % ee-wrist torsional spring value
solver = options.solver;
%accelerationBounds = options.accelerationLimits;
g = 9.81;                              % gravity constant

x0 = initialState;                     % initial robot configuration
m = length(x0);                        % state dimension

N = ceil(Tf/Ts);                       % total steps
qdim = ceil(m/2);                      % q dimension
xLast = [];                            % last optimal solution

% Setting useful indexes
qStart = 1;
qEnd = qdim;
dqStart = qdim + 1;

% uOpt = [u(0),u(1),...,u(N-1)]'
uStart = 1;
uEnd = N*c;


% Computing initial guess
% x = [u(0),u(1),...,u(N-1),Ks]'
initialGuess = options.initialGuess;

% System trajectory
sol =[];

% Setting problem bounds
[lb,ub] = computeBounds(controlBounds);

% Setting solver options
fminconOpts = optimoptions('fmincon', ...
    'Display','iter-detailed',...
    'Algorithm',solver,...
    'MaxFunctionEvaluations', 5e4, ...
    'StepTolerance',          1e-8, ...
    'ConstraintTolerance',    1e-3,...
    'OptimalityTolerance',    1e-4,...
    'MaxIterations',          3000);

% Solving the problem
disp(repmat('-',1,70));
disp('Starting optimization')
disp(repmat('-',1,70));
out = fmincon(@costFunction, initialGuess, [], [], [], [], lb, ub,...
               @nlConstraints, fminconOpts);

%--------------------------------------------------------------------------
%                             OCP Functions
%--------------------------------------------------------------------------

%------------------------------COST FUNCTION-------------------------------

function cost = costFunction(x)
%Integration of the trajectories
if ~isequal(x,xLast)
    [~,sol] = sysIntegration(x0,x,c,Tf,Ts,Ks);
    xLast = x;
end
% Final robot state
qf = sol(end,1:qdim)';
dqf = sol(end,qdim+1:m)';

%Computing final position and velocity
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

%-----------------------LINEAR INEQUALITY BOUNDS---------------------------

function [lb, ub] = computeBounds(controlBounds)
  % Preallocating variable's bounds
  lb = zeros(size(initialGuess));
  ub = zeros(size(initialGuess));
 
  % setting control constraints
  lb(uStart:uEnd) = repmat(controlBounds(:,1),N,1);
  ub(uStart:uEnd) = repmat(controlBounds(:,2),N,1);
    
end

%--------------------------NL CONSTRAINTS----------------------------------

function [cineq,ceq] = nlConstraints(x)
% Integration of the trajectories
if ~isequal(x,xLast)
    [~,sol] = sysIntegration(x0,x,c,Tf,Ts,Ks);
    xLast = x;
end

% Computing final position and velocity
qf = sol(end,1:qdim)';
dqf = sol(end,qdim+1:m)';

pf = frankaDirKin(qf);
pzf = pf(3);
vf = frankaVel(qf,dqf);
vzf = vf(3);

% Throwing constraint
throwingConstraint = vzf^2 + 2*g*pzf;

% Inequality constraints
maxVec = zeros(m,1);
minVec = zeros(m,1);
for i = 1:1:m
    maxVec(i) = max(sol(:,i));
    minVec(i) = min(sol(:,i));
end
max_diff =  maxVec - stateBounds(:,2);
min_diff = -minVec + stateBounds(:,1);
cineq = [max_diff; min_diff; -throwingConstraint];


% Equality constraints
% 1)constraint for throwing a ball ortogonal to the EE
vfVers = vf/norm(vf);
T0ee = frankaTf(qf);
ortVers = -T0ee(1:3,2);
ceq = vfVers - ortVers;

% ceq =[];

end
end







