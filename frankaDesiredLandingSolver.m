% FUNCTION DESCRIPTION:
% This function is the optimal control solver for throwing an object into a
% desired 3D position
%
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

function out = frankaDesiredLandingSolver(initialState,options)
%--------------------------------------------------------------------------
%                           Setting the OCP
%--------------------------------------------------------------------------
% Unpackaging the solver options
Tf = options.finalTime;                 % optimization max final time
stateBounds = options.stateBounds;      % state constraints
controlBounds = options.controlLimits;  % control constraints
c = options.controlsNumber;             % number of active joints
Ks = options.springValue;               % ee-wrist torsional spring guess
solver = options.solver;                % optimization solver
accBounds = options.accelerationLimits; % joint acceleration limits vector
g = 9.81;                               % gravity constant
pDes = options.desiredLanding;          % desired landing position

x0 = initialState;                      % initial robot configuration
m = length(x0);                         % state dimension

N = options.controlPoints;              % total steps
Ts = Tf/N;                              % initial sample time

qdim = ceil(m/2);                       % q dimension
xLast = [];                             % last optimal solution


% uOpt = [u(0),u(1),...,u(N-1)]'
uStart = 1;
uEnd = N*c;


% Computing initial guess
% x = [u(0),u(1),...,u(N-1)]'
initialGuess = options.initialGuess;


% System trajectory
sol = [];
time = [];

% Setting problem bounds
[lb,ub] = computeBounds(controlBounds);

% Setting solver options
fminconOpts = optimoptions('fmincon', ...
    'Display','iter-detailed',...
    'Algorithm',solver,...
    'MaxFunctionEvaluations', 5e4, ...
    'StepTolerance',          1e-4, ...
    'ConstraintTolerance',    1e-3,...
    'OptimalityTolerance',    1e-3,...
    'MaxIterations',          300);

% Solving the problem
disp(repmat('-',1,90));
disp('Starting optimization')
disp(repmat('-',1,90));
out = fmincon(@costFunction1, initialGuess, [], [], [], [], lb, ub,...
               @nlConstraints, fminconOpts);
           
disp(repmat('-',1,90));

%--------------------------------------------------------------------------
%                             OCP Functions
%--------------------------------------------------------------------------

%-----------------------JOINT LIMITS COST FUNCTION-------------------------

function cost = costFunction1(x)
if ~isequal(x,xLast)
    tau = x(uStart:uEnd);
    [~,sol] = sysIntegration(x0,tau,c,Tf,Ts,Ks);
    xLast = x;
end

maxVec = zeros(qdim-1,1);

for i = qdim+1:1:m-1
    maxVec(i-qdim) = max(abs(sol(:,i)));
end

max_diff =  maxVec - stateBounds(qdim+1:m-1,2);
cost = sum(max_diff);

end

%--------------------SENSITIVITY COST FUNCTION-----------------------------

function cost = costFunction2(x)
if ~isequal(x,xLast)
    tau = x(uStart:uEnd);
    [~,sol] = sysIntegration(x0,tau,c,Tf,Ts,Ks);
    xLast = x;
end

% Extracting the final robot state
qf = sol(end,1:qdim)';
dqf = sol(end,qdim+1:m)';

% Computing landing sensitivity during all the trajectory
sensitivity = frankaSensitivity(qf,dqf,pDes(3));
cost = norm(sensitivity);

end

%------------------MINIMUM ACCELERATION COST FUNCTION----------------------
function cost = costFunction3(x)
if ~isequal(x,xLast)
    tau = x(uStart:uEnd);
    [time,sol] = sysIntegration(x0,tau,c,Tf,Ts,Ks);
    xLast = x;
end
dqVariations = reshape(diff(sol(:,qdim+1:end-1)),[],1);
cost = dqVariations'*dqVariations;
end

%--------------------------UPPER AND LOWER BOUNDS--------------------------

function [lb, ub] = computeBounds(controlBounds)
  % Preallocating variable's bounds
  lb = zeros(size(initialGuess));
  ub = zeros(size(initialGuess));
 
  % setting control constraints
  lb(uStart:uEnd) = repmat(controlBounds(:,1),N,1);
  ub(uStart:uEnd) = repmat(controlBounds(:,2),N,1);
  
end

%---------------------------NONLINEAR CONSTRAINTS--------------------------

function [cineq,ceq] = nlConstraints(x)
% Integration of the trajectories
if ~isequal(x,xLast)
    tau = x(uStart:uEnd);
    [time,sol] = sysIntegration(x0,tau,c,Tf,Ts,Ks);
    xLast = x;
end

% Inequality constraints
maxVec = zeros(m,1);
minVec = zeros(m,1);
for i = 1:1:m
    maxVec(i) = max(sol(:,i));
    minVec(i) = min(sol(:,i));
end
max_diff =  maxVec - stateBounds(:,2);
min_diff = -minVec + stateBounds(:,1);
cineq = [max_diff; min_diff];


%Computing final position and velocity
qf = sol(end,1:qdim)';
dqf = sol(end,qdim+1:m)';

pf = frankaDirKin(qf);
pzf = pf(3);

vf = frankaVel(qf,dqf);
vzf = vf(3);

% Computing flight time
tFly = computeFlightTime(pzf, vzf, pDes(3));

%Computing landing position
xLand = pf(1) + vf(1)*tFly;
yLand = pf(2) + vf(2)*tFly;

% Landing position constraint
% 1) abs(xLand - pdes(1)) <= 0.2;
% 2) abs(yLand - pdes(2)) <= 0.2;

landTol = 0.05;
cLandx =  abs(xLand - pDes(1)) - landTol;
cLandy =  abs(yLand - pDes(2)) - landTol;

% Throwing constraint
throwingConst = vzf^2 + 2*g*(pzf - pDes(3));

% Acceleration constraint
timediff = diff(time);
der = zeros(length(timediff),qdim-1);
differences = diff(sol(:,qdim+1:15));
maxAcc = zeros(qdim-1,1);


if ~isempty(time)
    for i = 1:1:size(differences,2)
        der(:,i) = differences(:,i)./timediff;
        maxAcc(i) = max(der(:,i));
    end
end



cineq = [cineq;cLandx;cLandy;-throwingConst; maxAcc-accBounds];

% Equality constraints
% 1) velocity vector orthogonal to the ee: v_direction = -y_ee_direction
% T0ee = frankaTf(qf);
% ortVers = -T0ee(1:3,2);
% vfVers = vf/norm(vf);
% ceq = vfVers - ortVers;

ceq = [];

end

%-------------------------UTILITY FUNCTIONS--------------------------------

function tFly = computeFlightTime(pzr, vzr, pzdes)
    tFly = 0;
    if ( pzr > pzdes)
        if (vzr > 0)
            t_hmax = vzr/g;
            hmax = pzr + vzr^2/(2*g);
            t_fall = sqrt((hmax-pzdes)/g);
            tFly = t_hmax + t_fall;
        end
        if (vzr <= 0)
            tFly = (1/g)*(vzr + sqrt(vzr^2 + 2*g*(pzr-pzdes)));
        end
    else 
        if ( vzr <= 0)
            tFly = 0;
        end
        if (vzr > 0)
            hmax = pzr + vzr^2/(2*g);
            if (hmax < pzdes)
                tFly = 0;
            end
            if (hmax >= pzdes)
                t_hmax = vzr/g;
                hmax = pzr + vzr^2/(2*g);
                t_fall = sqrt((hmax-pzdes)/g);
                tFly = t_hmax + t_fall; 
            end
        end
    end

end

end









