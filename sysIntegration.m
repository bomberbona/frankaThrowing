% Integration of the dynamics of the franka robot
% INPUT: 1) ic = robot initial conditions [q0', dq0']'
%        2) uVec = control sequence vector [u(0)',...,u(N-1)']'
%        3) cNum = number of active controls (7 active)
%        4) Tf = integration final time
%        5) Ts = discretization step of the control sequence
%        6) Ks = wrist stiffness value
%
% OUTPUT: 1) time = vector containing the time values of the ode45 
%            integration
%         2) y = matrix containing the state trajectory of the ode45
%            integration ([ q_1(t),...,q_8(t),dq_1(t),...dq_8(t)])
%
% TIPS = for speeding up the system dynamics integration it is necessary to
%        create the mex function of this file. It is important to pay
%        attention that the dynamics parameter of the robot can be changed
%        inside the createRobot() function, since it is not possible to pass a
%        rigidBodyTree object as function input if the function is a mex
%        function.
%
function [time,y] = sysIntegration(ic,uVec,cNum,Tf,Ts,Ks)
    sys = createRobot(); % creating the robot object
    controlLimits = [87, 87, 87, 87, 12, 12, 12]';
    n = length(ic);      % robot state dimension
    qend = ceil(n/2);    % number of joints
    t_u = 0:Ts:Tf-Ts;    % control points
    tspan = [0 Tf];
    
    % Integration of the dynamics
    [time,y] = ode45(@(t,y) dynamics(t,y,uVec,cNum,sys,Ks),tspan,ic);
    
    
function dydt = dynamics(t,y,uVec,cNum,sys,Ks)
    u = getControl(uVec,t,cNum);     % optimal control at time t
    dydt = zeros(n,1);               % state dynamics vector
    
    % Computing joint accelerations
    tau = [u;-Ks*y(qend)-0.05*y(n)]; % actuation vector
    jointAcc = forwardDynamics(sys, y(1:qend)', y(qend+1:n)', tau')';
    
    % Filling the dynamics vector
    dydt(1:qend) = y(qend+1:end);
    dydt(qend+1:n) = jointAcc; 
end

function u_t = getControl(uVec,t,cNum)
    u = reshape(uVec,cNum,length(t_u));
    u_t = zeros(cNum,1);
    for i = 1:1:cNum
        % Obtaining the interpolated control at time t
        if (t < Tf - Ts)
            u_t(i) = interp1(t_u,u(i,:),t,'pchip');
        else
            u_t(i) = u(i,end);
        end
        if abs(u_t(i)) >= controlLimits(i)
            u_t(i) = sign(u_t(i))*controlLimits(i);
        end
    end
end

function robot = createRobot()
% Robot links lengths
d1 = 0.333;
d3 = 0.316;
d5 = 0.384;
d7 = 0.107;
a4 = 0.0825;
a5 = -a4;
a7 = 0.088;
a8 = 0.15;

% DHM Table
% https://frankaemika.github.io/docs/control_parameters.html
% Modified DH Table
%          a     alpha      d       theta
dhtable_rt =  [0,    0,         d1,     0;
               0    -pi/2,      0,      0;
               0,    pi/2,      d3,     0;
               a4,   pi/2,      0,      0;
               a5,  -pi/2,      d5,     0;
               0,    pi/2,      0,      0;
               a7,   pi/2,      d7,     0;
               0,    pi/2,      0,      0;
              a8,       0,      0,      0];

dhtable_rt = double(dhtable_rt);

% Creating the robot
robot = rigidBodyTree("MaxNumBodies",9,"DataFormat","row");
links = cell(9,1);
joints = cell(9,1);

links{1} = rigidBody("link1");
joints{1} = rigidBodyJoint("joint1", 'revolute');
setFixedTransform(joints{1}, dhtable_rt(1,:), 'mdh');

links{2} = rigidBody("link2");
joints{2} = rigidBodyJoint("joint2", 'revolute');
setFixedTransform(joints{2}, dhtable_rt(2,:), 'mdh');

links{3} = rigidBody("link3");
joints{3} = rigidBodyJoint("joint3", 'revolute');
setFixedTransform(joints{3}, dhtable_rt(3,:), 'mdh');

links{4} = rigidBody("link4");
joints{4} = rigidBodyJoint("joint4", 'revolute');
setFixedTransform(joints{4}, dhtable_rt(4,:), 'mdh');

links{5} = rigidBody("link5");
joints{5} = rigidBodyJoint("joint5", 'revolute');
setFixedTransform(joints{5}, dhtable_rt(5,:), 'mdh');

links{6} = rigidBody("link6");
joints{6} = rigidBodyJoint("joint6", 'revolute');
setFixedTransform(joints{6}, dhtable_rt(6,:), 'mdh');

links{7} = rigidBody("link7");
joints{7} = rigidBodyJoint("joint7", 'revolute');
setFixedTransform(joints{7}, dhtable_rt(7,:), 'mdh');

links{8} = rigidBody("link8");
joints{8} = rigidBodyJoint("joint8", 'revolute');
setFixedTransform(joints{8}, dhtable_rt(8,:), 'mdh');

links{9} = rigidBody("ee");
joints{9} = rigidBodyJoint("joint9",'fixed');
setFixedTransform(joints{9}, dhtable_rt(9,:), 'mdh');

% Setting joint limits
joints{1}.PositionLimits = [-2.8973, 2.8973];
joints{2}.PositionLimits = [-1.7628, 1.7628];
joints{3}.PositionLimits = [-2.8973, 2.8973];
joints{4}.PositionLimits = [-3.0718, 0.0698];
joints{5}.PositionLimits = [-2.8973, 2.8973];
joints{6}.PositionLimits = [-0.0175, 3.7525];
joints{7}.PositionLimits = [-2.8973, 2.8973];
joints{8}.PositionLimits = [-pi/4, pi/4];

% Setting links dynamic parameters
m1Val = 4.97;
m2Val = 0.6469;
m3Val = 3.2286;
m4Val = 3.5878;
m5Val = 1.2259;
m6Val = 1.6665;
m7Val = 0.7355;
m8Val = 0.5;
mEEVal = 0.1;
Ixx8 = 0.5 * m8Val* 0.03^2;
Iyy8 = (1/12) * m8Val * a8^2 + (1/4)* m8Val * 0.03^2;
Izz8 = Iyy8;

% Setting links masses
links{1}.Mass = m1Val;
links{2}.Mass = m2Val;
links{3}.Mass = m3Val;
links{4}.Mass = m4Val;
links{5}.Mass = m5Val;
links{6}.Mass = m6Val;
links{7}.Mass = m7Val;
links{8}.Mass = m8Val;
links{9}.Mass = mEEVal;

% Setting links center of mass
links{1}.CenterOfMass = [         0,          0,     -0.175];
links{2}.CenterOfMass = [ -3.141e-3,  -2.872e-2,   3.495e-3];
links{3}.CenterOfMass = [ 2.7518e-2,  3.9252e-2, -6.6502e-2];
links{4}.CenterOfMass = [ -5.317e-2, 1.04419e-1,  2.7454e-2];
links{5}.CenterOfMass = [-1.1953e-2,  4.1065e-2, -3.8437e-2];
links{6}.CenterOfMass = [ 6.0149e-2, -1.4117e-2, -1.0517e-2];
links{7}.CenterOfMass = [ 1.0517e-2,  -4.252e-3,  6.1597e-2];
links{8}.CenterOfMass = [      a8/2,          0,          0];
links{9}.CenterOfMass = [         0,          0,          0];

% Setting inertia tensor
%              I = [    Ixx        Iyy      Izz         Iyz          Ixz       Ixy    ]
links{1}.Inertia = [7.0337e-1, 7.0661e-1, 9.1170e-3,  1.9169e-2,  6.7720e-3, -1.390e-4];
links{2}.Inertia = [7.9620e-2, 2.8110e-2, 2.5995e-2,  7.0400e-4,  1.0254e-2, -3.925e-3];
links{3}.Inertia = [3.7242e-2, 3.6155e-2, 1.0830e-2, -1.2805e-2, -1.1396e-2, -4.761e-3];
links{4}.Inertia = [2.5853e-2, 1.9552e-2, 2.8323e-2,  8.6410e-3, -1.3320e-3,  7.796e-3];
links{5}.Inertia = [3.5549e-2, 2.9474e-2, 8.6270e-3,  2.2900e-4, -4.0370e-3, -2.117e-3];
links{6}.Inertia = [1.9640e-3, 4.3540e-3, 5.4330e-3,  3.4100e-4, -1.1580e-3,  1.090e-4];
links{7}.Inertia = [1.2516e-2, 1.0027e-2, 4.8150e-3, -7.4100e-4, -1.1960e-3, -4.280e-4];
links{8}.Inertia = [    Ixx8,       Iyy8,      Izz8,          0,          0,         0];
links{9}.Inertia = [        0,         0,         0,          0,          0,         0];


% Connecting the links 
links{1}.Joint = joints{1};
addBody(robot, links{1}, "base");
for i = 2:1:(size(dhtable_rt, 1)-1)
    links{i}.Joint = joints{i};
end
addBody(robot, links{2}, "link1");
addBody(robot, links{3}, "link2");
addBody(robot, links{4}, "link3");
addBody(robot, links{5}, "link4");
addBody(robot, links{6}, "link5");
addBody(robot, links{7}, "link6");
addBody(robot, links{8}, "link7");
links{9}.Joint = joints{9};
addBody(robot, links{9},"link8")

robot.DataFormat = 'row';
g = 9.81;
robot.Gravity = [0, 0, -g];
end
end

