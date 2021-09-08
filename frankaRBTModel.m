%% Creating panda robot kinematic chain
% Robot parameters
sympref('FloatingPointOutput',true);
d1 = 0.333;
d3 = 0.316;
d5 = 0.384;
d7 = 0.107;
a4 = 0.0825;
a5 = -a4;
a7 = 0.088;
a8 = 0.15;
vals = [d1 d3 d5 d7 a4 a5 a7 a8];

% DHM Table
syms q1 q2 q3 q4 q5 q6 q7 q8 real;
q = [q1 q2 q3 q4 q5 q6 q7 q8]';
% https://frankaemika.github.io/docs/control_parameters.html
% Modified DH Table
%          a     alpha      d       theta
dhtable = [0,    0,         d1,     q1;
           0    -pi/2,      0,      q2;
           0,    pi/2,      d3,     q3;
           a4,   pi/2,      0,      q4;
           a5,  -pi/2,      d5,     q5;
           0,    pi/2,      0,      q6;
           a7,   pi/2,      d7,     q7;
           0,    pi/2,      0,      q8;
          a8,       0,      0,       0];

dhtable_rt = dhtable;
dhtable_rt(:, 4) = zeros(size(dhtable,1), 1);
dhtable_rt = double(dhtable_rt);

% Creating the robot
robot = rigidBodyTree("MaxNumBodies",9,"DataFormat","row");
links = {};
joints = {};
for i = 1:1:(size(dhtable_rt, 1)-1)
    links{i} = rigidBody(strcat("link", num2str(i)));
    joints{i} = rigidBodyJoint(strcat("joint", num2str(i)), 'revolute');
    setFixedTransform(joints{i}, dhtable_rt(i,:), 'mdh');
end
links{9} = rigidBody("ee");
joints{9} = rigidBodyJoint("joint9",'fixed');
setFixedTransform(joints{9}, dhtable_rt(9,:), 'mdh');

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

% mass
links{1}.Mass = m1Val;
links{2}.Mass = m2Val;
links{3}.Mass = m3Val;
links{4}.Mass = m4Val;
links{5}.Mass = m5Val;
links{6}.Mass = m6Val;
links{7}.Mass = m7Val;
links{8}.Mass = m8Val;
links{9}.Mass = mEEVal;

% center of mass
links{1}.CenterOfMass = [         0,          0,     -0.175];
links{2}.CenterOfMass = [ -3.141e-3,  -2.872e-2,   3.495e-3];
links{3}.CenterOfMass = [ 2.7518e-2,  3.9252e-2, -6.6502e-2];
links{4}.CenterOfMass = [ -5.317e-2, 1.04419e-1,  2.7454e-2];
links{5}.CenterOfMass = [-1.1953e-2,  4.1065e-2, -3.8437e-2];
links{6}.CenterOfMass = [ 6.0149e-2, -1.4117e-2, -1.0517e-2];
links{7}.CenterOfMass = [ 1.0517e-2,  -4.252e-3,  6.1597e-2];
links{8}.CenterOfMass = [      a8/2,          0,          0];
links{9}.CenterOfMass = [         0,          0,          0];

% inertia tensor
%              I = [    Ixx        Iyy      Izz         Iyz          Ixz       Ixy    ]
links{1}.Inertia = [7.0337e-1, 7.0661e-1, 9.1170e-3,  1.9169e-2,  6.7720e-3, -1.390e-4];
links{2}.Inertia = [7.9620e-2, 2.8110e-2, 2.5995e-2,  7.0400e-4,  1.0254e-2, -3.925e-3];
links{3}.Inertia = [3.7242e-2, 3.6155e-2, 1.0830e-2, -1.2805e-2, -1.1396e-2, -4.761e-3];
links{4}.Inertia = [2.5853e-2, 1.9552e-2, 2.8323e-2,  8.6410e-3, -1.3320e-3,  7.796e-3];
links{5}.Inertia = [3.5549e-2, 2.9474e-2, 8.6270e-3,  2.2900e-4, -4.0370e-3, -2.117e-3];
links{6}.Inertia = [1.9640e-3, 4.3540e-3, 5.4330e-3,  3.4100e-4, -1.1580e-3,  1.090e-4];
links{7}.Inertia = [1.2516e-2, 1.0027e-2, 4.8150e-3, -7.4100e-4, -1.1960e-3, -4.280e-4];
links{8}.Inertia = [    Ixx8,       Iyy8,      Izz8,          0,          0,         0];
links{9}.Inertia = [     1e-5,      1e-5,      1e-5,          0,          0,         0];


% Connecting the links 
links{1}.Joint = joints{1};
addBody(robot, links{1}, "base");
for i = 2:1:(size(dhtable_rt, 1)-1)
    links{i}.Joint = joints{i};
    addBody(robot, links{i}, strcat("link", num2str(i-1)));
end
links{9}.Joint = joints{9};
addBody(robot, links{9},"link8")

robot.DataFormat = 'row';
g = 9.81;
robot.Gravity = [0, 0, -g];
pose = zeros(1,8);
show(robot,pose);
xlabel('x')
ylabel('y')

tf = getTransform(robot,[zeros(5,1)',pi/2,0,0],'ee','base');
T0ee = tfFromDHM(dhtable);
d0ee = T0ee(1:3,4);
% matlabFunction(d0ee,'file','functions/frankaDirKin','vars',{q})
%%
% Collecting the frames origins in a matrix
origin = sym(zeros(3,size(dhtable,1)));
for i = 1:1:size(origin,2)
    T = tfFromDHM(dhtable(1:i,:));
    d = T(1:3,4);
    origin(:,i) = d;
end

originSym = origin;
%% Differential kinematics
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 dq8 real
dq = [dq1 dq2 dq3 dq4 dq5 dq6 dq7 dq8]';
Jac_pos = jacobian(d0ee, q);
v = Jac_pos*dq;
% matlabFunction(v,'file','functions\frankaVel','vars',{q,dq})

%% Robot plot
syms dq d3 d5 d7 a3 a4 a6 a7 real
params = ["d1" "d3" "d5" "d7" "a3" "a4" "a6" "a7" "a8"];
vals = [d1 d3 d5 d7 a3 a4 a6 a7 a8];

qVal = [0 0 0 0 0 0 0 0]; % robot configuration

figure(1)
cla
grid on
plotRobot(q,qVal,params,vals,originSym)
show(robot,qVal);
axis('equal')
xlim([-1 1])
ylim([-1 1])
zlim([0 1.25])
xlabel('x')
ylabel('y')
zlabel('z')
sgtitle(strcat('Franka arm:  q = [',num2str(qVal(1)),', ',...
                                    num2str(qVal(2)),', ',...
                                    num2str(qVal(3)),', ',...
                                    num2str(qVal(4)),', ',...
                                    num2str(qVal(5)),', ',...
                                    num2str(qVal(6)),', ',...
                                    num2str(qVal(7)),', ',...
                                    num2str(qVal(8)),...
                                    ']'))

%% Random sampling for optimal throwing configuration

jointAngleLimits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -pi/2, 0;
                     2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  pi/2, 0];
                 
jointSpeedLimits = [-2.17, -2.17, -2.17, -2.17, -2.61, -2.61, -2.61, 0;
                     2.17,  2.17,  2.17,  2.17,  2.61,  2.61,  2.61, 0];
                 
stateLb = [jointAngleLimits(1,:),jointSpeedLimits(1,:)]';     
stateUb = [jointAngleLimits(2,:),jointSpeedLimits(2,:)]';
stateBounds = [stateLb, stateUb];


rng(0,'twister');
qsample = zeros(8,4e5);
dqsample = zeros(8,4e5);
for i = 1:1:size(qsample,2)
    for j = 1:1:8
        qmin = jointAngleLimits(1,j);
        qmax = jointAngleLimits(2,j);
        qsample(j,i) = (qmax-qmin).*rand(1,1) + qmin;
        dqmin = jointSpeedLimits(1,j);
        dqmax = jointSpeedLimits(2,j);
        dqsample(j,i) = (dqmax-dqmin).*rand(1,1) + dqmin;
    end
end

%% Optimal throwing configuration
% Choosing the best couple of configuration and velocity the maximize the
% throwing distance
d = zeros(size(qsample,2),1);
for i = 1:1:length(d)
    d(i) = distance(qsample(:,i),dqsample(:,i));
end
[dmax,idx] = max(d);
qmax = qsample(:,idx);
dqmax = dqsample(:,idx);
initialGuess = [qmax;dqmax];

% Optimizing from the initialGuess
optConf = frankaOptimalConfiguration(stateBounds,initialGuess);
qmax = optConf(1:8);
dqmax = optConf(9:16);
dmax = distance(qmax,dqmax);

clear qsample dqsample
vf = frankaVel(qmax,dqmax);
pee = double(subs(d0ee,q,qmax));
vfVers = vf/norm(vf);
pvel = vfVers*0.3 + pee;

figure(5)
show(robot,qmax')
hold on
plot3([pee(1), pee(1)+vfVers(1)*0.2],[pee(2), pee(2)+vfVers(2)*0.2],...
      [pee(3), pee(3)+vfVers(3)*0.2],'r','linewidth',1.5)


%% Plotting the final robot configuration
% plotting the velocity vector 
figure(2)
cla
%plotRobot(q,qmax,params,vals,originSym)
show(robot,qmax');
axis('equal')
xlim([-1 1])
ylim([-1 1])
zlim([0 1.25])
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on;
grid on;
plot3([pee(1),pvel(1)],[pee(2),pvel(2)],[pee(3),pvel(3)],'r','Linewidth',1.5)



%% Throwing sensitivity analysis
syms g pzdes real
g0 = [0; 0; -g];
v = Jac_pos*dq;
vzf = v(3);
pzf = d0ee(3);
tFly =  (1/g)*(vzf + sqrt(abs(vzf^2 + 2*g*(pzf - pzdes))));
d = d0ee + v*tFly + 0.5*g0*tFly^2;
h = v*tFly + 0.5*g0*tFly^2;

DpDq = jacobian(d0ee,q);
DpDdq = zeros(3,length(q));
DpDstate = [DpDq, DpDdq];

DhDq = jacobian(h,q);
DhDdq = jacobian(h,dq);
DhDstate = [DhDq, DhDdq];


d_sensitivity = DpDstate + DhDstate;
d_sensitivity = subs(d_sensitivity,g, 9.81);
% matlabFunction(d_sensitivity,'file','functions\frankaSensitivity','vars',{q,dq,pzdes})





%% Utility functions
function T = tfFromDHM(dhTable)
T = eye(4);
for i = 1:1:size(dhTable,1)
    a = dhTable(i,1);
    alpha = dhTable(i,2);
    d = dhTable(i,3);
    th = dhTable(i,4);
    sth = sin(th);
    cth = cos(th);
    cal = cos(alpha);
    sal = sin(alpha);
    tf1 = [cth, -sth, 0, a];
    tf2 = [sth*cal, cth*cal, -sal, -d*sal];
    tf3 = [sth*sal, cth*sal, cal, d*cal];
    tf4 = [0, 0, 0, 1];
    tf = [tf1; tf2; tf3; tf4];
    T = T*tf;   
end
end

function d = distance(q,dq)
g = 9.81;
pf = frankaDirKin(q);
pzf = pf(3);
vf = frankaVel(q, dq);
vzf = vf(3);
tFly = (1/g)*(vzf + sqrt(vzf^2 + 2*g*pzf));
xLand = pf(1) + vf(1)*tFly;
yLand = pf(2) + vf(2)*tFly;
d = -sqrt(xLand*xLand + yLand*yLand);
end
