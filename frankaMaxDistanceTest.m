%% Setting the solver parameters
%--------------------------------------------------------------------------
%           Test file for the optFranka1
%--------------------------------------------------------------------------
% Robot joint limits
jointAngleLimits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -pi/2, -pi/4;
                     2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  pi/2,  pi/4];
                 
jointSpeedLimits = [-2.15, -2.15, -2.15, -2.15, -2.6, -2.6, -2.6, -7;
                     2.15,  2.15,  2.15,  2.15,  2.6,  2.6,  2.6,  7];
                 
accelerationLimits = [15, 7.5, 10, 12.5, 15, 20, 20, 20]';
                 
stateLb = [jointAngleLimits(1,:),jointSpeedLimits(1,:)]';     
stateUb = [jointAngleLimits(2,:),jointSpeedLimits(2,:)]';
stateBounds = [stateLb, stateUb];

% Robot torque limits
controlLimits = [87, 87, 87, 87, 12, 12, 12]';
controlBounds = [-controlLimits, controlLimits];

% Robot elastic joint parameters
Ks = 13;       % Nm/rad

% Options for solver
Ts = 0.04;
Tf = 0.8;
controlPoints = Tf/Ts;

options.finalTime = Tf;                    % optimization final time
options.samplingTime = Ts;                 % optimization sampling time
options.stateBounds = stateBounds;         % state lower and upper bounds
options.controlLimits = controlBounds;     % torque lower and upper bounds  
options.controlsNumber = 7;                % num of active joints
options.springValue = Ks;                  % wrist torsional spring guess
options.accelerationLimits = [-accelerationLimits,accelerationLimits];
options.controlPoints = controlPoints;
options.initialGuess = sol;
options.solver = 'sqp';

% Robot initial robot state
q0 =  [0, -0.7626, -1.0000, -2.3435, -0.0306, 1.5397, 0.7539, 0]';
dq0 = zeros(8,1);

initialState = [q0', dq0']';              % robot initial state
N = controlPoints;                        % problem control points

%% Solving the optimal control
sol = frankaMaxDistanceSolver(initialState,options);

%% Plotting results
t_u = 0:Ts:Tf-Ts; % Control points time vector

% Obtaining the state trajectories
u = sol;
tau = reshape(u,7,N);
[time, y] = sysIntegration(initialState,u,7,Tf,Ts,Ks);
traj = y';


% Plotting the joint trajectories
figure(20)
for i = 1:1:8
    subplot(2,4,i)
    cla
    hold on
    xlim([0 Tf])
    plot(time,traj(i,:),'k','linewidth',1.5)
%     plot([time(1), time(end)],[stateBounds(i,1),stateBounds(i,1)],...
%         'r--','linewidth',1.5);
%     plot([time(1), time(end)],[stateBounds(i,2),stateBounds(i,2)],...
%         'r--','linewidth',1.5);
    xlabel('$Time\ [sec]$','interpreter','latex')
    ylabel('$[rad]$','interpreter','latex')
    legend(strcat('$q_',num2str(i),'$'),'location','southeast',...
           'interpreter','latex');
    grid on
end
sgtitle('Joint angle solutions')

% Plotting the velocity trajectories
figure(21)
for i = 9:1:16
    subplot(2,4,i-8)
    %cla
    hold on
    plot(time,traj(i,:),'k','linewidth',1.5)
    plot([time(1), time(end)],[stateBounds(i,1),stateBounds(i,1)],...
        'r--','linewidth',1.5);
    plot([time(1), time(end)],[stateBounds(i,2),stateBounds(i,2)],...
        'r--','linewidth',1.5);
    xlim([0,Tf])
    xlabel('$Time\ [sec]$','interpreter','latex')
    ylabel('$[rad/s]$','interpreter','latex')
    legend(strcat('$\dot q_',num2str(i-8),'$'),'$speed\ bounds$','location','southeast',...
           'interpreter','latex');
    grid on
end
sgtitle('Joint speed solutions')

% Plotting the control actions
tau_interp = zeros(7,length(time));
trueControlLimits = [87, 87, 87, 87, 12, 12, 12]';
for i = 1:1:7
    tau_interp(i,:) = interp1(t_u,tau(i,:),time,'pchip');
    for k = 1:1:size(tau_interp,2)
        if (time(k) >= Tf - Ts)
            tau_interp(i,k) = tau_interp(i,k-1);
        end
        if abs(tau_interp(i,k)) >= trueControlLimits(i)
            tau_interp(i,k) = sign(tau_interp(i,k))*trueControlLimits(i);
        end
    end
end

figure(22)
for i = 1:1:7
    subplot(2,4,i)
    %cla
    hold on
    plot(time,tau_interp(i,:),'k','linewidth',1.5);
%     plot([time(1), time(end)],[controlBounds(i,1),controlBounds(i,1)],...
%         'r--','linewidth',1.5);
%     plot([time(1), time(end)],[controlBounds(i,2),controlBounds(i,2)],...
%         'r--','linewidth',1.5);
    xlim([0,Tf])
    xlabel('$Time\ [sec]$','interpreter','latex')
    ylabel('$[Nm]$','interpreter','latex')
    grid on;
    legend(strcat('$\tau_',num2str(i),'$'),'interpreter','latex');
end
sgtitle('Control solutions')

%% Robot animation
%Plotting the robot animation
figure(23)
axis('equal')
distance = frankadistance(traj(1:8,end),traj(9:end,end));
sgtitle(strcat('Throwing distance: ',num2str(distance), ' m'));
grid on
xlims = [-4 4];
ylims = [-4 4];
zlims = [0 1.5];
lims = [xlims; ylims; zlims];
%pause(5);
pdes = [0 0 0]';
robotAnimation(robot,traj,lims)

%% End effector trajectory
pEE = zeros(3,length(time));
for i = 1:1:length(time)
    pEE(:,i) = frankaDirKin(traj(1:8,i));
end
figure(23)
plot3(pEE(1,:),pEE(2,:),pEE(3,:),'r','linewidth',1.5)
show(robot,traj(1:8,1)')

%% Throwing distance and elastic energy
dVec = zeros(size(traj,2),1);
for i = 1:1:length(dVec)
    dVec(i) = frankadistance(traj(1:8,i),traj(9:16,i));
end
%%
% Throwing distance versus time
figure(1)
hold on
plot(time,dVec,'linewidth',2);
grid on
xlabel('Time [s]')
ylabel('[m]')
sgtitle('Throwing distance');

% Computing elastic energy during the throwing motion
elasticEnergy = zeros(size(traj,2),1);
for i = 1:1:length(elasticEnergy)
    elasticEnergy(i) = 0.5*Ks*traj(8,i)^2;
end

figure(26)
cla
plot(time,elasticEnergy,'linewidth',2)
grid on
xlabel('Time [s]')
ylabel('[Joule]')
sgtitle('Elastic energy of the elastic joint')

eeVelocity = zeros(size(traj,2),1);
for i = 1:1:length(eeVelocity)
    eeVelocity(i) = norm(frankaVel(traj(1:8,i),traj(9:16,i)));
end

figure(27)
cla
plot(time,eeVelocity,'linewidth',2)
hold on
grid on
xlabel('Time [s]')
ylabel('Velocity [m/s]')
sgtitle('End effector velocity')

%% Utility functions
function d = frankadistance(q,dq)
g = 9.81;
pf = frankaDirKin(q);
pzf = pf(3);
vf = frankaVel(q, dq);
vzf = vf(3);
tFly = (1/g)*(vzf + sqrt(vzf^2 + 2*g*pzf));
xLand = pf(1) + vf(1)*tFly;
yLand = pf(2) + vf(2)*tFly;
d = sqrt(xLand*xLand + yLand*yLand);
end

function robotAnimation(robot,traj,lims)
qf = traj(1:8,end);
dqf = traj(9:end,end);
g = 9.81;
g0 = [0; 0; -g];
pf = frankaDirKin(qf);
pzf = pf(3);
vf = frankaVel(qf, dqf);
vzf = vf(3);
tFly = (1/g)*(vzf + sqrt(vzf.^2 + 2*g*pzf));

th = 0:0.01:2*pi;
rMax = 10;
xarea = rMax*cos(th);
yarea = rMax*sin(th);

for i = 1:6:size(traj,2)-1
    cla
    xlim(lims(1,:))
    ylim(lims(2,:))
    zlim(lims(3,:))
    p = frankaDirKin(traj(1:8,i));
    [ballx,bally,ballz] = createBall(p);
    hold on
    s = surf(ballx,bally,ballz);
    s.EdgeColor = 'r'; 
    s.FaceColor = 'r'; 
    a = area(xarea,yarea);
    a.LineWidth = 3;
    a.EdgeColor = [0.98,0.98,0.98];
    a.FaceColor = [0.98,0.98,0.98];
    show(robot,traj(1:8,i)');
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    drawnow
end
t = 0.02:0.02:tFly;
pVec = zeros(3,length(t));
hold on
for k = 1:1:length(t)
    cla
    xlim(lims(1,:))
    ylim(lims(2,:))
    zlim(lims(3,:))
    p = frankaDirKin(traj(1:8,end)) + vf*t(k) + 0.5*g0*t(k)^2;
    disp(p)
    if p(3) <= 0.04
        p(3) = 0.04;
    end
    pVec(:,k) = p;
    [ballx,bally,ballz] = createBall(p);
    hold on
    s = surf(ballx,bally,ballz);
    s.EdgeColor = 'r'; 
    s.FaceColor = 'r'; 
    plot3(pVec(1,1:k),pVec(2,1:k),pVec(3,1:k),'r--','linewidth',1)
    a = area(xarea,yarea);
    a.LineWidth = 3;
    a.EdgeColor = [0.98,0.98,0.98];
    a.FaceColor = [0.98,0.98,0.98];
    show(robot,traj(1:8,end)');
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    drawnow
end


function [ballx, bally, ballz] = createBall(pos)
ballCenter = pos;
r = 0.04;
[x,y,z] = sphere(100);
ballx = x*r + ballCenter(1);
bally = y*r + ballCenter(2);
ballz = z*r + ballCenter(3);
end

end
