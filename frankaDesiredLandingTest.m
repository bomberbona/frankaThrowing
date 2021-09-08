%% Setting the solver parameters
%--------------------------------------------------------------------------
%         Test file for the optFranka2 optimal control solver
%--------------------------------------------------------------------------
% Robot joint limits
jointAngleLimits = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -pi/2, -pi/4;
                     2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  pi/2, pi/4];
                                                   
jointSpeedLimits = [-1.8, -1.8, -1.8, -1.8, -2.4, -2.4, -2.4, -7;
                     1.8,  1.8,  1.8,  1.8,  2.4,  2.4,  2.4,  7]; 
                                  
stateLb = [jointAngleLimits(1,:),jointSpeedLimits(1,:)]';     
stateUb = [jointAngleLimits(2,:),jointSpeedLimits(2,:)]';
stateBounds = [stateLb, stateUb];

accelerationLimits = [15, 7.5, 10, 12.5, 15, 20, 20]';

% Robot torque limits
OptControlLimits = [80, 80, 80, 80, 10, 10, 10]';
controlBounds = [-OptControlLimits, OptControlLimits];


% Robot elastic joint parameters
Ks = 13;        % Nm/rad

% Options for solver
Ts = 0.07;
Tf = 0.7;
controlPoints = 10;
pdes = [1.2; 0; 0.2]; % desired landing position

options.finalTime = Tf;                    % optimization final time
options.samplingTime = Ts;                 % optimization sampling time
options.stateBounds = stateBounds;         % state lower and upper bounds
options.controlLimits = controlBounds;     % torque lower and upper bounds  
options.controlsNumber = 7;                % num of active joints
options.springValue = Ks;                  % wrist torsional spring guess
options.accelerationLimits = [-accelerationLimits,accelerationLimits];
options.desiredLanding = pdes;
options.controlPoints = controlPoints;
options.initialGuess = sol;
options.solver = 'sqp';
options.accelerationLimits = accelerationLimits;

% Robot initial robot state
q0 = [0, -0.17, 0, -2.34, 0, 0.74, 0, 0]';
dq0 = zeros(8,1);
initialState = [q0', dq0']';              % robot initial state

N = controlPoints;

%% Solving the optimal control
sol = frankaDesiredLandingSolver(initialState,options);


%% Plotting results
% Reshaping the control sequence solution
u = sol;
tau = reshape(u,7,N);
Ts = Tf/controlPoints;
%--------------------------------------------------------------------------
% Integration of the dynamics
[time, y] = sysIntegration_reduced_mex(initialState,u,7,Tf,Ts,Ks);
traj = y';
t_u = 0:Ts:Tf-Ts; % Control points


%--------------------------SAVING OF THE RESULTS---------------------------
% save('optimization/frankaLand_new3','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLand26_minControl','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLand26_minSensitivity','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLandFixed20','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLandFixed5_minSensitivity','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLandOrtogona20','sol','traj','time','Tf','Ts','Ks','pdes','q0')
% save('optimization/frankaLandOrtogonal2_minSensitivity','sol','traj','time','Tf','Ts','Ks','pdes','q0')
%--------------------------------------------------------------------------

% Plotting the joint angle trajectories
figure(20)
for i = 1:1:8
    subplot(2,4,i)
    hold on
    cla
    plot(time,traj(i,:),'linewidth',1.5)
    xlim([0,Tf])
    xlabel('Time [s]')
    ylabel('[rad]')
    legend(strcat('q_',num2str(i)));
    grid on
end
sgtitle('Joint angle solutions')

% Plotting the joint speed trajectories
figure(44)
for i = 9:1:16
    subplot(2,4,i-8)
    grid on
    cla
    hold on
    plot(time,traj(i,:),'linewidth',1.5)
    if i < 16
      plot([time(1), time(end)],[stateBounds(i,1),stateBounds(i,1)],...
          'r--','linewidth',1.5);
      plot([time(1), time(end)],[stateBounds(i,2),stateBounds(i,2)],...
          'r--','linewidth',1.5);
     end
    xlim([0,Tf])
    xlabel('Time [s]')
    ylabel('[rad/s]')
    legend(strcat('dq_',num2str(i-8)));
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
    cla
    hold on
    plot(time,tau_interp(i,:),'linewidth',1.5);
    xlim([0,Tf])
    xlabel('Time [s]')
    ylabel('[Nm]')
    grid on;
    legend(strcat('\tau_',num2str(i)));
end
sgtitle('Control solutions')

%% Robot animation
%Plotting the robot animation
figure(33)
axis('equal')
sgtitle(strcat('Throwing motion - desired position: [',num2str(pdes(1)),',',num2str(pdes(2)),',',num2str(pdes(3)),']'))
grid on
xlim([-1 3.3])
ylim([-2 2])
zlim([0 1.5]);
limits = [xlim;ylim;zlim];
%pause(5);
robotAnimation(robot,traj,pdes,limits)

%% End effector trajectory
pEE = zeros(3,length(time));
for i = 1:1:length(time)
    pEE(:,i) = frankaDirKin(traj(1:8,i));
end
figure(23)
hold on
plot3(pEE(1,:),pEE(2,:),pEE(3,:),'r','linewidth',1.5)
show(robot,traj(1:8,1)')

%% End effector final velocity
vf = frankaVel(traj(1:8,end),traj(9:end,end));
vfVers = vf/norm(vf);
vfVec = vfVers*0.2;
plot3([pEE(1,end),pEE(1,end)+vfVec(1)],...
      [pEE(2,end),pEE(2,end)+vfVec(2)],...
      [pEE(3,end),pEE(3,end)+vfVec(3)],'k','linewidth',1.5);

%% Throwing distance and sensitivity
% Computing the throwig distance
dVec = zeros(size(traj,2),1);
sensitivityVec = zeros(size(traj,2),1);
for i = 1:1:length(dVec)
    dVec(i) = frankadistance(traj(1:8,i),traj(9:16,i),pdes);
    sensitivityVec(i) = norm(frankaSensitivity_mex(traj(1:8,i),traj(9:16,i),pdes(3)));
end

sensitivityVecNew = 0;
for i = 1:1:length(sensitivityVec)
    if(sensitivityVec(i)>0)
        sensitivityVecNew = [sensitivityVecNew;sensitivityVec(i)];
    end
end
sensitivityVecNew = sensitivityVecNew(2:end); 
timeNew = time(1:10:end);
sensitivityVecInterp = interp1(timeNew,sensitivityVecNew,time,'pchip');
%% Utility plot
% Throwing distance versus time
figure(24)
hold on
plot(time,dVec,'linewidth',1.5);
grid on
xlabel('Time [s]')
ylabel('[m]')
sgtitle('Throwing distance');

figure(30)
hold on
plot(time,sensitivityVec,'linewidth',1.5);
grid on
xlabel('Time [s]')
ylabel('\sigma_{max}')
sgtitle('Throwing sensitivity');

 
% Computing elastic energy during the throwing motion
elasticEnergy = zeros(size(traj,2),1);
for i = 1:1:length(elasticEnergy)
    elasticEnergy(i) = 0.5*Ks*traj(8,i)^2;
end
figure(26)
cla
plot(time,elasticEnergy,'linewidth',1.5)
grid on
xlabel('Time [s]')
ylabel('[Joule]')
sgtitle('Elastic energy of the elastic joint')

%% Utility functions
function d = frankadistance(q,dq,pdes)
g = 9.81;
pf = frankaDirKin(q);
pzf = pf(3);
vf = frankaVel(q, dq);
vzf = vf(3);
tFly = (1/g)*(vzf + sqrt(vzf^2 + 2*g*(pzf-pdes(3))));
xLand = pf(1) + vf(1).*tFly;
yLand = pf(2) + vf(2).*tFly;
d = sqrt(xLand.*xLand + yLand.*yLand);
end

function tFly = computeFlightTime(pzr, vzr, pzdes)
    g = 9.81;
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

function robotAnimation(robot,traj,pdes,limits)
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
rMax = 5;
xarea = rMax*cos(th);
yarea = rMax*sin(th);
rspot = 0.3;
spotx = rspot*cos(th) + pdes(1);
spoty = rspot*sin(th) + pdes(2);
spotz = 0.01*ones(length(spotx),1) + repmat(pdes(3),length(spotx),1);

for i = 1:6:size(traj,2)-1
    cla
    xlim(limits(1,:))
    ylim(limits(2,:))
    zlim(limits(3,:))
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
    [ballx,bally,ballz] = createBall([pdes;0.04]);
%     s = surf(ballx,bally,ballz);
%     s.EdgeColor = 'b'; 
%     s.FaceColor = 'b'; 
    plot3(spotx,spoty,spotz,'g','linewidth',3);
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
    xlim(limits(1,:))
    ylim(limits(2,:))
    zlim(limits(3,:))
    p = frankaDirKin(traj(1:8,end)) + vf*t(k) + 0.5*g0*t(k)^2;
    if p(3) <= 0.04
        p(3) = 0.04;
    end
    pVec(:,k) = p;
    [ballx,bally,ballz] = createBall(p);
    hold on
    s = surf(ballx,bally,ballz);
    s.EdgeColor = 'r'; % red ball
    s.FaceColor = 'r'; % red ball
    plot3(pVec(1,1:k),pVec(2,1:k),pVec(3,1:k),'r--','linewidth',1)
    a = area(xarea,yarea);
    a.LineWidth = 3;
    a.EdgeColor = [0.98,0.98,0.98];
    a.FaceColor = [0.98,0.98,0.98];
    show(robot,traj(1:8,end)');
    [ballx,bally,ballz] = createBall([pdes;0.04]);
%     s = surf(ballx,bally,ballz);
%     s.EdgeColor = 'b'; 
%     s.FaceColor = 'b'; 
    plot3(spotx,spoty,spotz,'g','linewidth',3);
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
