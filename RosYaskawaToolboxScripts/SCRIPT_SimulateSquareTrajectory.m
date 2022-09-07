%% Clear Data

clc;
clear all;

%% Set Up Robot
% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];
endEffector = "tool0";

%% Set Up Home Structure

% create config structure with home joint angle and angle names
config = homeConfiguration(robot);
T_home = getTransform(robot, config, endEffector);
T_Rot = T_home(1:3,1:3);
T_Rot = T_Rot*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)];
T_home(1:3, 1:3) = T_Rot;

%Show Home Position
figure(1); clf
show(robot, config);
axis auto;
view([60,10]);

%% Determine Points

%Waypoints
%center = [0.2 -.7 .8];
%radius = .1;


%Determine Location of Corners of the Square
bl = [-.25 -.7 .4];
length = .5;
br = bl(1) + length;
br = [br, bl(2), bl(3)];
tr = br(3) + length;
tr = [br(1), br(2), tr];
tl = tr(1) - length;
tl = [tl, tr(2), tr(3)];

corners = [bl; br; tr; tl; bl];
%Number of Points on each line segment
n = 10;



%% Time Setup
Hz = 40;


%% Finish Point Set Ups
[q,qd,qdd,tSamples,pp] = trapveltraj(corners',n*4,'EndTime',10/4);

points = q';


%% Plot the Task Space Joints

figure(2)
plot(tSamples, q)
legend('X','Y','Z')

%% Inverse Kinematics

%Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10]; % what is this?

initial = config;

%Inverse Kinematics for Each Waypoint
numJoints = numel(config);
numWaypoints = size(q,2);
% q's - desired joint angles for trajectory
qs = zeros(numWaypoints, numJoints);

for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:);
    [q_sol, q_info] = ik(endEffector, T_des, weights, config);
    
    qs(i, :) = q_sol(1:numJoints);
    

    config = q_sol;
end

%% See In Action

%Visualization
figure; set(gcf,'Visible','on');

ax = show(robot, qs(1,:));
ax.CameraPositionMode = 'auto';
hold on;

%Waypoints Plot
plot3(points(:,1),points(:,2),points(:,3),'b','LineWidth',2);
axis auto;
view([60,10]);
grid('minor');
hold on;

% Animation
framesPerSecond = Hz;
r = robotics.Rate(framesPerSecond);
view(0,0)
for i = 1:1:numWaypoints
    show(robot, qs(i,:),'PreservePlot',false);
    % drawnow;
    waitfor(r);
end

figure(4); clf
plot(tSamples,qs)

%%
clear dqdt
for mm = 1:1:size(qs,2)
    pp=spline(tSamples,qs(:,mm));
    p_der=fnder(pp,1);
    dqdt(:,mm) = ppval(p_der,tSamples);
end


figure(5); clf
subplot(2,1,1)
title('Joint Trajectory Spline???');
plot(tSamples,qs)
xlabel('Time')
ylabel('Positions')
subplot(2,1,2)
plot(tSamples,dqdt)
hold on
plot(tSamples,[diff(qs)/mean(diff(tSamples)) ; zeros(1,7)],'--') % compare to simple differencing
xlabel('Time')
ylabel('Velocities')

