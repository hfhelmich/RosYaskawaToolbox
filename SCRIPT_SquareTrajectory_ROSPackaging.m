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
bl = [.2 -.7 .8];
length = .1;
br = bl(1) + length;
br = [br, bl(2), bl(3)];
tr = br(3) + length;
tr = [br(1), br(2), tr];
tl = tr(1) - length;
tl = [tl, tr(2), tr(3)];

%Number of Points on each line segment
n = 100;



%% Time Setup

t0 = 0;
tf = 10;
t = linspace(t0, tf, n*4);

Hz = 200;

%% Finish Point Set Ups

x_points_baseline = linspace(bl(1), br(1), n);
z_points_baseline = linspace(bl(3), br(3), n);
x_points_right_sideline = linspace(br(1), tr(1), n);
z_points_right_sideline = linspace(br(3), tr(3), n);
x_points_topline = linspace(tr(1), tl(1), n);
z_points_topline = linspace(tr(3), tl(3), n);
x_points_left_sideline = linspace(tl(1), bl(1), n);
z_points_left_sideline = linspace(tl(3), bl(3), n);


x_points = horzcat(x_points_baseline, x_points_right_sideline, x_points_topline, x_points_left_sideline);
z_points = horzcat(z_points_baseline, z_points_right_sideline, z_points_topline, z_points_left_sideline);
y_points = ones(size(x_points));
y_points = y_points*bl(2);

points = [x_points', y_points', z_points'];

%% Inverse Kinematics

%Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10];

initial = config;

%Inverse Kinematics for Each Waypoint
numJoints = numel(config);
numWaypoints = size(points,1);
qs = zeros(numWaypoints, numJoints);

for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)';
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

%Animation
acobian
%%
framesPerSecond = Hz;
r = robotics.Rate(framesPerSecond);
view(0,0)
for i = 1:1:numWaypoints
    show(robot, qs(i,:),'PreservePlot',false);
%     drawnow;
    waitfor(r);
end


figure(2); clf
plot(t,qs)


%%
clear dqdt
for mm = 1:1:size(qs,2)
    pp=spline(t,qs(:,mm));
    p_der=fnder(pp,1);
    dqdt(:,mm) = ppval(p_der,t);
end


figure(5); clf
subplot(2,1,1)
plot(t,qs)
subplot(2,1,2)
plot(t,dqdt)
hold on
plot(t,[diff(qs)/mean(diff(t)) ; zeros(1,7)],'--') % compare to simple differencing


% downsample trajectory at a chosen rate (frequency)
% creaste a timeseries object
ts = timeseries([qs dqdt],t)


%%
ds_rate = 0.05
t_stream = 0:ds_rate:tf;
tsout = resample(ts,t_stream);

qs_ds = tsout.Data(:,1:7);
dqdt_ds = tsout.Data(:,8:end);


figure(6); clf
subplot(2,1,1)
plot(t,qs,t_stream,qs_ds,'o')
subplot(2,1,2)
plot(t,dqdt,t_stream,dqdt_ds,'o')


%%
save('Square_Traj.mat','qs_ds','dqdt_ds','t_stream')