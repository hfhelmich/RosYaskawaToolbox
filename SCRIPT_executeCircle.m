%% Clear Data

clc;
clear all

%% Run Code
% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];
endEffector = "tool0";

% create config structure with home joint angle and angle Move it along trajectorynames
config = homeConfiguration(robot);
T_home = getTransform(robot, config, endEffector);
T_Rot = T_home(1:3,1:3);
T_Rot = T_Rot*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)];
T_home(1:3, 1:3) = T_Rot;

%Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10];

initial = config;

%Waypoints
center = [0.1 -.75 .85];
radius = .25;

% define important points/times in velocity profile
t0 = 0; % seconds
t1 = 1; % seconds
t2 = 8; % seconds
tf = 10; % seconds
Tp = tf; % period to complete 1 circle

% define task space cruise speed of trapezoidal trajectory
vc = 4*pi*radius/(Tp+t2-t1-t0-t0^2/(t1-t0)); % m/s (desired task space speed as it traverses the circle)


% Defines Times and Steps 
% ***(use high fidelity here, can be downsampled later in code)
Hz = 200;

t = transpose(0:1/Hz:Tp);

% specify velocity profile
v = vc/(t1-t0)*(t-t0);
v(t>=t1) = vc;

% define task space cruise speed of trapezoidal trajectory
vc = 4*pi*radius/(Tp+t2-t1-t0-t0^2/(t1-t0)); % m/s (desired task space speed as it traverses the circle)

% specify phase along circle
th0 = -vc*t0^2/(2*radius*(t1-t0));
th1 = th0 - vc/(2*radius)*(t1+t0);
theta = vc/(2*radius*(t1-t0))*(t-t0).^2 + th0;
theta(t>=t1) = vc/radius*t(t>=t1) + th1;
theta(t>=t2) = -vc/(2*radius*(tf-t2))*(t(t>=t2)-t2).^2 + vc/radius*t(t>=t2) + th1;

% specify trajectory in 3D space
points = center + radius*[cos(theta) 0*ones(size(theta)) sin(theta)];

%Inverse Kinematics for Each Waypoint
numJoints = numel(config);
numWaypoints = size(points,1);
qs = zeros(numWaypoints, numJoints);

% iterate through waypoints
for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)';
    [q_sol, q_info] = ik(endEffector, T_des, weights, config);
    qs(i, :) = q_sol(1:numJoints);
    config = q_sol;
    i/numWaypoints
end

%% Visualization
figure; 
set(gcf,'Visible','on');

ax = show(robot, qs(1,:));
ax.CameraPositionMode = 'auto';
hold on;

% Waypoints Plottf = 10; % seconds
plot3(points(:,1),points(:,2),points(:,3),'b','LineWidth',2);
axis auto;
view([60,10]);
grid('minor');

%% calculate joint velocities using spline interpolation
clear dqdt
for mm = 1:1:size(qs,2)
    pp = spline(t, qs(:,mm));   % cubic spline
    p_der = fnder(pp,1);
    dqdt(:,mm) = ppval(p_der,t);
end

% checking if both are same trajectory..

% plot result of interpolation
figure(5); clf
subplot(2,1,1)
plot(t,qs)
subplot(2,1,2)
plot(t,dqdt)
hold on
plot(t,[diff(qs)/mean(diff(t)) ; zeros(1,7)],'--') % compare to simple differencing

% downsample trajectory at a chosen rate (frequency)
% creaste a timeseries object
ts = timeseries([qs dqdt],t);

ds_rate = 0.05; % downsampled update period
t_stream = 0:ds_rate:tf;

%% Modify data set at slower rate, less samples
tsout = resample(ts,t_stream);

% down-sampled joint angles
qs_ds = tsout.Data(:,1:7);      % create new joint angles
dqdt_ds = tsout.Data(:,8:end);  % create new joint velocities (angular)


% plot downsampled/resampled joint space trajectories and velocities 
figure(6); clf
subplot(2,1,1)
plot(t,qs,t_stream,qs_ds,'o')
% CHECK JOINT ANGLE LIMITS
subplot(2,1,2)
plot(t,dqdt,t_stream,dqdt_ds,'o')
% CHECK JOINT RATE LIMITS

% trajectory_mat = [t_stream' qs_ds dqdt_ds]
% writematrix(trajector1) call rosservice call /robot_enabley_mat,'trajectory.csv','FileType','text')

% %% Animation
% framesPerSecond = 1/ds_rate;
% r = robotics.Rate(framesPerSecond);nly do once)
% view(0,0)
% for i = 1:1:numWaypoints
%     show(robot, qs_ds(i,:),'PreservePlot',false);
% %     drawnow;
%     waitfor(r);
% end
% 

save('Circle_Traj.mat','qs','dqdt','t')
save('Circle_Traj_execute20hz.mat', 'qs_ds', 'dqdt_ds', 't_stream')

%% Initialize ROS with remote host (only do once)
rosinit % connects MATLAB to ROS network

%% Create publisher to command joint states (only do once)
pub = rospublisher("/joint_trajectory_MATLAB",'IsLatching',false)

%% Create a subscriber to listen to joint states topic (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',{@write_callback,%%'test.txt'});
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when no%%t using callback

%% enable robot
enable_SIA20F

%% Compose message comprising trajectory
% see ROS Documentation for message types
msg = rosmessage(pub); % creates message of the correct message type
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message

for ii = 1:1:length(t_stream)
    msg.Points(ii) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(ii).Positions = qs_ds(ii,:);     % joint angles
    msg.Points(ii).Velocities = dqdt_ds(ii,:);  % joint rates
    msg.Points(ii).TimeFromStart = rosduration(t_stream(ii));
    ii
end

%% send to robot
send(pub,msg)

%% close ros connection
rosshutdown




