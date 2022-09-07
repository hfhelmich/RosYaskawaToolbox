%% SCRIPT_executeRose3
%
%
%
%
% Harrison Helmich 21 Mar 2022

clear;
close;
clc;

%% Define path in task space - Rose shape
r = 0.075;    % Radius will scale the size of the rose
% -> Position
f = @(s)[r*(2*sin(3*s)*cos(s)); -0.9*ones(size(s)); r*2*sin(3*s)*sin(s) + 0.85];
% -> "Velocity" (in quotes because f is not parameterized in time)
df = @(s)[r*(2*sin(3*s) + 2*cos(4*s)); zeros(size(s)); r*(4*sin(4*s) - 2*sin(2*s))];
% -> "Acceleration" (in quotes because f is not parameterized in time)
ddf = @(s)[r*(-4*sin(2*s) + 4*sin(4*s)); zeros(size(s)); r*(-4*cos(2*s) - 4*cos(4*s))];

n = 500;
tF = 30;                   % seconds
t = linspace(0,tF, n);
theta = 2*pi/tF*t;

%% Draw desired path in task space

X = zeros(3, n);
Xd = zeros(3, n);
Xdd = zeros(3, n);

for i = 1:n
    s = theta(i);
    % evaluate each pp func at s
    X(:,i)   = f(s);
    Xd(:,i)  = df(s);
    Xdd(:,i) = ddf(s);
end

% Plot task trajectory in 3D
% Shows the shape of the desired path
figure(1);
plot3(X(1,:),X(2,:),X(3,:)); % x y z
grid on;

%% Set Up
close;

% Set Up Robot
% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd, filesep, 'urdf', filesep, 'sia20.urdf'));
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% Set Up Home Structure
% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);
T_home = getTransform(robot, config, "tool0");
T_Rot = T_home(1:3,1:3);
% Change this here????
T_Rot = T_Rot*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)];
T_home(1:3, 1:3) = T_Rot;
% Show Home Position
figure(3); clf
show(robot, config);
axis auto;
view([60,10]);

% Set Up Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
% Weights
%   first three - weights on error in pose
%   second three - weights on error in task orientation xyz
weights = [0.5, 0.5, 0.5, 10, 10, 10];
initial = config;

%% Convert to joint space, check validity
close;
% Convert for joint angles and velocities
[q, qd] = siaTaskToJoint(config, weights, ik, T_home, X, Xd);
% Check for validity (with added tolerance)
qCheck = siaCheckLimits(q, 0.001, qd, 0.0001);

% Simulation visualization
% Robot moving at 8 different positions in trajectory
% Joints should look alright for each movement
figure(4);
set(gcf,'Visible','on');
for i = 1:4
    ax = show(robot, q(120*i - 119,:));
    ax.CameraPositionMode = 'auto';
    hold on;
    pause(0.5);
end

plot3(X(1,:), X(2,:), X(3,:), 'b', 'LineWidth', 2);
axis auto;
view([60,10]);
grid('minor');

figure(5);
p_q = plot(t, q, 'b');
title('Joint Angles for Rose Trajectory');
xlabel('time (s)');
ylabel('joint angles (rad)');

figure(7);
p_qd = plot(t, qd, 'b');
title('Joint Velocities for Rose Trajectory');
xlabel('time (s)');
ylabel('joint velocities (rad/s)');


%% Downsample
ts = timeseries([q qd], s_all);         
tf = 40;                                % how long robot will run
ds_rate = 0.1;                         
t_stream = 0:ds_rate:tf;        

tsout = resample(ts, t_stream);
q_ds = tsout.Data(:, 1:7);             % save smaller array of joint ang
dq_ds = tsout.Data(:, 8:end);         % save smaller array of joint vel

% plot downsampled/resampled joint space trajectories and velocities 
figure(6); clf
subplot(2,1,1)
plot(s_all, q, t_stream, q_ds, 'o') % CHECK JOINT ANGLE LIMITS
ylabel('joint angle (rad)');
xlabel('time (s)');
subplot(2,1,2)
plot(s_all, qd, t_stream, dq_ds, 'o') % CHECK JOINT RATE LIMITS
ylabel('joint vel (rad/s)');
xlabel('time (s)');

%%  ROS
close;
clc;

% Connect to master
rosinit
pause(0.5);
% Create publisher
js_pub = rospublisher('/joint_trajectory_MATLAB', 'IsLatching', false);
pause(0.5);
% Enable - Check for green light on pendant
enable_SIA20F

% Build message
msg = rosmessage(js_pub); % creates message of the correct message type
% names of SIA20F joints in ROS
jointNames = {'joint_s','joint_l','joint_e','joint_u',...
    'joint_r','joint_b','joint_t'}; 
msg.JointNames = jointNames; % append names to message

for ii = 1:1:numel(t) % make sure array bounds match
    msg.Points(ii) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(ii).Positions = q(ii,:);     % joint angles
    msg.Points(ii).Velocities = qd(ii,:);  % joint rates
    msg.Points(ii).TimeFromStart = rosduration(t(ii));
    ii
end

%% Send the message
send(js_pub, msg);

%% Shutdown
rosshutdown



