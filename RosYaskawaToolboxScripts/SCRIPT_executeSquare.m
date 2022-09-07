%%  SCRIPT_executeSquare
%   From "Set Up Robot" to "Visualization," the sections are copied
%   from the SCRIPT_SimulateSquareTrajectory
%
%
%
%   Harrison Helmich, 15 Feb 2022
%
%

clear;
close;
clc;

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
% Determine Location of Corners of the Square
bl = [-.25 -0.9 .4];    % meters
length = .3;
br = bl(1) + length;
br = [br, bl(2), bl(3)];
tr = br(3) + length;
tr = [br(1), br(2), tr];
tl = tr(1) - length;
tl = [tl, tr(2), tr(3)];

corners = [tl; bl; br; tr; tl];
%Number of Points on each line segment
n = 10;

% Time Setup
Hz = 40;

%% Different approach to trajectory
close;

% define important points/times in velocity profile
t0 = 0; % seconds
t1 = 3; % seconds
t2 = 6; % seconds
t3 = 9;
tf = 12; % seconds
Tp = tf; % period to complete 1 square

%% Build trajectory


X_star = corners.';             % 3x5
Xd_star = zeros(size(X_star));  % 3x5
Xdd_star = zeros(size(X_star)); % 3x5
% adjust time intervals to change velocity/accel
t_star = [t0, t1, t2, t3, tf];

% for i = 1:size(X_star,1)
% ppAlt(i) = fitpp(t_star,X_star(i,:),t_star,Xd_star(i,:),t_star,Xdd_star(i,:));
% end
% ppAlt = ppArray2pp(ppAlt);

% fit a 3D pp to the corners, breakpoints for each continuous graph
ppAlt = fitpp(t_star,X_star,t_star,Xd_star,t_star,Xdd_star);
% Find DERIVATIVES of piecewise polynomial
ppAltd  = diffpp(ppAlt);    % Piecewise polynomial for velocity
ppAltdd = diffpp(ppAltd);   % Piecewise polynomial for acceleration

breaks = ppAlt.breaks; % breaks it up based on t_star
% creates n time spaces, starting with breaks(1)
t = linspace(breaks(1), breaks(end), 1000);
% h = plotpp(ppX,t,2);

% Combine traj pp into one structure
ppALL(1) = ppAlt;
ppALL(2) = ppAltd;
ppALL(3) = ppAltdd;

fig = figure;
colors = 'rgb';
for i = 1:3
    axs(i) = subplot(3,1,i,'Parent',fig);
    hold(axs(i),'on');
    
    % evaluate position, then velocity, then accel at t
    y = ppval(ppALL(i),t);
    
    % graph x, then y, then z
    for j = 1:size(y,1)
        plot(t,y(j,:),colors(j));
        legend('x', 'y', 'z')
    end
    
end

% Plot evolution in TASK space
figure;
% evaluates position piecewise at each value t
xx = ppval(ppAlt,t);
plot3(xx(1,:),xx(2,:),xx(3,:)); % x y z
xlabel('X Axis (m)');
ylabel('Y Axis (m)');
zlabel('Z Axis (m)');
title('Square Trajectory in Task Space');
grid on;
hold on;

% Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10];
initial = config;

% ikin for each waypoint
numJoints = numel(config);
numWaypoints = size(xx,2);      % using X_star

% q's - desired joint angles for trajectory
qs = zeros(numWaypoints, numJoints);

for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = xx(:,i);     % using X_star
    [q_sol, q_info] = ik(endEffector, T_des, weights, config);
   

    qs(i, :) = q_sol(1:numJoints);
    
    config = q_sol;
    
    i/numWaypoints
end

figure;
plot(t, qs);
ylabel('angular position (rad)');
xlabel('time (s)');
title('Joint Angles for a Square Trajectory');
legend('joint S','joint L','joint E','joint U','joint R','joint B','joint T');

%% Visualization
% will draw path in 3D space. Shows validity
figure; 
set(gcf,'Visible','on');
for i = 1:4
    ax = show(robot, qs(250*i - 249,:));
    ax.CameraPositionMode = 'auto';
    hold on;
end

% Waypoints Plottf = 10; % seconds
plot3(X_star(1,:), X_star(2,:), X_star(3,:), 'b', 'LineWidth', 2);
axis auto;
view([60,10]);
grid('minor');
title('Simulated Joint Angles for Square Trajectory');

%%  END : copy from SCRIPT_SimulateSquareTrajectory
%% Convert task velocities to joint velocities
% JACOBIAN

dqdt = zeros(size(qs,1), 7);

% convert task velocities to joint velocities with EE jacobian
for i = 1:1:size(qs,1)
    dqdt(i,1:7) = pinv(jacobianSiaEE(qs(i,:)))*[xxd(:,i); 0; 0; 0];
    i
end

figure;
plot(t, dqdt);
ylabel('angular velocity (rad/s)');
xlabel('time (s)');
title('Joint Velocities for a Square Trajectory');
legend('joint S','joint L','joint E','joint U','joint R','joint B','joint T');

% calculate joint accel for graph purposes
ddqdt = zeros(size(qs,1), 7);
for i = 1:1:(size(qs,1)-1)
    ddqdt(i,:) = (dqdt(i+1,:) - dqdt(i,:))./0.0400;
end

figure;
plot(t, ddqdt, 'b');
ylabel('angular accel (rad/s/s)');
xlabel('time (s)');

%% Modify data at slower rate (less samples)
ts = timeseries([qs dqdt], t);     % clear dqdt
tf = 12;                                % how long robot will run
ds_rate = 1;                         
t_stream = 0:ds_rate:tf;        

tsout = resample(ts, t_stream);
qs_ds = tsout.Data(:, 1:7);             % save smaller array of joint ang
dqdt_ds = tsout.Data(:, 8:end);         % save smaller array of joint vel

% plot downsampled/resampled joint space trajectories and velocities 
figure(6); clf
subplot(2,1,1)
plot(t, qs, t_stream, qs_ds, 'o') % CHECK JOINT ANGLE LIMITS
ylabel('joint angle (rad)');
xlabel('time (s)');
subplot(2,1,2)
plot(t, dqdt, t_stream, dqdt_ds, 'o') % CHECK JOINT RATE LIMITS
ylabel('joint vel (rad/s)');
xlabel('time (s)');


%% Check joint angle & velocities
siaCheckLimits(qs_ds, 0.05, dqdt_ds, 0.0005);

%% Connect to master
rosinit

%% Create publisher to command joint states (only once)
% rospublisher(topic, name, value)
js_pub = rospublisher('/joint_trajectory_MATLAB', 'IsLatching', false);

%% Create a subscriber to listen to joint states topic (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',{@write_callback,'test.txt'});
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback

%% enable robot
enable_SIA20F

%% Compose message comprising trajectory
% see ROS Documentation for message typeslength(40)

msg1 = rosmessage(js_pub); % creates message of the correct message type
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
msg1.JointNames = jointNames; % append names to message

% assign values to message components
for ii = 1:1:numel(t_stream) % make sure array bounds match  
    msg1.Points(ii) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg1.Points(ii).Positions = qs_ds(ii,:);     % joint angles
    msg1.Points(ii).Velocities = dqdt_ds(ii,:);  % joint rates
    msg1.Points(ii).TimeFromStart = rosduration(5 + t_stream(ii));
    ii
end

%% send to robot
send(js_pub,msg1)

q_results = zeros(13, numJoints);

pause(5.5)
for i = 1:13
    q_results(i, 1:7) = js_sub.LatestMessage.Position;
    
    pause(1);
    i    
end    

figure(8);
plot(t, qs, t_stream, q_results, '*');
title('Actual and Desired Joint Angles');
ylabel('joint angle (rad)');
xlabel('time (s)');

%% close ros connection
% rosshutdown

