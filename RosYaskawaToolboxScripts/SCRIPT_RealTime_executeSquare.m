%%  SCRIPT_RealTime_executeSquare
%   From "Set Up Robot" to "Visualization," the sections are copied
%   from the SCRIPT_SimulateSquareTrajectory.
%   
%   Will execute the same square trajectory from SCRIPT_executeSquare
%   but send the robot one index of the message at a time.
%
%   Harrison Helmich; 22 Mar 2022

clear;
close;
clc;

%% Set Up 
% Set Up Robot
% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];
endEffector = "tool0";

% Set Up Home Structure
% create config structure with home joint angle and angle names
config = homeConfiguration(robot);
T_home = getTransform(robot, config, endEffector);
T_Rot = T_home(1:3,1:3);
T_Rot = T_Rot*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)];
T_home(1:3, 1:3) = T_Rot;

% Show Home Position
figure(1); clf
show(robot, config);
axis auto;
view([60,10]);

% Set Up Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10];
initial = config;

pause(0.5);
close;

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

% Define important points/times in velocity profile
tf = 12;
t_side = tf/4;         % seconds, time to complete one side of square
t0 = 0;             % seconds
t1 = t_side*1;      % seconds
t2 = t_side*2;      % seconds
t3 = t_side*3;      % seconds

%% Build trajectory
X_star      = corners.';                % 3x5
Xd_star     = zeros(size(X_star));      % 3x5
Xdd_star    = zeros(size(X_star));      % 3x5
t_star = [t0, t1, t2, t3, tf];

% Fit a 3D pp to the corners, breakpoints for each continuous graph
ppAlt = fitpp(t_star,X_star,t_star,Xd_star,t_star,Xdd_star);
ppAltd  = diffpp(ppAlt);    % Piecewise polynomial for velocity
ppAltdd = diffpp(ppAltd);   % Piecewise polynomial for accelerationkutzer@usna.edu

% Number of total points
n = 12;

breaks = ppAlt.breaks;
t = linspace(breaks(1), breaks(end), n);

X = ppval(ppAlt, t);        % task pos
Xd = ppval(ppAltd, t);      % task vel
Xdd = ppval(ppAltd, t);     % task accel

figure;
plot3(X(1,:), X(2,:), X(3,:)); % x y z
grid on;

pause(0.5);
clear X_star Xd_star Xdd_star
close;

clc;

%% Connect to master
rosinit
pause(0.5);

%% ROS
% Create publisher
pub = rospublisher('/joint_trajectory_MATLAB', 'IsLatching', false);
jt_sub = rossubscriber('/joint_trajectory_MATLAB', 'trajectory_msgs/JointTrajectory');
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback
pause(0.5);

% Build message
msg = rosmessage(pub); % creates message of the correct message type
% names of SIA20F joints in ROS
jointNames = {'joint_s','joint_l','joint_e','joint_u',...
    'joint_r','joint_b','joint_t'}; 
msg.JointNames = jointNames;
% Create Joint Traj part of structure
msg.Points = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% Joint angle and velocity arrays
numJoints = numel(config);
% *** 1x7 or nx7 
q = zeros(1, numJoints);
qd = zeros(1, numJoints);
q_results = zeros(n, numJoints);

%% Enable - Check for green light on pendant
enable_SIA20F

%%  Real Time Message

for ii = 1:numel(t)
          
%     msg = rosmessage(pub); % creates message of the correct message type
%     msg.JointNames = jointNames; % append names to message 
    
    % Convert task to joint
    [q, qd] = siaTaskToJoint...
       (config, weights, ik, T_home, X(:,ii), Xd(:,ii));
    % Update values for next iteration
    config = q;
    
    check = siaCheckLimits(q, 0, qd, 0);
    
    if check == 1
        msg.Points.Positions = q;
        msg.Points.Velocities = qd;
        % Needs to be time between each angle
        % Make it fixed?
        msg.Points.TimeFromStart.Sec = 2;
        
        % Send message  
        send(pub, msg);
        
        fprintf("\nWaypoint \t%d",ii);
    else
        error("Invalid message components.");
    end
    
    if ii == 1 
        pause(10);
    end
    
   
    out1 = js_sub.LatestMessage.Position
    out2 = jt_sub.LatestMessage.Points;
    
%     while ~ismembertol(out1, q, 0.1)
%        out1 = js_sub.LatestMessage.Position
%        q
%     end   
    pause(3);
%     clear msg

    q_results(ii, 1:7) = out1;
end

%%
figure(8);
plot(t, qs, t_steam, q_results, '*') % CHECK JOINT ANGLE LIMITS
ylabel('joint angle (rad)');
xlabel('time (s)');


%% Shutdown
% rosshutdown
