%%  SCRIPT_RealTime_KeyPressXZ
%   Moves the robot in XZ plane according to inputs from arrows on
%   keyboard
%
%
%
%   Harrison Helmich; 5 April 2022
%
%   ToDo:   - find a way to clear publisher or quick reset when script gets
%           finicky
%           - 

%% Set Up
% Set Up Robot
% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
robot.DataFormat = 'row';
robot.Gravity = [0, 0, -9.81];

% Set Up Home Structure
% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);
T_home = getTransform(robot, config, "tool0");
T_Rot = T_home(1:3,1:3);
T_Rot = T_Rot*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)];
T_home(1:3, 1:3) = T_Rot;

% Set Up Inverse Kinematics Solver with Parameters
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [.1, .1, .1, 10, 1, 10];
initial = config;

% Show Home Position
figure(1); clf
show(robot, config);
axis auto;
view([60,10]);

pause(0.2);
close;

%% 
rosinit
pause(0.5);

%% ROS
js_pub = rospublisher('/joint_trajectory_MATLAB', 'IsLatching', false);

enable_SIA20F

% Build message
msg = rosmessage(js_pub); % creates message of the correct message type
% names of SIA20F joints in ROS
jointNames = {'joint_s','joint_l','joint_e','joint_u',...
    'joint_r','joint_b','joint_t'}; 
msg.JointNames = jointNames;
% Create Joint Traj part of structure
msg.Points = rosmessage('trajectory_msgs/JointTrajectoryPoint');

t_split = 0.5;

% Joint angle and velocity arrays
numJoints = numel(config);

%% Create figure, axes, etc 
% (you need the object handles for keypress to work)
startPos = [-.1; -.9; .55];

fig = figure;
axs = axes('Parent',fig);
plt = plot(axs, startPos(1), startPos(3), '*m');

xlim(axs, [-.35, .15]);     % xlim(axs, [-.25, .05]);
ylim(axs, [.3, .8]);        % ylim(axs, [.4, .7]);
xlabel('X Axis (m)');
ylabel('Z Axis (m)');

%%  Move to starting position
%   
[q, qd] = siaTaskToJoint(config, weights, ik, T_home, startPos, [0; 0; 0]);
config = q;

check = siaCheckLimits(q, 0, qd, 0);

if check == 1
    msg.Points.Positions = q;
    msg.Points.Velocities = qd;
    msg.Points.TimeFromStart.Sec = 5;
    
    % Send message
    send(js_pub, msg);
    
    fprintf("\nWaypoint \t%d\n",1);
else
    error("Invalid message components.");
end

%% Set the window key press function for the figure
set(fig,'WindowKeyPressFcn',@(src,cbd)moveDotSiaPlanar ...
    (src, cbd, axs, plt, js_pub, msg, config, ik));

 %% Bring the figure to the foreground so the function will work
% -> NOTE: The figure window must be selected for the keypress function to
% run. If you select another window, you will see that the outputs stop
% streaming to the command window.
figure(fig);
