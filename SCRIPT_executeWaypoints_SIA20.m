%% Drive Yaskawa SIA20 hardware (FS100 Controller) or ROS Simulation from MATLAB using ROS Toolbox
% SCRIPT_move_SIA20_MATLAB.m demonstrates how to connect to a remote ROS
% Master at a known IP address then subscribe and publish to topics on the
% ROS network. Additional code is included to plot joint states of the
% robot in real time.
%
% Requirements:
%   ROS Toolbox
%
% L. DeVries, USNA, WRC, 6/15/2021

clear all
clc
%% Initialize ROS with remote host (only do once)

% % USE If the machine running the roscore is not running the instance of MATLAB, connect to it remotely
% hostIP = '10.0.0.64'; % replace with ROS Master IP or localhost depending on setup configuration
% hostPort = '11311'; % host port, not certain this is the default
% setenv('ROS_MASTER_URI',['http://' hostIP ':' hostPort]); % set ROS master uri
% myIP = '10.0.0.118';
% setenv('ROS_IP',myIP); % my own IP address
% setenv('ROS_HOSTNAME',hostIP)
% rosinit(hostIP)

% If machine running MATLAB is also running the roscore, connect to it this way
rosinit


%% Create publisher to command joint states (only do once)
pub = rospublisher('/joint_trajectory_MATLAB');


%% Create a subscriber to listen to joint states topic and any other topics of interest (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information

% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',@JointState_Callback);
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback
% status_sub = rossubscriber('/robot_status') % subscribe to robot status


%% Compose trajectory in joint space for robot to follow

% load joint angles, velocities, and time points
load('Square_Traj.mat')

% create ROS message containing trajectory
msg = packagePointsList(qs',zeros(size(qs')),t)



%% publish the message to ROS
send(pub,msg) % send message
pause(1)

status=1;
while(status==1)
    st = receive(status_sub); % grab message from status subscriber, inMotion velue goes to zero when trajectory complete
    status = st.InMotion.Val; % update value of status variable
    disp('waiting on trajectory to finish') %
    
    % Can add further code here to get joint angles and velocities and
    % monitor trajectory as it unfolds
end



%% close ros connection
rosshutdown

