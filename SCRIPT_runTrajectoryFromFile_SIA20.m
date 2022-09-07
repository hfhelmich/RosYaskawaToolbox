%% Drive Yaskawa SIA20 hardware (FS100 Controller) or ROS Simulation from MATLAB using ROS Action
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
% hostIP = '10.0.0.64'; % replace with ROS Master IP or localhost depending on setup configuration
% hostPort = '11311'; % host port, not certain this is the default
% setenv('ROS_MASTER_URI',['http://' hostIP ':' hostPort]); % set ROS master uri
% myIP = '10.0.0.118';
% setenv('ROS_IP',myIP); % my own IP address
% setenv('ROS_HOSTNAME',hostIP)

% rosinit(hostIP)
rosinit


%%
response = enable_SIA20F;


%% Create publisher to command a trajectory (only do once)
trajPub = rospublisher('/joint_trajectory_MATLAB');

%% Create a subscriber to listen to joint states topic (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',@JointState_Callback);
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback
% fb_sub = rossubscriber('/joint_trajectory_action/feedback');

%% Create joint state command (need only do once)

% create a carrier ros message to send trajectory
msg = rosmessage(trajPub); % creates message of the correct message type

% specify names of each joint
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
% fill joint names field of message with robot joint names
msg.JointNames = jointNames; % append names to message

% load joint trajectory file
fname = 'SAMPLE_jointTrajectories.mat';
load(fname)


% fill points field of ROS trajectory message with joint angles and
% velocities loaded from file
msg = packagePointsList(q',qd',tvec);



%%
% publish the command to ROS
send(trajPub,msg)




%% close ros connection
rosshutdown

