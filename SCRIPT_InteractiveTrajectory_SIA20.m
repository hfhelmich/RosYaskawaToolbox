clear all
clc


%% Initialize ROS (with remote host (only do once))
% hostIP = '10.10.130.150'; % replace with ROS Master IP or localhost depending on setup configuration
% hostPort = '11311'; % host port, not certain this is the default
% setenv('ROS_MASTER_URI',['http://' hostIP ':' hostPort]); % set ROS master uri
% myIP = '10.10.130.148';
% setenv('ROS_IP',myIP); % my own IP address
% %setenv('ROS_HOSTNAME',hostIP)
% 
% rosinit(hostIP)

% corrected ROS initialization (if remote)
% rosinit('10.10.130.150', 'NodeHost', '10.10.130.148');


rosinit


%% enable the robot
response = enable_SIA20F

%% Create publisher to command joint states (only do once)
pub = rospublisher('/joint_trajectory_MATLAB')

%% Create a subscriber to listen to joint states topic (do only once)
% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',@JS_Callback);
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback



%% Create MATLAB manipulator model from ROS URDF and meshes

% Establish robot rigid body tree from ROS urdf and mesh stl's
robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));

% create config structure with home joint angle and angle names
config = homeConfiguration(robot);




%% Allow user to set configurations
m0 = receive(js_sub); % grab initial joint states message
angs = m0.Position;
iviz = interactiveRigidBodyTree(robot,'Configuration',angs);
ax = gca;
addConfiguration(iviz)
for ii = 1:3
    if ii==1
        title('Drag Robot to desired configuration: Press enter when ready')
    else
        title(['Move robot to configuration '  num2str(ii) '. Press enter when ready'])
    end
    pause
    addConfiguration(iviz)
end
title('Trajectory complete')

%% create smooth trajectory using trapezoidal velocity interpolation
% numSamples = 10*size(iviz.StoredConfigurations, 2) + 1;
% [q,qd,~, tvec] = trapveltraj(iviz.StoredConfigurations,numSamples,'EndTime',15);


%% Package trajectory into ROS message

clear msg
numPoints = size(iviz.StoredConfigurations,2)
dt = 4;
msg = packagePointsList(iviz.StoredConfigurations,zeros(7,numPoints),transpose(0:dt:(numPoints-1)*dt))




%% send to robot

send(pub,msg)





