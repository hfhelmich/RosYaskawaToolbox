%%
rosinit


%% Create publisher to command joint states (only do once)
pub = rospublisher('/joint_trajectory_MATLAB','IsLatching',false);

%% create trajectory

clear pos
% original motion
pos(1,:) = zeros(1,7)
pos(end+1,:) = [1.0;0.0;0.0;1.0;0.0;1.0;0.0]'
pos(end+1,:) = [0.75; -0.75; 0.0; 0.8; 0.75; 0.75; 0.0]'

% extra motion Donny/Mike wanted
pos(end+1,:) = zeros(1,7);
pos(end+1,:) = [0    0 0    0     0    pi/2 0];
pos(end+1,:) = [0    0 0    0    -pi/2 pi/2  0];
pos(end+1,:) = [0    0 0    0    -pi/2 pi/2 -pi];
pos(end+1,:) = [0    0 0    pi/2 -pi/2 pi/2 -pi];
pos(end+1,:) = [0    0 pi/2 pi/2 -pi/2 pi/2 -pi];
pos(end+1,:) = [pi/2 0 0    pi/2 -pi/2 pi/2 -pi];
pos(end+1,:) = zeros(1,7);
pos(end+1,:) = [0  pi/4 0  pi/4  -pi/2  pi/2  0];
pos(end+1,:) = [0 -pi/4 0 -pi/4  -pi/2  pi/2  0];

timeBetween = [0 3 2 4 1 1.5 4 4 3 3 5 2 3]
% t = [0.0;3.0;5.0;8+1;8+2;8+3.5;8+8;8+12;8+15;8+18;8+24;8+26;8+29]
t = cumsum(timeBetween)';


size(t)
size(pos)

%% compile message


clear msg
msg = rosmessage(pub); % creates message of the correct message type
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message


for ii = 1:1:size(pos,1)
    msg.Points(ii) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(ii).Positions = pos(ii,:)';
    msg.Points(ii).Velocities = zeros(7,1);
    msg.Points(ii).TimeFromStart = rosduration(t(ii));
    ii 
end


%%
enable_SIA20F

%% send message
send(pub,msg)
