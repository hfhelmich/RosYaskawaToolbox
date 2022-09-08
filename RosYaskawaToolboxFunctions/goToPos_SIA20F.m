function out = goToPos_SIA20F(time, q)
%   GOTPOS_SIA20F(TIME, Q) will send the Yaskawa to a single waypoint in a
%   given time. This function was copied from the URx_ROS class.
%
%   Inputs:
%       time -  integer for seconds
%       q    -  7x1 array
%
%   L. DeVries & H. Helmich, 29Jul22, USNA

PosPub = rospublisher('/ysk/joint_path_command');
PosSub = rossubscriber('/ysk/joint_states');

pause(1)
% Create joint state command (need only do once)
msg = rosmessage(PosPub); % creates message of the correct message type
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message
m0 = receive(PosSub); % grab initial joint states message

msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(1).Positions = m0.Position;
msg.Points(1).Velocities = zeros(7,1);
% m0.Position

msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(2).Positions = q;
msg.Points(2).Velocities = zeros(7,1);
msg.Points(2).TimeFromStart.Sec = time;

send(PosPub,msg)

out = 'sent pos message';
clear PosPub PosSub

end