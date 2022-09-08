function out = goHome_SIA20F(time2Home)
%   GOHOME_SIA20F(TIME2HOME) moves the Yaskawa SIA20F manipulator to its
%   home position with in a certain time interval, as desired by the user.
%
%   L. DeVries, 1Mar20, USNA

HomePub = rospublisher('/ysk/joint_path_command');
HomeSub = rossubscriber('/ysk/joint_states');

pause(1)
% Create joint state command (need only do once)
msg = rosmessage(HomePub); % creates message of the correct message type
jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
msg.JointNames = jointNames; % append names to message
m0 = receive(HomeSub); % grab initial joint states message

msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(1).Positions = m0.Position;
msg.Points(1).Velocities = zeros(7,1);
% m0.Position

msg.Points(2) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
msg.Points(2).Positions = zeros(7,1);
msg.Points(2).Velocities = zeros(7,1);
msg.Points(2).TimeFromStart.Sec = time2Home;

send(HomePub,msg)

out = 'sent home message';
clear HomePub HomeSub

end