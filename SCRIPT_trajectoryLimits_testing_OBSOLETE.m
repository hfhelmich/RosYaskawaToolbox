clear all
clc
%%
% Establish robot rigid body tree from ROS urdf and mesh stl's
%robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
% "fullfile.m" adds filesep automatically
robot = importrobot(fullfile(pwd,'urdf','sia20.urdf'));

% create config structure with home joint angle and angle names
config = homeConfiguration(robot);


figure(1)
show(robot,config)

%% set up ros

rosinit

%%
% Create publisher to command joint states (only do once)
pub = rospublisher('/joint_path_command');
trajPub = rospublisher('/joint_trajectory_action/goal');

% Create a subscriber to listen to joint states topic (do only once)

% Third input argument is optional and specifies a callback function to
% automatically handle data as it is published to the topic. See
% JointState_Callback.m for more information
% js_sub = rossubscriber('/joint_states','sensor_msgs/JointState',{@write_callback,'test.txt'});
js_sub = rossubscriber('/joint_states','sensor_msgs/JointState'); % Syntax when not using callback

% enable service client
eclient = rossvcclient('/robot_enable_MATLAB')
% disable service client
dclient = rossvcclient('/robot_disable_MATLAB')

%% enable robot
req = rosmessage(eclient)
resp = call(eclient,req,'Timeout',3);





%%

% Create independent variable (time) array
dt = [1/1 1/2 1/10 1/20 1/30 1/50 1/100 1/150];
t_final = 3;
sub = rossubscriber('/joint_states','sensor_msgs/JointState',{@write_callback,'test.txt'});
status_sub = rossubscriber('/robot_status');
pause(1.0)
for mm = 1:1:length(dt)
    mm
    t = 0:dt(mm):t_final;
    
    % Create a set of joint trajectories over time
    amps = [pi/4 pi/10 pi/4 pi/5 pi/6 pi/6 pi/6];
    pers = 2*pi./[10 5 7 7 8 9 10];
    
    dist = pi/4;
    v_amp = dist*pi/(-2*t_final);
    
    % Positions
    j_s = 0*amps(1)*sin(pers(1)*t);
    j_l = v_amp/(pi/t_final)*(cos(pi/t_final*t)-1);%amps(2)*sin(pers(2)*t);
    j_e = 0*amps(3)*sin(pers(3)*t);
    j_u = v_amp/(pi/t_final)*(cos(pi/t_final*t)-1);%0*amps(4)*sin(pers(4)*t);
    j_r = 0*amps(5)*sin(pers(5)*t);
    j_b = 0*amps(6)*sin(pers(6)*t);
    j_t = 0*amps(7)*sin(pers(7)*t);
    
    % matrix of joint positions
    positions = [j_s' j_l' j_e' j_u' j_r' j_b' j_t'];
    
    
    % Velocities
    v_s = 0*amps(1)*pers(1)*cos(pers(1)*t);
    v_l = v_amp*sin(pi/t_final*t); %amps(2)*pers(2)*cos(pers(2)*t);
    v_e = 0*amps(3)*pers(3)*cos(pers(3)*t);
    v_u = v_amp*sin(pi/t_final*t);% 0*amps(4)*pers(4)*cos(pers(4)*t);
    v_r = 0*amps(5)*pers(5)*cos(pers(5)*t);
    v_b = 0*amps(6)*pers(6)*cos(pers(6)*t);
    v_t = 0*amps(7)*pers(7)*cos(pers(7)*t);
    
    % matrix of joint velocities
    velocities = [v_s' v_l' v_e' v_u' v_r' v_b' v_t'];
%     max(abs(velocities)*180/pi)
    
    figure(2)
    subplot(2,1,1)
    plot(t,positions*180/pi)
    subplot(2,1,2)
    plot(t,velocities*180/pi)
    
    % %% iterate over all angles in angs
    % figure(1)
    % ti = title(['Time: ' num2str(t(1)) ' sec'])
    % hold on
    % for mm = 1:length(t)
    %     config(1).JointPosition = j_s(mm);
    %     config(2).JointPosition = j_l(mm);
    %     config(3).JointPosition = j_e(mm);
    %     config(4).JointPosition = j_u(mm);
    %     config(5).JointPosition = j_r(mm);
    %     config(6).JointPosition = j_b(mm);
    %     config(7).JointPosition = j_t(mm);
    %
    %     show(robot,config,'Visuals','on','PreservePlot',false); % update the image
    %     set(ti,'String',['Time: ' num2str(t(mm)) ' sec'])
    %     drawnow % draw the image
    % end
    
    
    
    % Create joint state command (need only do once)
    clear msg
    msg = rosmessage(pub); % creates message of the correct message type
    jointNames = {'joint_s','joint_l','joint_e','joint_u','joint_r','joint_b','joint_t'}; % names of SIA20F joints in ROS
    msg.JointNames = jointNames; % append names to message
    m0 = receive(sub); % grab initial joint states message
    
    msg.Points(1) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(1).Positions = m0.Position;
    msg.Points(1).Velocities = zeros(7,1);
    msg.Points(1).TimeFromStart = rosduration(0.0);
    
    
    tmp = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    msg.Points(2:length(t)+1) = tmp;
    for ii = 1:1:length(t)
        msg.Points(ii+1).Positions = positions(ii,:);
        msg.Points(ii+1).Velocities = velocities(ii,:);
        msg.Points(ii+1).TimeFromStart = rosduration(t(ii)+dt(mm));
        %ii
    end
    
    
    % Send trajectory to robot
    
    send(pub,msg)
    pause(1)
    ss = receive(status_sub);
    while(ss.InMotion.Val==1)
        disp('waiting for motion to clear')
        ss = receive(status_sub);
    end
    
    
    % Send robot home when trajectory is complete
    
    pause(1)
    goHome_SIA20F(pub,sub)
    pause(1)
    ss = receive(status_sub);
    while(ss.InMotion.Val==1)
        disp('waiting for homing')
        ss = receive(status_sub);
    end
    
    % check to see if it homed
    m0 = receive(sub); % grab initial joint states message
    if(sum(abs(m0.Position))<0.001)
        goHome_SIA20F(pub,sub)
    end
    pause(1.0)
    ss = receive(status_sub);
    while(ss.InMotion.Val==1)
        disp('waiting for homing')
        ss = receive(status_sub);
    end
end
clear sub

