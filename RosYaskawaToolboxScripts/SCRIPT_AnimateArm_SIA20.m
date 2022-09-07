clear all
clc

% Establish robot rigid body tree from ROS urdf and mesh stl's
%robot = importrobot(fullfile(pwd,filesep,'urdf',filesep,'sia20.urdf'));
% "fullfile.m" adds filesep automatically
robot = importrobot(fullfile(pwd,'urdf','sia20.urdf'));

% create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% create array of angles
angs = linspace(-pi/4,pi/4,50);

% iterate over all angles in angs
for mm = 1:length(angs)
    config(2).JointPosition = angs(mm); % update joint configuration
    config(4).JointPosition = angs(end-mm+1);
    config(6).JointPosition = angs(mm);
    show(robot,config,'Visuals','on','PreservePlot',false); % update the image
    drawnow % draw the image
end


%% Allow user to set configurations

iviz = interactiveRigidBodyTree(robot);
ax = gca;
addConfiguration(iviz)
for ii = 1:4
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
numSamples = 50*size(iviz.StoredConfigurations, 2) + 1;
[q,qd,~, tvec] = trapveltraj(iviz.StoredConfigurations,numSamples,'EndTime',15);

%% % animate user-defined trajectory
iviz.ShowMarker = false;
showFigure(iviz)
rateCtrlObj = rateControl(numSamples/(max(tvec) + tvec(2)));
for i = 1:numSamples
    iviz.Configuration = q(:,i);
    waitfor(rateCtrlObj);
end



