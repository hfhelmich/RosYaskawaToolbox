%% SCRIPT_Kinematics_SIA20
% Explore the URDF kinematics prescribed by the "sia20.urdf"
%
% M. Kutzer, 21Jun2021, USNA
clear all
close all
clc

%% Import URDF model
% Establish robot rigid body tree from ROS urdf and mesh stl's
% "Official Motoman Release"
robot = importrobot(fullfile(pwd,'urdf','sia20.urdf'));
% Original GitHub version [DOESN'T DUCKING WORK]
%{
cd('urdf_files_old');
robot = importrobot(fullfile('urdf','sia20.urdf'));
cd('..');
%}

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

%% Show robot details
showdetails(robot)

%% Recover forward kinematics
[strs,funcs,links,jlims,jhome] = rigidBodyTree2fkin(robot);

%% Recover actual forward kinematics
fkin_str = strs{2};
fkin_func = funcs{2};
fkin_jlims = jlims{2};

%% Compare kinematics
if exist('h_o2a','var')
    delete(h_o2a);
end
h_o2a = triad('Parent',axs,'Matrix',eye(4),'Scale',0.25,'LineWidth',2);

% Define joint configuration
% q = zeros(7,1);
% q(3) = pi/6;
q = 2*pi*rand(7,1) - pi;

% Enforce joint limits
bin_n = q < fkin_jlims(:,1);
q(bin_n) = fkin_jlims(bin_n);
bin_p = q > fkin_jlims(:,2);
q(bin_p) = fkin_jlims(bin_p);

% Double check joint limits
for i = 1:size(q,1)
    if q(i) < fkin_jlims(i,1)
        fprintf(2,'q(%d) below limit: %.6f < %.6f\n',i,q(i),fkin_jlims(i,1));
    end
    if q(i) > fkin_jlims(i,2)
        fprintf(2,'q(%d) above limit: %.6f > %.6f\n',i,q(i),fkin_jlims(i,2));
    end
end

% Set configuration
Q = homeConfiguration(robot);
for i = 1:numel(Q)
    Q(i).JointPosition = q(i);
end
% Show robot in configuration
show(robot,Q,'Parent',axs,'PreservePlot',false);

% Test forward kinematics 
% - From rigid body tree
H_rb2o = getTransform(robot,Q,'tool0');
% - From returned function handle
H_hk2o = fkin_func(q); 
% - From function created using forward kinematic string
H_fk2o = fkin_SIA20f(q);
% - Test DH table
DHtable = DH_SIA20f(q);
H_dh2o  = DHtableToFkin(DHtable);

% Visualize results
h_rb2o = triad('Parent',h_o2a,'Matrix',H_rb2o,'Scale',0.25,'LineWidth',2,...
    'AxisLabels',{'x_{rb}','y_{rb}','z_{rb}'});
h_hk2o = triad('Parent',h_o2a,'Matrix',H_hk2o,'Scale',0.35,'LineWidth',2,...
    'AxisLabels',{'x_{hk}','y_{hk}','z_{hk}'});
h_fk2o = triad('Parent',h_o2a,'Matrix',H_fk2o,'Scale',0.45,'LineWidth',2,...
    'AxisLabels',{'x_{fk}','y_{fk}','z_{fk}'});
h_dh2o = triad('Parent',h_o2a,'Matrix',H_dh2o,'Scale',0.55,'LineWidth',2,...
    'AxisLabels',{'x_{dh}','y_{dh}','z_{dh}'});

% Hide patch
kids = get(axs,'Children');
set(kids(1:numel(robot.BodyNames)),'Visible','off');

return

%%
% -------------------------------------------------------------------------
% WORK IN PROGRESS
% This is an attempt at automatically building a DH table given a specified
% set of frame assignments.
%   NOTE: The following must be true:
%       (1) z-directions for joints need to be defined (+/- directions are
%           acceptable, but +/- must be applied to joint variable)
%       (2) Joint positions must be preserved only if they are not parallel
%           with a z-translation
%       (3) Base frame must be held constant
%       (4) End-effector frame must be held constant
% -------------------------------------------------------------------------
%% Isolate transforms
%tforms = kinematicStr2transforms(fkin_str);

%% Find candidate DH rows
% This doesn't work!
n = numel( tforms );
i0 = 1;
i1 = 1;
q = sym('q_%d',[1,size(jlims,1)]);
kStrs = {};
while true
    tforms_i = tforms(i0:i1);
    kStr_i = transformsStr2kinematicStr(tforms_i);
    H = eval(kStr_i);
    tst = recoverDH(H);
    if isempty(tst)
        % Save previous transform as DH row candidate
        tforms_i = tforms(i0:i1-1);
        kStrs{end+1} = transformsStr2kinematicStr(tforms_i);
        % Update indices
        i0 = i1;
        i1 = i1;
    else
        i1 = i1+1;
    end
    
    if i1 > numel(tforms)
        break
    end
end

%% Recover transforms
%{
for i = 1:numel(robot.Bodies)
    H_j2p{i} = robot.Bodies{i}.Joint.JointToParentTransform;
    H_c2j{i} = robot.Bodies{i}.Joint.ChildToJointTransform;
    
    fprintf('Body %d - "%s"\n',i,robot.BodyNames{i});
    fprintf('Joint Type: "%s", Joint Axis: [%6.2f,%6.2f,%6.2f]\n',...
        robot.Bodies{i}.Joint.Type,robot.Bodies{i}.Joint.JointAxis);
    fprintf('Joint Limits: [%6.2f,%6.2f] deg\n',...
        rad2deg(robot.Bodies{i}.Joint.PositionLimits));
    
    %{
    % Recover "rpy" and "xyz"
    rpy_j2p = rotm2eul(H_j2p{i}(1:3,1:3),'XYZ');
    xyz_j2p = H_j2p(1:3,4).';
    rpy_c2j = rotm2eul(H_c2j{i}(1:3,1:3),'XYZ');
    xyz_c2j = H_c2j(1:3,4).';
    %}
        
    fprintf('\tJoint relative to Parent:\n');
    mm = size(H_j2p{i},1);
    nn = size(H_j2p{i},2);
    for ii = 1:mm
        fprintf('\t\t[');
        for jj = 1:nn
            fprintf('%11.8f',H_j2p{i}(ii,jj));
            if jj ~= mm
                fprintf(', ');
            end
        end
        fprintf(']\n');
    end
    
    
    fprintf('\tChild relative to Joint:\n');
    mm = size(H_c2j{i},1);
    nn = size(H_c2j{i},2);
    for ii = 1:mm
        fprintf('\t\t[');
        for jj = 1:nn
            fprintf('%11.8f',H_c2j{i}(ii,jj));
            if jj ~= mm
                fprintf(', ');
            end
        end
        fprintf(']\n');
    end
end
%}