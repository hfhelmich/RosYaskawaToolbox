%% SCRIPT_FrameAssignments_SIA20F
% This function creates a "side-on" view of the SIA20F to assign frames for
% spherical printing DH table derivation. 
% 
% See also SCRIPT_UR_PlanarInverseKinematics
%
% ...\GitHub\C3SurfacePrinting_Sphere\01_InverseKinematics\Universal_Robots
%
% M. Kutzer, 08Jul2021, USNA

clear all
close all
clc

%% Import URDF model
% Establish robot rigid body tree from ROS urdf and mesh stl's
% "Official Motoman Release"
robot = importrobot(fullfile(pwd,'urdf','sia20.urdf'));

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

%% Define pose for frame assignments
% Define joint configuration
q = deg2rad( [0; 45; 0; -60; 0; 30; 0]);

% Set configuration
Q = homeConfiguration(robot);
for i = 1:numel(Q)
    Q(i).JointPosition = q(i);
end
% Show robot in configuration
show(robot,Q,'Parent',axs,'PreservePlot',false);

view(axs,[0,0]);

%% Adjust figure & axes parameters
set(axs,'Projection','Orthographic');
grid(axs,'off');
fig = get(axs,'Parent');
set(fig,'Color',[1,1,1]);
set(axs,'Visible','off');
set(fig,'Units','Inches','Position',[1,1,12.67,5.4]);
axis(axs,'tight');

%% Hide unwanted objects
kids = get(axs,'Children');
type = get(kids,'Type');
% Find first line object
for i = 1:numel(type)
    if strfind(type{i},'line')
        break
    end
end
set(kids( (i):(end-2) ),'Visible','off');

%% Define sphere
sc = 1.8;
sfit.Center = zeros(3,1);
sfit.Radius = 0.1 * sc;
ptch = patch( patchSphere(sfit,1000),'EdgeColor','None','FaceColor','b',...
    'FaceAlpha',0.5 );
h_s2o = triad('Parent',axs);
set(ptch,'Parent',h_s2o);

%% Place Sphere
ds = (120/1000) * sc;   % Stem length (m)
H_e2o = fkin_SIA20f(q);
H_s2o = H_e2o * Tz(ds);
set(h_s2o,'Matrix',H_s2o);
hideTriad(h_s2o);

%% Save image
saveas(fig,'SIA20F_XZ.png','png');