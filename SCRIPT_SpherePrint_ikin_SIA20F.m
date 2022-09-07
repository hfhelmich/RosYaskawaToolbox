%% SCRIPT_SpherePrint_ikin_SIA20F
% Animates elbow-up and elbow-down spherical print solutions for the SIA20F
% manipulator.
%
%   M. Kutzer, 14Jul2021, USNA

clear all
close all
clc

%% Import URDF model
% Establish robot rigid body tree from ROS urdf and mesh stl's
% "Official Motoman Release"
robot = importrobot(fullfile(pwd,'urdf','sia20.urdf'));
robot.DataFormat = 'column'

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

%% Recover robot joint limits
[~,~,~,qlims,~] = rigidBodyTree2fkin(robot);
qlims = qlims{2};

%% Define pose to place nozzle
% Define joint configuration
q = deg2rad( [0; 90; 0; 90; 0; -90; 0]);

% Show robot in configuration
show(robot,q,'Parent',axs,'PreservePlot',false);

view(axs,[-30,10]);

%% Define sphere & stem length
r = 0.15;   % Define sphere radius (m)
ds = 0.20;  % Define stem length (m)
sfit.Center = zeros(3,1);
sfit.Radius = r;
ptch = patch( patchSphere(sfit,1000),'EdgeColor','None','FaceColor','b',...
    'FaceAlpha',0.5 );
h_StoB = triad('Parent',axs);
set(ptch,'Parent',h_StoB);

%% Place Sphere
H_EtoB = fkin_SIA20f(q);
H_StoB = H_EtoB * Tz(ds);
set(h_StoB,'Matrix',H_StoB);
hideTriad(h_StoB);

%% Define nozzle position
x_StoB = H_StoB(1:3,4);
x_NtoB = x_StoB + [0; 0; r];


%% Plot nozzle position
plt = plot3(axs,x_NtoB(1),x_NtoB(2),x_NtoB(3),'*m');

%% Test inverse kinematics (default settings)

for el = linspace(0,pi,100)
    for az = linspace(-pi,pi,10)
        AzEl = [az,el];
        q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB);
        
        % Set configuration
        if ~isempty(q)
            % Show robot in configuration
            show(robot,q,'Parent',axs,'PreservePlot',false);
            
            % Move sphere
            H_EtoB = fkin_SIA20f(q);
            H_StoB = H_EtoB * Tz(ds);
            set(h_StoB,'Matrix',H_StoB);
            
            drawnow
        end
    end
end


%% Test inverse kinematics (elbow-up settings with joint limits)
% Adjust figure & axes parameters
set(axs,'Projection','Orthographic');
view(axs,[0,0]);

az = 0;
for el = linspace(0,pi,500)
    %for az = linspace(-pi,pi,10)
    AzEl = [az,el];
    q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,qlims,'elbow-up');
    
    ttl = title(axs,sprintf('AzEl = [%8.3f,%8.3f]',rad2deg(AzEl)));
    % Set configuration
    if ~isempty(q)
        Q = homeConfiguration(robot);
        for i = 1:numel(Q)
            Q(i).JointPosition = q(i);
        end
        % Show robot in configuration
        show(robot,Q,'Parent',axs,'PreservePlot',false);
        
        % Move sphere
        H_EtoB = fkin_SIA20f(q);
        H_StoB = H_EtoB * Tz(ds);
        set(h_StoB,'Matrix',H_StoB);
        set(ttl,'Color','k');
        drawnow
    else
        set(ttl,'Color','r');
    end
    %end
end

%% Test inverse kinematics (elbow-down with joint limits)
%{
for el = linspace(0,pi,100)
    for az = linspace(-pi,pi,10)
        AzEl = [az,el];
        q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,qlims,'elbow-down');
        
        % Set configuration
        if ~isempty(q)
            Q = homeConfiguration(robot);
            for i = 1:numel(Q)
                Q(i).JointPosition = q(i);
            end
            % Show robot in configuration
            show(robot,Q,'Parent',axs,'PreservePlot',false);
            
            % Move sphere
            H_EtoB = fkin_SIA20f(q);
            H_StoB = H_EtoB * Tz(ds);
            set(h_StoB,'Matrix',H_StoB);
            
            drawnow
        end
    end
end
%}