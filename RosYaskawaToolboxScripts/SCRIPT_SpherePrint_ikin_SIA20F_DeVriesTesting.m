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
robot.DataFormat = 'column';

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

%% Recover robot joint limits
[~,~,~,qlims,~] = rigidBodyTree2fkin(robot);
qlims = qlims{2};

%% Define pose to place nozzle
% Define joint configuration
q = deg2rad( [0; 0; 0; 0; 0; 0; 0]);

% Show robot in configuration
show(robot,q,'Parent',axs,'PreservePlot',false);

view(axs,[-30,10]);

%% Define sphere & stem length
r = 0.05;   % Define sphere radius (m)
ds = 0.10;  % Define stem length (m)
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

%% Define array of potential nozzle positions
x_NtoB = [linspace(r,1.2,10);...
          zeros(1,10);...
          0.7*ones(1,10)];



%% Plot nozzle position
plt = plot3(axs,x_NtoB(1,1),x_NtoB(2,1),x_NtoB(3,1),'*m');


%% Test inverse kinematics (elbow-up settings with joint limits)
% Adjust figure & axes parameters
set(axs,'Projection','Orthographic');
view(axs,[0,0]);

az = 0;
for mm = 1:size(x_NtoB,2)
    set(plt,'XData',x_NtoB(1,mm),'YData',x_NtoB(2,mm),'ZData',x_NtoB(3,mm))
    for el = linspace(0,pi,50)    
        AzEl = [az,el];
        q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB(:,mm),qlims,'elbow-up');
        ttl = title(axs,sprintf('AzEl = [%0.3f,%0.3f], x_{NtoB} = [%0.3f,0,%0.3f]',[rad2deg(AzEl) x_NtoB(1) x_NtoB(3)]));
        % Set configuration
        if ~isempty(q)
            % Show robot in configuration
            show(robot,q,'Parent',axs,'PreservePlot',false);
            
            % Move sphere
            H_EtoB = fkin_SIA20f(q);
            H_StoB = H_EtoB * Tz(ds);
            set(h_StoB,'Matrix',H_StoB);
            
            drawnow
        else
            set(ttl,'string',sprintf('Cannot reach! AzEl = [%8.3f,%8.3f], x_{NtoB} = [%0.3f,0,%0.3f]',[rad2deg(AzEl) x_NtoB(1) x_NtoB(3)]))
            drawnow
        end
    end
end

