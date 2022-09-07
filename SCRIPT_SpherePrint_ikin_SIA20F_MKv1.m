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
%robot.DataFormat = 'column'

% Create config structure with home joint angle and angle names
config = homeConfiguration(robot);

% Show the robot in the home joint configuration
axs = show(robot,config,'Visuals','on','PreservePlot',false);

% Adjust figure & axes parameters
set(axs,'Projection','Orthographic');
view(axs,[0,0]);
hold(axs,'on');

fig = get(axs,'Parent');
%set(fig,'Units','Inches','Position',[1,1,12.67,5.4]);
set(fig,'Units','Normalized','Position',[0,0,1,1]);
drawnow;

%% Recover robot joint limits
[~,~,~,qlims,~] = rigidBodyTree2fkin(robot);
qlims = qlims{2};

%% Define sphere & stem length
r = 0.15;   % Define sphere radius (m)
ds = 0.20;  % Define stem length (m)
%r_in = 2:0.25:9;

%% Find candidate nozzle positions for el = 0
H_EtoB_z = fkin_SIA20f(zeros(7,1));
H_EtoB_x = fkin_SIA20f([0; pi/2; 0; 0; 0; 0; 0]);
x_ALL = linspace(0,H_EtoB_x(1,4),50);
z_ALL = linspace(0,H_EtoB_z(3,4),50);

% Update axes
% set(axs,'PositionConstraint','OuterPosition')
% xlim(axs,[min(x_ALL),max(x_ALL)+1.1*r]);
% zlim(axs,[min(z_ALL),max(z_ALL)+1.1*r]);
% InSet = get(axs, 'TightInset');
% set(axs, 'Position', [InSet(1:2), 1-InSet(1)-InSet(3), 1-InSet(2)-InSet(4)])

if ~exist('x_NtoB_el0','var')
    warning off
    x_NtoB_el0 = [];
    plt_el0 = plot(axs,0,0,'.k');
    el = 0;
    for x = x_ALL
        for z = z_ALL
            AzEl = [0,el];
            q = SpherePrint_ikin_SIA20F(AzEl,ds,r,[x; 0; z],qlims,'elbow-up');
            if ~isempty(q)
                x_NtoB_el0(:,end+1) = [x; 0; z];
                set(plt_el0,...
                    'XData',x_NtoB_el0(1,:),...
                    'YData',x_NtoB_el0(2,:),...
                    'ZData',x_NtoB_el0(3,:));
                drawnow
            end
        end
    end
    warning on
end

%% Find candidate nozzle positions for el = [some value]
H_EtoB_z = fkin_SIA20f(zeros(7,1));
H_EtoB_x = fkin_SIA20f([0; pi/2; 0; 0; 0; 0; 0]);
x_ALL = linspace(0,H_EtoB_x(1,4),50);
z_ALL = linspace(0,H_EtoB_z(3,4),50);

warning off
iter = 0;
overlap = false;
for el = linspace(pi,3*pi/4,50)
    title(axs,sprintf('El = %.6f',el));
    drawnow
    iter = iter+1;
    %color = rand(1,3);
    color = 'b';
    x_NtoB_el = [];
    plt_el = plot(axs,0,0,'.','Color',color);
    
    for x = x_ALL
        for z = z_ALL
            AzEl = [0,el];
            q = SpherePrint_ikin_SIA20F(AzEl,ds,r,[x; 0; z],qlims,'elbow-up');
            if ~isempty(q)
                x_NtoB_el(:,end+1) = [x; 0; z];
                
                % Check if there is any overlap
                if nnz( ismember(x_NtoB_el0.',x_NtoB_el.','rows') ) > 0
                    overlap = true;
                end
            end
        end
    end
    if overlap
        set(plt_el,...
            'XData',x_NtoB_el(1,:),...
            'YData',x_NtoB_el(2,:),...
            'ZData',x_NtoB_el(3,:));
        break
    end
end
warning on

%% Define peal elevation
el_star = el;

%% Define nozzle position
binA = ismember(x_NtoB_el0.',x_NtoB_el.','rows');
x_NtoB = x_NtoB_el0(:,binA);

%% Plot sphere
sfit.Center = zeros(3,1);
sfit.Radius = r;
ptch = patch( patchSphere(sfit,1000),'EdgeColor','None','FaceColor','b',...
    'FaceAlpha',0.5 );
h_StoB = triad('Parent',axs);
set(ptch,'Parent',h_StoB);

%% Place Sphere
H_StoB = Tx(x_NtoB(1))*Ty(x_NtoB(2))*Tz(x_NtoB(3))*Tz(-r);
set(h_StoB,'Matrix',H_StoB);
hideTriad(h_StoB);

%% Plot nozzle position
plt = plot3(axs,x_NtoB(1),x_NtoB(2),x_NtoB(3),'*m');

% Nozzle frame matching orientation of base frame
H_NtoB = Tx(x_NtoB(1))*Ty(x_NtoB(2))*Tz(x_NtoB(3));

%% Test inverse kinematics (elbow-up settings with joint limits)
% Adjust figure & axes parameters
set(axs,'Projection','Orthographic');
view(axs,[0,0]);

% Visualize deposition line
plt_NtoS_in = plot(h_StoB,0,0,'.g');    % Inside of joint limits
plt_NtoS_out = plot(h_StoB,0,0,'.r');   % Outside of joint limits
x_NtoS_in = [];
x_NtoS_out = [];

Q = homeConfiguration(robot);
az = 0;
for el = linspace(0,el_star,500)
    %for az = linspace(-pi,pi,10)
    AzEl = [az,el];
    q_in = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,qlims,'elbow-up');
    q_out = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,[],'elbow-up');
    
    ttl = title(axs,sprintf('AzEl = [%8.3f,%8.3f]',rad2deg(AzEl)));
    % Set configuration
    if ~isempty(q_in)
        % Update robot joint position for visualization
        for i = 1:numel(Q)
            Q(i).JointPosition = q_in(i);
        end
        % Show robot in configuration
        show(robot,Q,'Parent',axs,'PreservePlot',false);
        
        % Move sphere
        H_EtoB = fkin_SIA20f(q_in);
        H_StoB = H_EtoB * Tz(ds);
        set(h_StoB,'Matrix',H_StoB);
        set(ttl,'Color','k');
        
        % Show deposition
        H_NtoS = invSE(H_StoB)*H_NtoB;
        x_NtoS_in(:,end+1) = H_NtoS(1:3,4);
        set(plt_NtoS_in,...
            'XData',x_NtoS_in(1,:),...
            'YData',x_NtoS_in(2,:),...
            'ZData',x_NtoS_in(3,:));
        drawnow
    elseif ~isempty(q_out)
        set(ttl,'Color','m');
        % Update robot joint position for visualization
        for i = 1:numel(Q)
            Q(i).JointPosition = q_out(i);
        end
        % Show robot in configuration
        show(robot,Q,'Parent',axs,'PreservePlot',false);
        
        % Move sphere
        H_EtoB = fkin_SIA20f(q_out);
        H_StoB = H_EtoB * Tz(ds);
        set(h_StoB,'Matrix',H_StoB);
        set(ttl,'Color','k');
        
        % Show deposition
        H_NtoS = invSE(H_StoB)*H_NtoB;
        x_NtoS_out(:,end+1) = H_NtoS(1:3,4);
        set(plt_NtoS_out,...
            'XData',x_NtoS_out(1,:),...
            'YData',x_NtoS_out(2,:),...
            'ZData',x_NtoS_out(3,:));
        drawnow
    else
        set(ttl,'Color','r');
    end
    %end
end
