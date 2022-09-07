function moveDotSiaPlanar(src,callbackdata,axs,plt,pub,msg,config,ik)
% MOVEDOTSIAPLANAR is a key press function with additional
% inputs to be used with the "WindowKeyPressFcn" property of figures. 
% Sends direct ROS messages each time the method is called.
%
% Inputs (not included in original Kutzer KeyPressFCN):
%       pub - ROS publisher to the jointTrajectoryPoint topic
%       msg - jointTrajectoryPoint message to be sent by publisher
%       config - current joint angles of robot. This is calculated once
%                from starting pos in SCRIPT
%       ik - inverse kinematics object to convert task to joint
%   
% Example Use:
%   Given a valid figure object (fig), axes object (axs) and 
%   line object (plt) already defined:
%
%       set(fig,'WindowKeyPressFcn',...
%           @(src,cbd)moveDotKeyPressFCN(src,cbd,axs,plt));
%
% Harrison Helmich; 5 April 2022
%
% Updates:
%
%

%% Parse useful inputs
% -> For this example, the source (src) represents the figure triggering or
% calling this function. It can be useful if you want to update a plot or 
% change the figure in some way.
fig = src; 

% -> The variable "callbackdata" has some  useful information stored in a
% structured array. For this example, we are only using the field named
% "Key" that corresponds to the key being pressed.
keySTR = lower(callbackdata.Key);

%% Define change in x/y
step = 0.08;       % in meters
dx = step*[1; 0];
dy = step*[0; 1];
% dz = step*[0; 0; 1];

depth = 0.08;

%% Get current x/y position
X(1,:) = get(plt,'XData');
X(2,:) = get(plt,'YData');
% X(3,:) = get(plt,'ZData');
Y = -0.9;

%% Check the key that was pressed and do something
% -> We are going to assign the following:
%       '1'             - Move down & left
%       '2'             - Move down
%       '3'             - Move down & right
%       '4'             - Move left
%       '5'             - Stay still
%       '6'             - Move right
%       '7'             - Move up & left
%       '8'             - Move up
%       '9'             - Move up & right
%       'left arrow'    - Move left     
%       'right arrow'   - Move right
%       'up arrow'      - Move up
%       'down arrow'    - Move down
%       'f'             - Move forward
%       'b'             - Move backward
%
X_new = nan(2,1);

switch keySTR
    case 'numpad1'
        % 'numpad1' - Move down & left
        X_new = X - dy - dx;
    case 'numpad2'
        % 'numpad2' - Move down
        X_new = X - dy;
    case 'numpad3'
        % 'numpad3' - Move down & right
        X_new = X - dy + dx;
    case 'numpad4'
        % 'numpad4' - Move left
        X_new = X - dx;
    case 'numpad5'
        % 'numpad5' - Stay still
        X_new = X;
    case 'numpad6'
        % 'numpad6' - Move right
        X_new = X + dx;
    case 'numpad7'
        % 'numpad7' - Move up & left
        X_new = X + dy - dx;
    case 'numpad8'
        % 'numpad8' - Move up
        X_new = X + dy;
    case 'numpad9'
        % 'numpad9' - Move up & right
        X_new = X + dy + dx;
    case 'leftarrow'
        % 'numpad4' - Move left
        X_new = X - dx;
    case 'rightarrow'
        % 'numpad6' - Move right
        X_new = X + dx;
    case 'uparrow'
        % 'numpad8' - Move up
        X_new = X + dy;
    case 'downarrow'
        % 'numpad2' - Move down
        X_new = X - dy;
    case 'f'
        % Move forward in plane
        Y = Y - depth;
        X_new = X + dy;
    case 'b'
        % Move backward in plane
        Y = Y + depth;
        X_new = X - dy;
    case 'g'
        % Move FARTHER forward in plane
        Y = Y - depth - 0.2;
        X_new = X + dy;
    case 'n'
        % Move FARTHER backward in plane
        Y = Y + depth + 0.2;
        X_new = X - dy;
    otherwise
        fprintf('Unrecognized key:\n');
        fprintf('\tThe key pressed was "%s"\n',keySTR);
        return
end

%% Check limits (we can just use the xlim/ylim)
x_lim = xlim(axs);
y_lim = ylim(axs);

if X_new(1) < x_lim(1) || X_new(1) > x_lim(2)
    fprintf(2,'Outside x-limit!\n')
    X_new(1,:) = X(1,:);
end

if X_new(2) < y_lim(1) || X_new(2) > y_lim(2)
    fprintf(2,'Outside y-limit!\n')
    X_new(2,:) = X(2,:);
end

%% Update the position of the dot
set(plt,'XData',X_new(1,:),'YData',X_new(2,:));
% set(plt, 'MarkerSize', 10*X_new(3,:));

Xout = X_new(1);    % X component for rosmessage
Yout = Y;           % Y component
Zout = X_new(2);    % Z component

T_home = [-1 0 0 Xout; 0 0 -1 Yout; 0 -1 0 Zout; 0 0 0 1];

%%

% Convert task to joint
[q, qd] = siaTaskToJoint(config, [.1, .1, .1, 10, 1, 10],...
    ik, T_home, [Xout; Yout; Zout], [0; 0; 0]);

% ToDo - update config for each call of this method after the first or does
% it matter if the same config is used?

check = siaCheckLimits(q, 0, qd, 0);

if check == 1
    msg.Points.Positions = q;
    msg.Points.Velocities = qd;
    % Robot runs around 40 Hz
    msg.Points.TimeFromStart.Sec = 1;
    
    % Send message
    send(pub, msg);
    
    fprintf("\nWaypoint \tpass");
else
    error("Invalid message components.");
end







