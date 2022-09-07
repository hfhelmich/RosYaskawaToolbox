function moveDotKeyPressFCN(src,callbackdata,axs,plt,pub,msg,config,ik)
% MOVEDOTKEYPRESSFCN is an example key press function with additional
% inputs to be used with the "WindowKeyPressFcn" property of figures.
%
% Example Use:
%   Given a valid figure object (fig), axes object (axs) and 
%   line object (plt) already defined:
%
%       set(fig,'WindowKeyPressFcn',...
%           @(src,cbd)moveDotKeyPressFCN(src,cbd,axs,plt));
%
% M. Kutzer, USNA, 09Mar2022

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

%% Get current x/y position
X(1,:) = get(plt,'XData');
X(2,:) = get(plt,'YData');

%% Check the key that was pressed and do something
% -> We are going to assign the following:
%       'numpad1' - Move down & left
%       'numpad2' - Move down
%       'numpad3' - Move down & right
%       'numpad4' - Move left
%       'numpad5' - Stay still
%       'numpad6' - Move right
%       'numpad7' - Move up & left
%       'numpad8' - Move up
%       'numpad9' - Move up & right
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

Xout = X_new(1);    % X component for rosmessage
Yout = -0.9;        % Y component (FIXED)
Zout = X_new(2);    % Z component

T_home = [-1 0 0 Xout; 0 0 -1 Yout; 0 -1 0 Zout; 0 0 0 1];

%%

% Need to adjust config

% Convert task to joint
[q, qd] = siaTaskToJoint(config, [.1, .1, .1, 10, 1, 10],...
    ik, T_home, [Xout; Yout; Zout], [0; 0; 0]);

% Update values for next iteration
config = q;

check = siaCheckLimits(q, 0, qd, 0);

if check == 1
    msg.Points.Positions = q;
    msg.Points.Velocities = qd;
    msg.Points.TimeFromStart = rosduration(0.5);
    
    % Send message
    send(pub, msg);
    
    fprintf("\nWaypoint \tpass");
else
    error("Invalid message components.");
end







