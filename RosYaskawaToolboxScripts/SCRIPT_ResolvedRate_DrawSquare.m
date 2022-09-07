%% SCRIPT_ResolvedRate_DrawCircle
% Draw a circle in the x/z plane using:
%   (1) the UR3 manipulator world-referenced Jacobian
%   (2) the UR3 manipulator body-fixed Jacobian
%
%   M. Kutzer, 23Feb2022, USNA

clear
close
clc

%% Enable/disable use of acceleration limits
% NOTE: Ignoring acceleration limits avoids overshoot issues *but* the
%       resultant trajectory can violate safe acceleration bounds for
%       joints.
useAccLimits = true;

%% Enable/disable use of "lookahead" proportional gain
useLookAhead = true;

%% Define the model of robot (this can differ from the simulation)
urMod = 'UR3e';

%% Initialize UR3 simulation (this requires the URSimulationToolbox)
simMod = 'UR3';
sim = URsim;
sim.Initialize(simMod);
sim.Home; 

% Hide intermittent frames
% (Keep only the base frame and end-effector frames)
for i = 1:6
    hideTriad(sim.(sprintf('hFrame%d',i)));
end
hideTriad(sim.hFrameT);
set(sim.Figure,'Color',[1 1 1],...
    'Units','Inches','Position',[0.25,0.65,6.27,5.40]);

% Make the simulation transparent
for i = 0:6
    set(sim.(sprintf('pLink%d',i)),'FaceAlpha',0.5);
end

%% Define anonymous forward kinematics function
fkin = @(q)UR_fkin(urMod,q);

%% Define Forward Kinematics & Jacobians
% Define forward kinematics symbolically
syms q1 q2 q3 q4 q5 q6
q = [q1; q2; q3; q4; q5; q6];
H_e2o_sym = fkin(q);

% Derive function for body-fixed Jacobian matrix
% Note: J_e is currently an "anonymous function". We can have
%       calculateJacobian.m save a Jacobian function to file if we specify
%       a filename.
J_e = calculateJacobian(q,H_e2o_sym,...
    'Reference','Body','Order','RotationTranslation');

%% Define Joint Limitations
[q_lims,dq_lims,ddq_lims] = UR_jlims(urMod);

%% Get the current configuration of the simulation
q_Now = sim.Joints;

%%
n = 40;    % Number of discrete samples of the function
% corners
tl = [200, 200, 500];
bl = [200, 200, 300];
br = [-200, 200, 300];
tr = [-200, 200, 500];
corners = [tl; bl; br; tr; tl];

square_f = zeros(40, 3);
square_df = zeros(40, 3);
square_ddf = zeros(40, 3);
for i = 1:4
    xo = corners(i,:);
    xf = corners(i + 1,:);
    
    for j = 1:10
        if i == 1
            % top left to bottom left. change Z
            xEdge = linspace(xo(3), xf(3), 10);
            square_f(j,:) = [xo(1) xo(2) xEdge(j)];
            
            square_df(j,:) = [0 0 -1/20*(j-5).^2 + 1.25];
            square_ddf(j,:) = [0 0 -1/10*(j-5)];
        elseif i == 2
            % bottom left to bottom right. change X
            xEdge = linspace(xo(1), xf(1), 10);
            square_f(j+10,:) = [xEdge(j) xo(2) xo(3)];
            
            square_df(j+10,:) = [-1/20*(j-5).^2 + 1.25 0 0];
            square_ddf(j+10,:) = [-1/10*(j-5) 0 0];
        elseif i == 3
            % bottom right to top right. change Z
            xEdge = linspace(xo(3), xf(3), 10);
            square_f(j+20,:) = [xo(1) xo(2) xEdge(j)];
            
            square_df(j+20,:) = [0 0 -1/20*(j-5).^2 + 1.25];
            square_ddf(j+20,:) = [0 0 -1/10*(j-5)];
        elseif i == 4
            % top right to top left. change X
            xEdge = linspace(xo(1), xf(1), 10);
            square_f(j+30,:) = [xEdge(j) xo(2) xo(3)];
            
            square_df(j+30,:) = [-1/20*(j-5).^2 + 1.25 0 0];
            square_ddf(j+30,:) = [-1/10*(j-5) 0 0];
        end
    end
end

square_df(1,:) = [0 0 0];
sqaure_ddf(1,:) = [0 0 0];
square_df(end,:) = [0 0 0];
sqaure_ddf(end,:) = [0 0 0];

square_f = square_f';
square_df = square_df';
square_ddf = square_ddf';

% Plot evolution in TASK space
figure;
% evaluates position piecewise at each value t
plot3(square_f(1,:),square_f(2,:),square_f(3,:)); % x y z
grid on;

%% Define path
% r = 100;    % Radius of circle
% % -> Position
% f = @(s)[r*cos(s); 300*ones(size(s)) + 100; r*sin(s) + 425];
% % -> "Velocity" (in quotes because f is not parameterized in time)
% df = @(s)[-r*sin(s); zeros(size(s)); r*cos(s)];
% % -> "Acceleration" (in quotes because f is not parameterized in time)
% ddf = @(s)[-r*cos(s); zeros(size(s)); -r*sin(s)];

%% Draw desired path
nViaPnts = n;    % Number of discrete samples of the function
% s_all = linspace(0,2*pi,nViaPnts); % Discrete values for parameterization
for i = 1:nViaPnts
    % Current path parameter% clear dqdt
% for mm = 1:1:size(qs,2)
%     pp = spline(t, qs(:,mm));   % cubic spline
%     p_der = fnder(pp,1);
%     dqdt(:,mm) = ppval(p_der,t);
% end
% 
% % dqdt = xxd; % velocity piecewise evaluated at t from earlier
% % qs = qs';
% figure(5);
% plot(t, dqdt, 'b');

    % s = s_all(viaPnt);
    % Define desired position
    d_Des2o = square_f(:,i);
    % Define desired orientation
    x_hat = square_df(:,i)./norm(square_df(:,i));
    z_hat = [0; 1; 0];
    y_hat = cross(z_hat,x_hat);
    % Define end-effector orientation
    R_eDes2o = [x_hat,y_hat,z_hat];
    % Define desired end-effector pose
    H_eDes2o_all{i} = [R_eDes2o, d_Des2o; 0,0,0,1];

    % Visualize designated via points along the path
    h_eDes2o(i) = triad('Parent',sim.hFrame0,'Matrix',H_eDes2o_all{i},...
        'Scale',30,'LineWidth',0.5);
end
drawnow

%% Numerically solve inverse kinematics to move to initial pose
q_Now = numericIkin(fkin,J_e,H_eDes2o_all{1},q_Now);

%% Initialize joint velocity
dq_Now = zeros(6,1);

%% Move arm simulation to initial configuration
sim.Joints = q_Now;
drawnow

% define hardware params
ur.BlockingTime = 0.5;
ur.JointAcc = 2;
% move real robot to initial config
ur.Joints = q_Now;
% ur.ServoJ(q_Now);

%% Setup visualization plots
plt.k_eDes2eNow = plot3(sim.hFrameE,nan,nan,nan,'c','LineWidth',1);
plt.d_eDes2eNow = plot3(sim.hFrameE,nan,nan,nan,'m','LineWidth',1);
plt.d_eNow2o = plot3(sim.hFrame0,nan,nan,nan,'k','LineWidth',2);

%% Loop through each desired point and move the arm to the desired point

% Define variable to collect joint position ("inverse kinematic") data
traj.t   = [];
traj.q   = [];
traj.dq  = [];
traj.ddq = [];
traj.v   = [];

dt = 0.01;                      % Define desired time step
delta_q_min = deg2rad(0.05);    % User-defined stop condition
errT_switch = 10;               % Switch condition to change via points (mm)
errR_switch = 0.1;              % Switch condition to change via points (rad)

if useLookAhead
    % Define scaling/gain terms for weighting task errors when calculating 
    % gain to proportionally weight current and future via points
    % NOTE: Smaller values favor the future via point, larger values favor
    %       the desired (current) via point
    kR = 10; % Proportional weight for task rotation error
    kT = 0.2; % Proportional weight for task translation error
end

k = 10; % Gain used for increasing speed 

% Loop through each via point defined along the path
for i = 1:nViaPnts
    
    % Define desired end-effector pose
    H_eDes2o = H_eDes2o_all{i};

    if useLookAhead
        % Define *future* desired end-effector pose
        if i < nViaPnts
            H_eFut2o = H_eDes2o_all{i+1};
        else
            % Last via point, desired and future are the same
            H_eFut2o = H_eDes2o_all{i};
        end
    end
    % Reset all path frames to default
    setTriad(h_eDes2o,'LineWidth',0.5,'Scale',30);
    % Highlight current via point (desired end-effector pose)
    setTriad(h_eDes2o(i),'LineWidth',2,'Scale',40);

    % Move the arm to the desired configuration
    while true
        % Get current end-effector pose
        H_eNow2o = fkin(q_Now);

        % Calculate desired change in end-effector pose
        H_eDes2eNow = invSE(H_eNow2o)*H_eDes2o;

        % Calculate \Delta{\vec{v}}^e (relative to current end-effector frame)
        R_eDes2eNow = H_eDes2eNow(1:3,1:3);
        d_eDes2eNow = H_eDes2eNow(1:3,4);
        delta_k_eDes2eNow = vee( logSO(R_eDes2eNow),'fast' );
        delta_d_eDes2eNow = d_eDes2eNow;
        delta_v_eDes2eNow = [delta_k_eDes2eNow; delta_d_eDes2eNow];

        % Visualize input task velocity
        set(plt.k_eDes2eNow,...
            'XData',rad2deg( [0,delta_k_eDes2eNow(1)] ),...
            'YData',rad2deg( [0,delta_k_eDes2eNow(2)] ),...
            'ZData',rad2deg( [0,delta_k_eDes2eNow(3)] ));
        set(plt.d_eDes2eNow,...
            'XData',[0,delta_d_eDes2eNow(1)],...
            'YData',[0,delta_d_eDes2eNow(2)],...
            'ZData',[0,delta_d_eDes2eNow(3)]);
        appendLine(plt.d_eNow2o,H_eNow2o(1:3,4));

        % Calculate current task configuration error
        errR = norm(delta_k_eDes2eNow);
        errT = norm(delta_d_eDes2eNow);
        
        
        if useLookAhead
            % ---- BEGIN LOOK AHEAD VIA POINT -----------------------------

            % Calculate desired change in end-effector pose
            H_eFut2eNow = invSE(H_eNow2o)*H_eFut2o;

            % Calculate \Delta{\vec{v}}^e (relative to current end-effector frame)
            R_eFut2eNow = H_eFut2eNow(1:3,1:3);
            d_eFut2eNow = H_eFut2eNow(1:3,4);
            delta_k_eFut2eNow = vee( logSO(R_eFut2eNow),'fast' );
            delta_d_eFut2eNow = d_eFut2eNow;
            delta_v_eFut2eNow = [delta_k_eFut2eNow; delta_d_eFut2eNow];

            % Calculate gains to proportionally transition between current and
            % future waypoints
            k_Future  = exp(-( kR*(errR-errR_switch) + kT*(errT-errT_switch)));
            k_Current = 1 - k_Future;

            % Calculate delta_q using a combo of the desired waypoint and 
            % future waypoint
            delta_q_Des = pinv( J_e(q_Now) ) * ...
                (k_Current.*delta_v_eDes2eNow + k_Future.*delta_v_eFut2eNow);

            % ---- END LOOK AHEAD VIA POINT -------------------------------
        else
            % Calculate delta_q using a combo of the desired waypoint only
            delta_q_Des = pinv( J_e(q_Now) ) * delta_v_eDes2eNow;
        end
        

        % Define unscaled desired velocity
        dq_Des = k*delta_q_Des;

        % Bound desired velocty using velocity limits
        dq_Des = limitVector(dq_Des,dq_lims);
        
        % ---- BEGIN APPLY ACCELERATION LIMITS ----------------------------
        if useAccLimits
            % THIS METHOD IS NOT YET IMPLEMENTED
        end
        % ---- END APPLY ACCELERATION LIMITS ------------------------------

        % Check to switch via points OR for end-condition
        switch i
            case nViaPnts
                % We are trying to achieve the final via point
                if norm( dq_Des,"inf" ) < delta_q_min
                    break
                end
            otherwise
                % We are working on an intermittent via point
                if errT < errT_switch && errR < errR_switch
                    fprintf(2,'SWITCH VIA POINT\n');
                    break
                end
        end

        % Display desired velocity ----------------------------------------
        fprintf('Desired velocity: ')
        fprintf('[')
        fprintf('%.4f ',dq_Des);
        fprintf('], errT: %f, errR: %f\n',errT,errR);
        % -----------------------------------------------------------------

        % Calculate updated q and update dq
        ddq_Now = (dq_Des - dq_Now)./dt;
        dq_Now = dq_Des;
        q_Now = q_Now + dq_Des*dt;

        % Collect joint configuration and time data -----------------------
        % -> Update time
        if isempty(traj.t)
            traj.t(:,end+1) = 0;
        else
            traj.t(:,end+1) = traj.t(end) + dt;
        end
        % -> Update joint information
        traj.q(:,end+1) = q_Now;
        traj.dq(:,end+1) = dq_Now;
        traj.ddq(:,end+1) = ddq_Now;
        % -> Update task state
        traj.v(:,end+1) = delta_v_eDes2eNow;
        % -----------------------------------------------------------------

        % Move the simulated robot
        sim.Joints = q_Now;
        drawnow;
        
        % Move real robot
        % ur.SpeedJ(dq_Now);    % takes velocity param
        ur.ServoJ(q_Now);       % takes position param
        
    end
end

%% Plot trajectory data
% -> Task trajectory vs time
fig = figure('Name','Task Trajectory');
axs(1) = subplot(2,1,1,'Parent',fig);
hold(axs(1),'on');
xlabel(axs(1),'Time (s)');
ylabel(axs(1),'Task Orientation (rad)');
for i = 1:3
    plot(axs(1),traj.t,traj.v(i,:) );
end
lgnd = legend(axs(1),...
    '$k_1$','$k_2$','$k_3$',...
    'Interpreter','latex');
axs(2) = subplot(2,1,2,'Parent',fig);
hold(axs(2),'on');
xlabel(axs(2),'Time (s)');
ylabel(axs(2),'Task Position (mm)');
for i = 4:size(traj.v,1)
    plot(axs(2),traj.t,traj.v(i,:) );
end
lgnd = legend(axs(2),...
    '$d_1$','$d_2$','$d_3$',...
    'Interpreter','latex');

% -> Joint position vs time
fig = figure('Name','Joint Position Trajectory');
axs = axes('Parent',fig);
hold(axs,'on');
xlabel(axs,'Time (s)');
ylabel(axs,'Joint Angle (deg)');
for i = 1:size(traj.q,1)
    plot(axs,traj.t,rad2deg(traj.q(i,:)) );
end
lgnd = legend(axs,...
    '$\theta_1$','$\theta_2$','$\theta_3$',...
    '$\theta_4$','$\theta_5$','$\theta_6$',...
    'Interpreter','latex');

% -> Joint velocity vs time
fig = figure('Name','Joint Velocity Trajectory');
axs = axes('Parent',fig);
hold(axs,'on');
xlabel(axs,'Time (s)');
ylabel(axs,'Joint Velocity (deg/s)');
for i = 1:size(traj.q,1)
    plot(axs,traj.t,rad2deg(traj.dq(i,:)) );
end
lgnd = legend(axs,...
    '$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$',...
    '$\dot{\theta}_4$','$\dot{\theta}_5$','$\dot{\theta}_6$',...
    'Interpreter','latex');

% -> Joint acceleration vs time
fig = figure('Name','Joint Acceleration Trajectory');
axs = axes('Parent',fig);
hold(axs,'on');
xlabel(axs,'Time (s)');
ylabel(axs,'Joint Acceleration (deg/s^2)');
for i = 1:size(traj.q,1)
    plot(axs,traj.t,rad2deg(traj.ddq(i,:)) );
end
lgnd = legend(axs,...
    '$\ddot{\theta}_1$','$\ddot{\theta}_2$','$\ddot{\theta}_3$',...
    '$\ddot{\theta}_4$','$\ddot{\theta}_5$','$\ddot{\theta}_6$',...
    'Interpreter','latex');
