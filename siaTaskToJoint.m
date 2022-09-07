function [q, qd] = siaTaskToJoint(config, ik, T_home, X, Xd)
%SIATASKTOJOINT converts 3D task space to 7 dim joint space for Yaskawa
% SIA20F. Needs to be maximally optimized for calculating joint space
% quickly in real time.
%
%   Inputs
%       config  - structure of robot with home joint angle and angle names
%       weights - predetermined array for ikin
%       ik      - ROS ikin object instantiated outside of method to prevent
%                 using up unnecessary RAM
%       T_Home  - base rigid body transform
%       X       - piecewise function of position  (3xN)
%       Xd      - piecewise function of velocity  (3xN)
%
%   Outputs
%       q       - joint angles for desired traj (7xN)
%       dq      - joint velocities for desired traj (7xN)
%
%   Harrison Helmich, 9 Mar 22

%   Updates:
%       9 Mar 22:   prevented reloading jacobian data within method
%                   instantiate ik object outside of method

% TODO - Ensure input arrays are correct size




% Build joint arrays
numWaypoints = size(X,2);
q = zeros(7, numWaypoints);     % Nx7 array
qd = zeros(7, numWaypoints);            % Nx7 array
weights = [.1, .1, .1, 10, 1, 10];

% Get joint angles, fill q output array
for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = X(:,i);
    % using Inverse Kinematics
    [q_sol, q_info] = ik("tool0", T_des, weights, config);
    
    % If a joint angle is found...
    if isequal(q_info.Status, 'success')
        q(:,i) = q_sol(1:7);
        config = q_sol;
    else
        error("Joint angles could not be found. Try again.");
    end
end

% Don't really care about joint velocities unless doing a large trajectory
if numWaypoints > 2
    % Get joint velocities, fill dqdt output array
    for i = 1:numWaypoints
        % Inserted task velocity config only takes translation into
        % account, no rotation --> [x; y; z; 0; 0; 0]
        % using Jacobian
        qd(:,i) = pinv(jacobianSiaEE(q(i,:)))*[Xd(:,i); 0; 0; 0];
    end
else
    qd(1:7, :) = 0;
end

end