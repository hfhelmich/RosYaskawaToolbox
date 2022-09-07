function H_e2o = fkin_SIA20f(q)
% FKIN_SIA20F calculates the forward kinematics for the SIA20F manipulator.
%   H_e2o = FKIN_SIA20F(q)
%
%   Input(s)
%       q     - 7-element array containing joint values or symbolic 
%               variables. Values must be specified in radians.
%
%   Output(s)
%       H_e2o - 4x4 array element of SE(3) representing the end-effector
%               pose (position and orientation) relative to the robot base 
%               frame. Linear units are specified in meters. 
%
%   This function was derived using rigidBodyTree2fkin.
%
%   This function requires the "Transformation Toolbox" to run.
%
%   See also rigidBodyTree2fkin
%
%   M. Kutzer, 29Jun2021, USNA

%% Check input(s)
narginchk(1,1);
if numel(q) ~= 7
    error('Joint state must be defined as a 7-element vector.');
end

% Make q 7x1
q = reshape(q,7,[]);

%% Define joint limits
% Joint limits in degrees
%   Note - These limits are specified in the SIA20F documentation. Joint
%          limits recovered from the SIA20F URDF file are within:
%               +0.0053 deg for negative values
%               -0.0053 deg for positive values
jlims = ...
    [-180, 180;...
     -110, 110;... % Shoulder "elbow"
     -170, 170;...
     -130, 130;... % Elbow
     -180, 180;...
     -110, 110;... % Wrist "elbow"
     -180, 180];
 
% Joint limits in radians
jlims = deg2rad(jlims);

%% Check joint limits
if strcmpi( class(q), 'sym' ) 
    % Symbolic value or values of q
    % TODO - consider using tools from zeroFPError to check fixed values 
    %        contained within q against joint limits.
    warning('Not checking joint limits for symbolic q.');
else
    binN = q < jlims(:,1); % Check for values below the negative joint limit
    binP = q > jlims(:,2); % Check for values above the positive joint limit
    bin = binN | binP;
    if nnz(bin) > 0
        str = '';
        for i = 1:numel(bin)
            if bin(i)
                str = sprintf('%s\n\tJoint %d is %.5f which is outside of the [%.5f,%.5f] limit.',...
                    str,i,q(i),jlims(i,1),jlims(i,2));
            end
        end
        warning(str);
    end
end

%% Calculate forward kinematics
H_e2o = ...
    Tz(0.41000)*Rz( q(1) )*Ry( q(2) )*Tz(0.49000)*...
    Rz( q(3) )*Ry( -q(4) )*Tz(0.42000)*Rz( -q(5) )*...
    Ry( -q(6) )*Tz(0.18000)*Rz( -q(7) )*Rz( pi );


