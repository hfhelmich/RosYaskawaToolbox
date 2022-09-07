function [DHtable,H_0toB] = SpherePrint_q2DH_SIA20F(q,qlims)
% SPHEREPRINT_Q2DH_SIA20F gives the spherical printing DH table for the 
% SIA20F manipulator given a joint configuration q. 
%   [DHtable,H_0toB] = SPHEREPRINT_Q2DH_SIA20F(q)
%   [DHtable,H_0toB] = SPHEREPRINT_Q2DH_SIA20F(q,qlims)
%
%   Input(s)
%       q -     7x1 array containing ordered joint configuration in 
%               radians. See constraints below.
%       qlims - 7x2 array containing joint limits in radians. If qlims is
%               not specified, joint limits are not considered
%
%           Constraints:
%               q = [q(1); q(2); 0; q(4); 0; q(5); q(6); q(7)]
%
%   Output(s)
%       DHtable - 4x4 array containing the DH table associated with planar
%                 spherical printing. Linear units are defined in meters.
%                 An empty set is returned if q is outside of the 
%                 prescribed joint limits
%       H_0toB  - 4x4 rigid body transform specifying the transformation
%                 relating the spherical printing DH frame 0 to the
%                 manufacturer base frame of the manipulator. Linear units
%                 are defined in meters. An empty set is returnd if  q is 
%                 outside of the prescribed joint limits
%
%   M. Kutzer, 14Jul2021, USNA

%% Initialize outputs
DHtable = [];
H_0toB = [];

%% Check input(s)
narginchk(1,2);
if numel(q) ~= 7
    error('Joint configuration must be specified as a 7-element vector.');
end
q = reshape(q,7,1);

% Check zero elements of q
ZERO = 1e-6;
if abs(q(3)) > ZERO
    warning('Joint 3 is non-zero. Assuming q(3) = 0.');
end
if abs(q(5)) > ZERO
    warning('Joint 5 is non-zero. Assuming q(5) = 0.');
end

if nargin > 1
    if ~ismatrix(qlims) || size(qlims,1) ~= 7 || size(qlims,2) ~= 2
        error('Joint limits must be specified as a 7x2 array.');
    end
    
    % Check limits
    tf_min = q < qlims(:,1);
    tf_max = q > qlims(:,2);
    
    if nnz(tf_min) > 0
        warning('Joint configuration contains at least one value below the lower joint limit.');
        return
    end
    
    if nnz(tf_max) > 0
        warning('Joint configuration contains at least one value above the upper joint limit.');
        return
    end
end

%% Define link lengths (meters)
l(1,1) = 0.41;
l(2,1) = 0.49;
l(3,1) = 0.42;
l(4,1) = 0.18;

%% Define DH parameters
theta(1,1) = pi/2 - q(2);
theta(2,1) = q(4);
theta(3,1) = q(6) + pi/2;
theta(4,1) = q(7);

d(1,1) = 0;
d(2,1) = 0;
d(3,1) = 0;
d(4,1) = l(4);

a(1,1) = l(2);
a(2,1) = l(3);
a(3,1) = 0;
a(4,1) = 0;

alpha(1,1) = 0;
alpha(2,1) = 0;
alpha(3,1) = pi/2;
alpha(4,1) = 0;

DHtable = [theta, d, a, alpha];

if nargout < 2
    return;
end

%% Calculate H_0toB
H_0toB = Rz(q(1))*Tz(l(1))*Rx(pi/2);