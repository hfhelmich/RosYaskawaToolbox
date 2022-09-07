function q = SpherePrint_DH2q_SIA20F(DHtable,H_0toB,qlims)
% SPHEREPRINT_DH2Q_SIA20F gives the joint configuration of the SIA20F
% manipulator given the spherical print DH table and the transformation
% relating the DH frame 0 to the base frame of the manipulator.
%   q = SPHEREPRINT_DH2Q_SIA20F(DHtable)
%   q = SPHEREPRINT_DH2Q_SIA20F(DHtable,H_0toB)
%   q = SPHEREPRINT_DH2Q_SIA20F(DHtable,H_0toB,qlims)
%
%   Input(s)
%       DHtable - 4x4 array containing the DH table associated with planar
%                 spherical printing. Linear units are defined in meters.
%                 See constraints below.
%       H_0toB  - 4x4 rigid body transform specifying the transformation
%                 relating the spherical printing DH frame 0 to the
%                 manufacturer base frame of the manipulator. If no
%                 transformation is specified, q(1) is assumed 0. Linear
%                 units are defined in meters. See constraints below.
%       qlims   - 7x2 array containing joint limits in radians. If qlims
%                 is not specified, joint limits are not considered
%
%           Constraints:
%
%               DHtable = [\theta_1, d_1, a_1,    0]
%                         [\theta_2, d_2, a_2,    0]
%                         [\theta_3, d_3, a_3, pi/2]
%                         [\theta_4, d_4,   0,    0]
%
%               H_0toB  = [ cos(o), 0,  sin(o),    0]
%                         [ sin(o), 0, -cos(o),    0]
%                         [      0, 1,       0, 0.41]
%                         [      0, 0,       0,    1]
%
%   Output(s)
%       q - 7x1 array containing ordered joint configuration in radians.
%           An empty set is returned if q is outside of the prescribed
%           joint limitsSee constraints below.
%
%           Constraints:
%               q = [q(1); q(2); 0; q(4); 0; q(5); q(6); q(7)]
%
%   M. Kutzer, 14Jul2021, USNA

%% Initialize output
q = [];

%% Check input(s)
narginchk(1,3);
if ~ismatrix(DHtable) || size(DHtable,1) ~= 4 || size(DHtable,2) ~= 4
    error('DH table must be specified as a 4x4 array.');
end

if nargin > 1
    % TODO - check H_0toB
    q(1,1) = atan2(H_0toB(2,1),H_0toB(1,1));
else
    q(1,1) = 0;
end

if nargin > 2
    if ~ismatrix(qlims) || size(qlims,1) ~= 7 || size(qlims,2) ~= 2
        error('Joint limits must be specified as a 7x2 array.');
    end
else
    qlims = [];
end

%% Define q
theta = DHtable(:,1);
% TODO - check DH table
% d = DHtable(:,2);
% a = DHtable(:,3);
% alpha = DHtable(:,4);
q(2,1) = wrapToPi( pi/2 - theta(1) );
q(3,1) = 0;
q(4,1) = wrapToPi( theta(2) );
q(5,1) = 0;
q(6,1) = wrapToPi( theta(3) - pi/2 );
q(7,1) = wrapToPi( theta(4) );

%% Check joint limits
if ~isempty(qlims)
    % Check limits
    tf_min = q < qlims(:,1);
    tf_max = q > qlims(:,2);
    
    if nnz(tf_min) > 0
        str = '';
        for i = 1:numel(tf_min)
            if tf_min(i)
                str = sprintf('%s\n\tJoint %d is %.5f which is outside of the [%.5f,%.5f] limit.',...
                    str,i,q(i),qlims(i,1),qlims(i,2));
            end
        end
        warning(str);
        q = [];
    end
    
    if nnz(tf_max) > 0 && ~isempty(q)
        str = '';
        for i = 1:numel(tf_max)
            if tf_max(i)
                str = sprintf('%s\n\tJoint %d is %.5f which is outside of the [%.5f,%.5f] limit.',...
                    str,i,q(i),qlims(i,1),qlims(i,2));
            end
        end
        warning(str);
        q = [];
    end
    
    if isempty(q)
        return
    end
end