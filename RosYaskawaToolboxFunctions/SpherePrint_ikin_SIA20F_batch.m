function q = SpherePrint_ikin_SIA20F_batch(AzEl,ds,r,x_NtoB,qlims,sol)
% SPHEREPRINT_IKIN_SIA20F calculates the inverse kinematics solution for
% the spherical application of the SIA20F given azimuth, elevation, and 3D
% nozzle position.
%   q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB)
%   q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,sol)
%   q = SpherePrint_ikin_SIA20F(AzEl,ds,r,x_NtoB,qlims,sol)
%
%   Input(s)
%       AzEl    - Nx2-element matrix containing N pairs of [Az,El] angles
%                 in radians defining azimuth and elevation on the sphere
%       ds      - stem length
%       r       - print radius
%       x_NtoB  - Nx3 element matrix containing x/y/z positions of the
%                 nozzle tip relative to the robot base frame.
%       qlims   - 7x2 array containing joint limits in radians. If qlims
%                 is not specified or an empty set, joint limits are not
%                 considered
%       sol     - (OPTIONAL) character array specifying desired solution:
%                 {['elbow-up'], 'elbow-down'}
%
%   Output(s)
%       q - Nx7 array containing the joint configuration for the SIA20F
%           manipulator. If the ith desired task configuration is outside
%           of the workspace or if the calculated q is outside of specified
%           joint limits the 7 elements corresponding to the ith row will
%           be returned as nan
%
%   M. Kutzer, 14Jul2021, USNA
%   Modified by L. DeVries, 14Jul2021

%% Check input(s)
narginchk(4,6);

if mod(numel(AzEl),2) ~= 0
    error('Azimuth/Elevation matrix must be specified as a Nx2-element array.');
end
if numel(ds) ~= 1
    error('Stem length must be specified as a scalar value.');
end
if numel(r) ~= 1
    error('Radius must be specified as a scalar value.');
end
if mod(numel(x_NtoB),3) ~= 0 
    error('Nozzle position must be specified as a Nx3 matrix of 3D coordinates relative to the base frame of the robot.');
end
if nargin > 4
    if ~isempty(qlims)
        if ~ismatrix(qlims) || size(qlims,1) ~= 7 || size(qlims,2) ~= 2
            error('Joint limits must be specified as a 7x2 array or an empty set.');
        end
    end
else
    qlims = [];
end
if nargin > 5
    sol = lower(sol);
    switch sol
        case 'elbow-up'
            sol = 'elbow-up, wrist-up';
        case 'elbow-down'
            sol = 'elbow-down, wrist-up';
        otherwise
            error('"%s" is not a designated solution.',sol);
    end
else
    sol = 'elbow-up, wrist-up';
end

% quantify how many Az/El combinations have been provided (N)
[dimA,dimB] = size(AzEl);
if dimA==2
    N_chkAzEl = dimB;
    Ndim_chkAzEl = 2;
elseif dimB==2
    N_chkAzEl = dimA;
    Ndim_chkAzEl = 1;
end


% quantify how many x/y/z combinations have been provided (N)
[dimA,dimB] = size(x_NtoB);
if dimA==3
    N_chkXYZ = dimB;
    Ndim_chkXYZ = 2;
elseif dimB==3
    N_chkXYZ = dimA;
    Ndim_chkXYZ = 1;
end

% make sure a x/y/z position has been provided for each Az/El point
if N_chkXYZ~=N_chkAzEl
    error('AzEl and x_NtoB matrices do not have consistent number of points')
end


%% Calculate q(1) and H_0toB
q = zeros(N_chkAzEl,7);
q(:,1) = atan2(x_NtoB(:,2),x_NtoB(:,1)); % shoulder joint aligns to plane of nozzle location
% TODO: Check this!!!
if isempty(qlims)
    [DHtable,H_0toB] = SpherePrint_q2DH_SIA20F(q);
else
    [DHtable,H_0toB] = SpherePrint_q2DH_SIA20F(q,qlims);
end

%% Reference nozzle position to frame 0
x_NtoB = x_NtoB(1:3);
x_NtoB = reshape(x_NtoB,3,1);
x_NtoB(4) = 1;
H_Bto0 = invSE(H_0toB);
x_Nto0 = H_Bto0*x_NtoB;

%% Calculate inverse kinematic solution
q4 = SpherePrintDH_ikin_batch(AzEl,ds,r,x_Nto0,DHtable,sol);
if ~isempty(q4)
    DHtable(:,1) = q4;
    if isempty(qlims)
        q = SpherePrint_DH2q_SIA20F(DHtable,H_0toB);
    else
        q = SpherePrint_DH2q_SIA20F(DHtable,H_0toB,qlims);
    end
else
    q = [];
end

    