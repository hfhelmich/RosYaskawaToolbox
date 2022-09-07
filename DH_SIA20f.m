function DHtable = DH_SIA20f(q)
% DH_SIA20F creates a DH table for the SIA20F manipulator
%   DHtable = DH_SIA20F(q)
%
%   Input(s)
%       q       - 7-element array containing joint values or symbolic 
%                 variables. Values must be specified in radians.
%
%   Output(s)
%       DHtable - 7x4 array containing the DH table parameters for the
%                 SIA20f for a given joint configuration q. Linear units
%                 are defined in meters.
%
%        DHtable = [theta_1,d_1,a_1,alpha_1; 
%                   theta_2,d_2,a_2,alpha_2;
%                   ...
%                   theta_N,d_N,a_N,alpha_N];
%
%        H_{i}^{i-1} = Rz(theta_i)*Tz(d_i)*Tx(a_i)*Rx(alpha_i);
%        H_{N}^{0} = H_{1}^{0}*H_{2}^{1}*...*H_{N}^{N-1};
%
%   This function leverages the Transformation Toolbox
%
%   See also DHtabletoFkin DH plotDHtable 
%
%   M. Kutzer, 07Jul2021, USNA

%% Check input(s)
narginchk(1,1);
if numel(q) ~= 7
    error('Joint state must be defined as a 7-element vector.');
end

%% Define fixed link lengths
L_1 = 0.41; 
L_2 = 0.49;
L_3 = 0.42;
L_4 = 0.18;

%% Create DH table
DHtable = [...
     q(1)     , L_1, 0, -pi/2;...
     q(2)     ,   0, 0,  pi/2;...
     q(3)     , L_2, 0, -pi/2;...
    -q(4)     ,   0, 0,  pi/2;...
    -q(5)     , L_3, 0, -pi/2;...
    -q(6)     ,   0, 0,  pi/2;...
    -q(7) + pi, L_4, 0,     0];

