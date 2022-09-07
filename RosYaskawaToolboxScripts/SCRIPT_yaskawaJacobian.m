%%  Yaskawa and Jacobians
%   Harrison Helmich 27 Feb 22
%
%

clear;
clc;
%%
%   robot has 7 DOF
syms q1 q2 q3 q4 q5 q6 q7
q = [q1 q2 q3 q4 q5 q6 q7]

%%  Derive jacobian
%   forward kinematics, get H
%dh_table = DH_SIA20f(q);
%H = DHtableToFkin(dh_table);
H = fkin_SIA20f(q)

%% jacobian is body-fixed
jacobianSiaEE = calculateJacobian(q, H, 'Reference', 'Body',...
    'Order', 'RotationTranslation', 'Filename', 'jacobianSiaEE');

%% jacobian is world-referenced
jacobianSiaBase = calculateJacobian(q, H, 'Reference', 'Base',...
    'Order', 'RotationTranslation', 'Filename', 'jacobianSiaBase');

%% Save
%save('jacobianSia.mat', 'jacobianSiaEE', 'jacobianSiaBase');