%% SCRIPT_SIA20F_DeriveJacobian
% Derive the SIA20F world fixed and body referenced Jacobians using
% DH_DH_SIA20F.m and symbolic variables
%
%   M. Kutzer, 28Feb2022, USNA
clear all
close all
clc

%% Define Forward Kinematics & Jacobians
% Define forward kinematics symbolically
syms q1 q2 q3 q4 q5 q6 q7
q = [q1; q2; q3; q4; q5; q6; q7];
dhTable = DH_SIA20f(q);
H_e2o_sym = DHtableToFkin(dhTable);

% Derive function for body-fixed Jacobian matrix
% Note: J_e is an "anonymous function"
J_o = calculateJacobian(q,H_e2o_sym,...
    'Reference','World','Order','RotationTranslation',...
    'Filename','J_o_SIA20F');

% Derive function for body-fixed Jacobian matrix
% Note: J_e is an "anonymous function"
J_e = calculateJacobian(q,H_e2o_sym,...
    'Reference','Body','Order','RotationTranslation',...
    'Filename','J_e_SIA20F');

