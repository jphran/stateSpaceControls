% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 4
clc;
clear; close all;

load('mathModel.mat');
load('robust.mat');

%finding A_tilde 
syms k1 k2 ka
K_lqr = [k1 k2 ka];
B_lqr = [-B; 0];
A_lqr = double(A_rt + B_lqr*K_lqr);

%form Q and R
%diag
q11 = 1;
q22 = 1e3;
q33 = 1e3;
%off diag
q12 = 5;
q13 = 5;
q23 = 5;

Q = [[q11 q12 q13];[q12 q22 q23];[q13 q23 q33]]; %cost applied to states


R = 1e2; %cost applied to inputs


[K S CLP] = lqr(A_lqr, B_lqr, Q, R);

K_lqr = K(1:2);
ka = K(end);

% model
lqrSim = sim('part4model.slx');

Ts = findSettlingTime(lqrSim.tout, lqrSim.pos);