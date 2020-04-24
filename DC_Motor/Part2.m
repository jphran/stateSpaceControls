% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 1, find A, B matrix
clc;
clear; close all;

load('mathModel.mat');

%verify contr
isControllable = (rank(ctrb(A,B)) == length(A));

%% design feedback controller
Ts_des = 1; %[s]
tau_des = Ts_des/4;

%find poles
p = -1/tau_des;

%find K
K_fsf = place(A, B, [p p+.00001]);

%find kff
dcGain_cl = C*inv(-(A-B*K_fsf))*B + D;
kff = 1/dcGain_cl;

%convert conts of enc to rad
encGain = 2*pi / 4096; 


%model
% fsf = sim('part2model.slx');

Ts = findSettlingTime(fsf.tout, fsf.pos);

RMSE = findRMSE(fsf.tout, fsf.pos, 1);
