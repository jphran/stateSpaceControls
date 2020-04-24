% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 3
clc;
clear; close all;

load('mathModel.mat');

[num, den] = ss2tf(A,B,C,D);
zeros = roots(num);
% its controllable because no zeros at s = 0

%find K matrix
syms ka k1 k2
%char eqn desired: s^3+12s^2+48s+64
K = [k1 k2];
A_rt = [[A+B*K B*ka];[-C 0]];
charEqn = charpoly(A_rt);

eqn1 = 12 == charEqn(2);
eqn2 = 48 == charEqn(3);
eqn3 = 64 == charEqn(4);

[meatMat,potatoesMat] = equationsToMatrix([eqn1, eqn2, eqn3], [k1, k2, ka]);
K = double(linsolve(meatMat, potatoesMat));

K_rt = K(1:2).';
ka = K(end);

% model
encGain = 2*pi /4096;

rtSim = sim('part3model.slx');

Ts = findSettlingTime(rtSim.tout, rtSim.pos);

RMSE = findRMSE(rtSim.tout, rtSim.pos, 1);

%save vars
save('robust.mat', 'A_rt', 'encGain');