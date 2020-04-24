% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 1, find A, B matrix
clc; 
clear; close all;
modelSim = sim('StateSpaceLab2019b.slx');
enc = modelSim.encoder * (2*pi) / 4096;
t = modelSim.time; 

tach = modelSim.tachometer;

tach = medfilt1(tach,20);

input = modelSim.current;

yss = mean(tach(length(tach)-20));

dcGain = yss/mean(input);

%find settling time
Ts_check = tach > 0.96 * yss;
t_check = t(Ts_check);
Ts = t_check(1); %settling time

tau = Ts/4; %time const

%find A matrix
A11 = -1/tau;
A21 = 1; %given from ss model
A12 = 0; %given
A22 = 0; %given

A = [[A11 A12];[A21 A22]];

B1 = dcGain/tau;
B2 = 0; %given model

B = [B1 B2].';

C = [0 1];
D = [0];

save('mathModel', 'A', 'B', 'C', 'D');