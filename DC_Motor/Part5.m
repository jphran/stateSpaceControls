% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 5
clc;
clear; 
close all;

load('mathModel.mat');
load('lqrPoles.mat');

%check observability
Ob = obsv(A, C);
isObservable = (rank(Ob) == length(A));

%design obsv
p = 5 * real(max(CLP));
L_clo = place(A.', C.', [p p+.0001]).';

%sim 
cloSim = sim('part5model.slx');

%find Ts etc
Ts = findSettlingTime(cloSim.tout, cloSim.pos)
RMSE = findRMSE(cloSim.tout, cloSim.pos, 1)

%plot 
figure();
hold on;
plot(cloSim.tout, cloSim.pos);
plot(Ts, cloSim.pos(find(cloSim.tout == Ts)), 'ro', 'DisplayName', 'Settling Time = 0.320 [s]');
title('Closed-Loop Observer Full State Feedback');
xlabel('Time, t[s]');
ylabel('Position, \theta[rad]');
grid();

% commentary
comments = ['The RMSE of the LQR Method gains on a robust tracking controller' ...
    ' with a closed loop observer full state feedback was %.3f [rad]. When compared' ...
    ' to just the LQR robust tracking controller, the settling time increased by ' ...
    '%.3f percent and the RMSE remained the same.'];
    
fprintf(comments, 0.0047, 4.375);