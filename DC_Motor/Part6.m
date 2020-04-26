% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 6
clc;
clear; 
close all;

%load data
load('mathModel.mat');
load('robust.mat');
load('lqrPoles.mat');

%find obsv gains
p = real(max(CLP));
p_des = [p p+.0001];
L_clo = place(A.', C.', p_des).';

cloSim = sim('part6model.slx');

%plot 
figure();
hold on;
plot(cloSim.tout, cloSim.tachometer, 'DisplayName', 'Raw Velocity');
plot(cloSim.tout, cloSim.simStates(:,1), 'LineWidth', 2, 'DisplayName', 'Simulated Velocity');
title('Closed-Loop Observer as a Filter');
xlabel('Time, t[s]');
ylabel('Velocity, v[rad/s]');
grid();
legend('Location', 'southeast');

% commentary
comments = ['The estimated state is always slightly higher than the actual '...
    'sensor output (by visual inspection). We believe the tachometer is actually' ...
    ' closer to the true velocity of the system. Unfortunately, it is noisy,' ...
    ' therefore, if we were to use one of these as a state feedback, we would '...
    'use the estimated velocity because it is relatively less noisy.'];

fprintf(comments);