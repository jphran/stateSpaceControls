% Justin Francis
% MEEN 5210, state space controls
% Dr. Jake Abbot, U of U
% HW 7, stability and controllability

clc; clear; close all;

%% Problem 3
% % (a) Verify that this system continuous-time system is controllable. You must show every step of
% % your work, but you may use MATLAB to assist with your numerical calculations.
% % (b) We want to implement a sampled-data system using a microcontroller with a sampling period
% % of T = 1 second. Use the MATLAB function c2d to do this for you. Verify that the system is still
% % controllable after sampling. You may again use MATLAB to assist with your numerical
% % calculations. Note: use a dot notation to pull a matrix out of a state-space system sys in
% % MATLAB (e.g., sys.A).
% % (c) Using equation 6.67 in the textbook, write your own MATLAB script to calculate discrete
% % equivalent A and B matrices, and verify that they match the matrices from Part (b). If you've
% % done it correctly, your matrices should be very close, with only negligible numerical errors.
% % (d) Using Theorem 6.9 from the textbook, determine the fasted sampling period (i.e., the smallest
% % T) for which we should expect to lose controllability due to sampling.
% % (e) Repeat Part (b) using your T from Part (d), and verify that the system is not controllable after
% % sampling.
clc;

% part a
A = [[-15 -79 -145];[1 0 0];[0 1 0]];
B = [2 0 0].';
C = [10 0 0];
D = [0];
Co = ctrb(A,B);
isControllable = (rank(Co) == length(A));

% part b
T = 1; % [s]
sysDesired = ss(A,B,C,D);
[sysD, G] = c2d(sysDesired, T);
CoD = ctrb(sysD.A, sysD.B);
isControllableD = (rank(CoD) == length(sysD.A));

% part c
[A_d, B_d] = discreteEquivAandB(A, B, T);
isAcorrect = isequal(ones(length(A)), (A_d - sysD.A) < 0.001);
isBcorrect = isequal(ones(size(B)), (B_d - sysD.B) < 0.001);

% part d
eVals = eig(A);
T_fast = findFastestStableSamplingTime(A, 5);
T_max = findFastestUncontrollableSamplingTime(A);

% part e
sysNew = c2d(sysDesired, T_max);
CoNew = ctrb(sysNew.A, sysD.B);
isControllableNew = (rank(CoNew) == length(sysNew.A));


%% prob 4
A = [[2 1]; [-1 1]];
B = [1 2].';
C = [1 1];
D = [0];

syms x y s
k = [x y];
A_n = (A-B*k);
charEqn = det(s.*eye(2) - A_n);

eqn1 = x + 2*y -3 == 3;
eqn2 = x - 5*y +3 == 2;

[X,Y] = equationsToMatrix([eqn1, eqn2], [x, y]);
K = linsolve(X,Y);

check = place(A,B,[-1,-2]);

%% prob 5
A = [[2 1]; [-1 1]];
B = [1 2].';
C = [1 1];
D = [0];

k_bar = [6 -1];
Co = [[1 4]; [2 1]];
Co_barInv = [[1 -3]; [0 1]];
Co_inv = inv(Co);
Co_bar = inv(Co_barInv);

k = k_bar*Co_bar*Co_inv;

%% prob 6
A = [[1 1 -2];[0 1 1];[0 0 1]];
B = [1 0 1].';
C = [2 0 0];
D = [0];
p_des = [-2 -1+1j -1-1j];

k = place(A,B,p_des);

sysDesired = ss((A-B*k), B, C, D);
Gs = tf(sysDesired);

p = dcgain(sysDesired)^-1;

% plots for x0 = [0 0 0]
close all;
x0 = [0 0 0].';
r = 1;

t = linspace(0,10);
u = p*r*ones(size(t));

[Y,T,X] = lsim(sysDesired, u, t, x0);

figure();
plot(t,Y);
title('Justin Francis, Ouput of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('Output, y(t)[units]');
grid();

figure();
plot(t, u);
title('Justin Francis, Input of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('Input, u(t)[units]');
grid();

figure();
plot(t, X);
title('Justin Francis, States of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('States, x(t)[units]');
grid();
legend({'x1', 'x2', 'x3'});


%plots for x0 = [2 0 -2]
x0 = [2 0 -2].';
[Y,T,X] = lsim(sysDesired, u, t, x0);

figure();
plot(t,Y);
title('Justin Francis, Ouput of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('Output, y(t)[units]');
grid();

figure();
plot(t, u);
title('Justin Francis, Input of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('Input, u(t)[units]');
grid();

figure();
plot(t, X);
title('Justin Francis, States of Zero State, x(0) = [0 0 0]');
xlabel('Time, t[s]');
ylabel('States, x(t)[units]');
grid();
legend({'x1', 'x2', 'x3'});
