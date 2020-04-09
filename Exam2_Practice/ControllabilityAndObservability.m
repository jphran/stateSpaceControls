% Justin Francis
% MEEN 5210, State Space Controls
% Dr. Jake Abbot, U of U
% Exam 2, familiarization

clc; clear; close all;

%% is it controllable?
%using thm 6.1 statement 4 (prefered)
% >>>ctrb Matrix: Co = [B AB ... A^(n-p)B], where n is length(A) 
% (or the number of rows) and p is the number of columns in B. if Co is
% full rank, than the system is controllable

%example
A = [[-2 3 0];[1 0 0];[0 1 0]];
B = [1 0 0].';
C = [0 1 3];
D = [0];

Co = ctrb(A,B);
isControllable = (rank(Co) == length(A));

%using thm 6.1 statement 3 (not prefered)
% charEqn = charpoly(A);
% evals = roots(charEqn);
% then check that [A-(eval*I) B] is full rank


%% is it observable?
%using thm 6.o1
% >>>obsv Matrix: Ob = [C CA ... C^(n-p)A], where n and p are the same as
% controllablility above

%example
A = [[-2 3 0];[1 0 0];[0 1 0]];
B = [1 0 0].';
C = [0 1 3];
D = [0];

Ob = obsv(A,C);
isObservable = (rank(Ob) == length(A));

%% Kalman Decomp: reduce the state eqn to a controllable one assuming its uncontrollable
% >>>form P_inv matrix with lin indie cols of Co and any other col that
% makes P_inv full rank. Conduct similarity transform (x_tild = P*x, 
% A_tild = P*A*P_inv, B_tild = P*B, C_tild = C*P_inv, D_tild = D).
% Extract controllable system from A_tild (standard structure, check notes
% (3/5/20) or text on Kalman Decomp

%example
A = [[-1 4];[4 -1]];
B = [1 1].';
C = [1 1];
D = [0];

Co = ctrb(A,B);

P_inv = [[1 0];[1 1]];
P = inv(P_inv);

% x_tild = P*x; 
A_tild = P*A*P_inv;
B_tild = P*B;
C_tild = C*P_inv;
D_tild = D;

% in standard struct, we can see A(1,1) is the controllable portion