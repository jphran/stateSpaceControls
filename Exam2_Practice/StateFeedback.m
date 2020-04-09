% Justin Francis
% MEEN 5210, State Space Controls
% Dr. Jake Abbot, U of U
% Exam 2, familiarization

clc; clear; close all;

%% state feedback gains
% >>>use u = r-Kx to add in controller gains K. Then the st eqns become A_n
% = (A-BK), find the char poly of A_n, set equal to the desired charpoly
% with desired eigenvals, solve for K

%example
A = [[2 1];[-1 1]];
B = [1 2].';
C = [1 1];
D = [0];

syms k1 k2 s
K = [k1 k2];
A_n = A - B*K;
desCharEqn = (s + 1)*(s+2);
charEqn = charpoly(A_n);

K = inv([[1 2];[1 -5]])*[6;-1]; % pulled from char Eqn and desired char Eqn

%check with 
desEigVals = [-1 -2];
K_check = place(A,B, desEigVals);

%% state observer gains
% >>>The st eqns become A_n
% = (A-LC), find the char poly of A_n, set equal to the desired charpoly
% with desired eigenvals, solve for L

