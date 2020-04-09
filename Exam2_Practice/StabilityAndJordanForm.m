% Justin Francis
% MEEN 5210, State Space Controls
% Dr. Jake Abbot, U of U
% Exam 2, familiarization

clc; clear; close all;

%% is it stable?
% >>>BIBO is defined as all of the poles of the tf (C(sI-A)^(-1)B + D) are < 1 or < 0 for discrete
% and cont time sys respectively
% >>>MS is defined as all eigen vals of the sys are <= 1 or <= 0 for disc
% and cont time sys respectively and that all eig vals that == 1 or == 0
% are simple roots of the charpoly (only one root or jordan block size of
% 1)
% >>>AS is defined as all e vals are < 1 or < 0 for disc and cont time sys

%example
A = [[-1 5];[0 2]];
B = [2 0].';
C = [-2 4];
D = [-2];

%use this for checking poles (roots of den) of transfer function
syms s
G = C*inv((s*eye(size(A)) - A))*B - D;

%%%%%%%%%%%%%%%DO NOT USE%%%%%%%%%%%%%%%%%%%%
%idk why but this gives some wack answers
% %create tf
% [num, den] = ss2tf(A,B,C,D);
% 
% %find poles of tf
% poles = pole(ss(A,B,C,D));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% jordan form
A = [ 0.9 0 2;
       0 1 1;
       0 0 1];
A = sym(A);
[V,J] = jordan(A);
