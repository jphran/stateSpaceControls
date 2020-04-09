% Justin Francis
% MEEN 5210, State Space Controls
% Dr. Jake Abbot, U of U
% Exam 2, familiarization

clc; clear; close all;

%% Jordan form controllablity
% >>>B matrix must have lin indie entries that correspond to the last entry
% of the jordan blocks of the same eigen value.

%exmple
v = [2 2 2 2 1 1 1];
v1 =[1 0 0 0 1 0];
A = diag(v) + diag(v1, 1); %in jordan form
B = [[2 1 1];[2 1 1];[1 1 -1];[3 2 1];[-1 0 1];[1 0 1];[1 -1 2]];
C = [[2 2 -1 3 -1 -1 1];[1 3 -1 2 0 0 0];[0 -4 -1 1 1 1 0]];
D = [0];

checkLinIndB_eval2 = [B(2,:).', B(3,:).', B(4,:).'];
checkLinIndB_eval1 = [B(6,:).', B(7,:).'];
size2 = size(checkLinIndB_eval2);
size1 = size(checkLinIndB_eval1);

isControllable = (rank(checkLinIndB_eval2) == size2(2) && rank(checkLinIndB_eval1) == size1(2));

%% Jordan form observability
% >>>C matrix must have lin indie entries that correspond to the first entry
% of the jordan blocks of the same eigen value.

%exmple
v = [2 2 2 2 1 1 1];
v1 =[1 0 0 0 1 0];
A = diag(v) + diag(v1, 1); %in jordan form
B = [[2 1 1];[2 1 1];[1 1 -1];[3 2 1];[-1 0 1];[1 0 1];[1 -1 2]];
C = [[2 2 -1 3 -1 -1 1];[1 3 -1 2 0 0 0];[0 -4 -1 1 1 1 0]];
D = [0];

checkLinIndC_eval2 = [C(:,1), C(:,3), C(:,4)];
checkLinIndC_eval1 = [C(:,5), C(:,7)];

size2 = size(checkLinIndC_eval2);
size1 = size(checkLinIndC_eval1);

isObservable = (rank(checkLinIndC_eval2) == size2(2) && rank(checkLinIndC_eval1) == size1(2));

%% finding B and C matrices to make sys controllable and observable in jordan form
v = ones(1,5);
v1 = [1 0 1 0];
A = diag(v) + diag(v1, 2);
%size(B) == (5,2);
%size(C) == (3,5);
D = [0];
%it is impossible to control because there are not enough columns of B to 
%create a full rank matrix
%it is possible to observe because there are enough rows in C to form a
%full rank matrix

%% convert cont to discrete time sys
%check notes on 3/20/20, last page. Also, check function in HW7
%(discreteEquivAandB.m)
% >>>Find fastest sampling period before losing controllability: peep notes
% on 3/20/20, last page and findFastestUnctonrollable.m in HW7


