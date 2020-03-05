% Justin Francis
% MEEN 5210, State Space
% HW 5


%% prob 1, 5.9
clc; clear; close all;
A = [-1 5; 0 2];
B = [2 0].';
C = [-2 4]; 
D = [-2];

sys = ss(A,B,C,D);
t = [0:0.01:10];
u =  ones(length(t),1);

figure();
lsim(sys, u, t);
% step(sys);
title('Zero-State Step Response 5.9, Justin Francis');
saveas(gcf, 'Prob5_9.png');

%% prob 1, 5.11
x0 = rand(2,1);
u = zeros(length(t), 1);

figure();
lsim(sys, u, t, x0);
title('Zero-Input Response 5.11, Justin Francis');
saveas(gcf, 'Prob5_11.png');

%% prob 2, 5.12
clc; clear;

A = [-1 0 2; 0 0 0; 0 0 0];
B = [0 0 0].';
C = [0 0 0];
D = 0;
n = size(A);
x0 = rand(n(1),1);
t = [0:0.01:10];
u = zeros(size(t));
sys = ss(A, B, C, D);

lsim(sys, u, t, x0);
title('Zero-Input Response 5.12, Justin Francis');
saveas(gcf, 'Prob5_12.png');

%% prob 3, 5.13
clc; clear;

A = [-1 0 2; 0 0 1; 0 0 0];
B = [0 0 0].';
C = [0 0 0];
D = 0;
n = size(A);
x0 = rand(n(1),1);
t = [0:0.01:10];
u = zeros(size(t));
sys = ss(A, B, C, D);

lsim(sys, u, t, x0);
title('Zero-Input Response 5.13, Justin Francis');
saveas(gcf, 'Prob5_13.png');

%% prob 4, 5.14
clc; clear;

A = [0.9 0 2; 0 1 0; 0 0 1];
B = [0 0 0].';
C = [0 0 0];
D = 0;
n = size(A);
x0 = rand(n(1),1);
t = [0:0.01:10];
u = zeros(size(t));
sys = ss(A, B, C, D, 0.01);

lsim(sys, u, t, x0);
title('Discrete Zero-Input Response 5.14, Justin Francis');
saveas(gcf, 'Prob5_14.png');

%% prob 5, 5.15
clc; clear;

A = [0.9 0 2; 0 1 1; 0 0 1];
B = [0 0 0].';
C = [0 0 0];
D = 0;
n = size(A);
x0 = rand(n(1),1);
t = [0:0.01:10];
u = zeros(size(t));
sys = ss(A, B, C, D, 0.01);

lsim(sys, u, t, x0);
title('Discrete Zero-Input Response 5.15, Justin Francis');
saveas(gcf, 'Prob5_15.png');
