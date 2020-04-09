%% 1
clc; close all; clear;

A = [[1 0];[-10 0]];
B = [1 -10].';
C = [1 -10];

Co = ctrb(A,B);
rank(Co)

Ob = obsv(A,C);
rank(Ob)

%% 2
clc; clear;

A = [[0 11];[0 -4]];
jordan(A)

%% 3
clc; clear;

A = [[-4 20];[0 -4]];
B = [1 -1].';
C = [0 2];
D = [0];

%use this for checking poles (roots of den) of transfer function
syms s
G = C*inv((s*eye(size(A)) - A))*B - D;

%% 4
clc; clear;

% state feedback gains
% >>>use u = r-Kx to add in controller gains K. Then the st eqns become A_n
% = (A-BK), find the char poly of A_n, set equal to the desired charpoly
% with desired eigenvals, solve for K

%example
A = [[0 0];[0 2]];
B = [10 -10].';
C = [1 0];
D = [0];

syms k1 k2 s
K = [k1 k2];
A_n = A - B*K;
desCharEqn = (s + 10)*(s+10);
charEqn = charpoly(A_n);

K = inv([[10 -20];[-10 0]])*[22;100]; % pulled from char Eqn and desired char Eqn

%check with 
% desEigVals = [-10 -10];
% K_check = place(A,B, desEigVals);

%% 5
clc; clear;

A = [[-5 2];[0 -4]];
B = [0 100].';
C = [0.2 0];
D = [0];

[num, den] = ss2tf(A,B,C,D);

    
%% 6
clc; clear;

bad = 2*pi/0.001;



%% 7
clc; clear;
rank2b = rank([[1 0];[0 1]])
rank10b = rank([0;1])

rank2 = rank([[-4 -2];[0 1];[4 2]])
rank10 = rank([-2 0 2].')

%% 8
% Kalman Decomp: reduce the state eqn to a controllable one assuming its uncontrollable
% >>>form P_inv matrix with lin indie cols of Co and any other col that
% makes P_inv full rank. Conduct similarity transform (x_tild = P*x, 
% A_tild = P*A*P_inv, B_tild = P*B, C_tild = C*P_inv, D_tild = D).
% Extract controllable system from A_tild (standard structure, check notes
% (3/5/20) or text on Kalman Decomp

%example
A = [[-4.5 -.25];[-13 1.5]];
B = [0.5 1].';

Co = ctrb(A,B);

P_inv = [[.5 1];[1 0]];
P = inv(P_inv);

% x_tild = P*x; 
A_tild = P*A*P_inv;
B_tild = P*B;

% in standard struct, we can see A(1,1) is the controllable portion