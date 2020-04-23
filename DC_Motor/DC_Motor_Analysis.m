% Jacob Anderson and Justin Francis
% MEEN 5210, State Space Lab
% Dr. Abbot, U of U
% DC Motor Lab

%% part 1, find A, B matrix
sim('StateSpaceLab2019b.slx');
enc = out.encoder * (2*pi) / 4096;
t = out.time; 

dxdt = (enc(end)-enc())/(t(4000)-t(3000));

tach = out.tachometer;

tach = medfilt1(tach,20);

p = polyfit(t(3500:end), enc(3500:end), 2);

dcGainByInspection = 45;

%% part 2
A = [-2.6788 0;1 0];
B = [120.5465;0];
C = [0 1];
D = [0];

isControllable = (rank(ctrb(A,B)) == length(A));

% using r = thetaDes 
% we want Ts = 1 [s], and crit damped
% Tc = 1/4*Ts
Ts = 1; %[s]
Tc = 1/4*Ts; 
p = -1/Tc; % desired poles

K = place(A, B, [p, (p+.00001)]);

dcGain = C * inv(-(A-B*K))* B + D;
feedForwardGain = 1/dcGain;
encGain = (2*pi)/4096; %[rad/counts]
%simulink feedback analysis
% sim('fullStateFeedback.slx');
pos = out.pos;
% t = out.tout;
% 
% figure();
% plot(t,vel);
% 
% settling time
Ts_check = pos > 0.96*pos(end);
t_check = t(Ts_check);

Ts_sim = t_check(1);

% rmse steady state
posDes = 1;
pos_check = pos(Ts_check);

RSME = sqrt(mean((pos_check - posDes).^2));

%% robust tracking and distrubance rejection
[num, den] = ss2tf(A,B,C,D);
zeros = roots(num);
% its controllable because no zeros at s = 0

initCond = 0;

syms ka k1 k2
%char eqn desired: s^3+12s^2+48s+64
K = [k1 k2];
An = [[A+B*K B*ka];[-C 0]];
charEqn = charpoly(An);
% Kn = place(A,B, [p+.0001 p-.0001 p]);
% eqn1 = 64 == (6786164656010219*ka)/562949953421312;
% eqn2 = 48 == -(6786164656010219*k2)/562949953421312;
% eqn3 = 12 == 6697/2500 - (6786164656010219*k1)/562949953421312;

eqn1 = 12 == charEqn(2);
eqn2 = 48 == charEqn(3);
eqn3 = 64 == charEqn(4);

[meatMat,potatoesMat] = equationsToMatrix([eqn1, eqn2, eqn3], [k1, k2, ka]);
K = double(linsolve(meatMat, potatoesMat));

%check
K1 = K(1);
K2 = K(2);
Ka = K(3);
K = [K1 K2];
Acheck = [[A+B*[K1 K2] B*Ka];[-C 0]];
Bcheck = [0 0 1].';
Kcheck = place(Acheck,Bcheck, [p+.0001 p-.0001 p]); 

pos = out.pos;

% settling time
Ts_check = pos > 0.96*pos(end);
t_check = t(Ts_check);

Ts_sim = t_check(1);

% rmse steady state
posDes = 1;
pos_check = pos(Ts_check);

RSME = sqrt(mean((pos_check - posDes).^2));

%% part 4, LQR
initCond = -1;
stepInput = 0;

%finding A_tilde 
syms k1 k2 ka
K_sym = [k1 k2 ka];
B_tilde = [-120.5465 0 0].';
A_tilde = double(An + B*K_sym);

C_tilde = [0 0 1];
D_tilde = [0];
sys = ss(A_tilde, B_tilde, C_tilde, D_tilde);

%diag
q11 = 1;
q22 = 1;
q33 = 1;
%off diag
q12 = 0;
q13 = 0;
q23 = 0;

Q = [[q11 q12 q13];[q12 q22 q23];[q13 q23 q33]]; %cost applied to states


R = 1; %cost applied to inputs

[K_tilde, S, CLP] = lqr(sys,Q,R);

K = K_tilde(1:2);
Ka = K_tilde(end);