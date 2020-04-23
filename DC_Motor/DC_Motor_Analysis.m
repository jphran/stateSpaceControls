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

%% part 2
A = [-2.6788 0;1 0];
B = [12.05465;0];
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

%simulink feedback analysis
% sim('Sim2.slx');
% vel = out.vel;
% t = out.tout;
% 
% figure();
% plot(t,vel);

%% settling time
Ts_check = vel > 0.96*vel(end);
t_check = t(Ts_check);

Ts_sim = t_check(1);

%% rmse steady state
velDes = 1;
vel_check = vel(Ts_check);

RSME = sqrt(mean((vel_check - velDes).^2));

%% robust tracking and distrubance rejection
[num, den] = ss2tf(A,B,C,D);
zeros = roots(num);
% its controllable because no zeros at s = 0

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
Acheck = [[A+B*[K1 K2] B*Ka];[-C 0]];
Bcheck = [0 0 1];
Kcheck = place(Acheck,B, [p+.0001 p-.0001 p]); 
