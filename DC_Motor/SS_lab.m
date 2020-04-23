%%%%%%%%%%%%%%%%%%%%%%
% Jacob Anderson & Justin Francis
% ME EN 5210
% Lab Assignment
% 04/22/2020
%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc

%% - part 1 -
tach = out.tachometer;
t = out.tout;
y_ss = 4.5;
tach_check = tach > (0.96 * y_ss);
t_check = t(tach_check);
Ts = t_check(1);
T = Ts/4;

%% - part 2 -
A = [-2.6788 0;1 0];
B = [12.05465;0];
contr = [B A*B];
rank_Contr = rank(contr);
pdes = [-4 -4.000001];
k = place(A,B,pdes);
ydes = 0.1;
g_dc = 4.5;
kff = ydes/g_dc;



