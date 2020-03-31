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


 A = [[-15 -79 -145];[1 0 0];[0 1 0]];
 B = [2 0 0].';
 C = [10 0 0];
 D = 0;
 Co = ctrb(A,B);