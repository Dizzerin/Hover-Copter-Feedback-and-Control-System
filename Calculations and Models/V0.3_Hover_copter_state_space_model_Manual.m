%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Caleb Nelson
% Revision: 0.3
% Revision Info: Actual system model (horizontal instead of hanging down)
%                with state feedback - manual pole placement
% Last Edit: 5/26/2021
% 
% Description
%   This script is used to model and control a hover arm
%   for the feedback and control systems course offered 
%   at Walla Walla University
%   
%   This v0.3 of the script uses a new model (not the hanging down model from 
%   v0.2) to model the system with the theta angle defined to be 0 when the 
%   hover arm is horizontal and with state feedback gain parameters included.
%   Instead of the system being x_dot = Ax + Bu we now have x_dot = (A-BG)*x
%   The controller feeback gain parameters are set by manually placing them
%   to some arbitry but reasonable values in the left hand plane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;
close all;
clc;
pkg load control
pkg load signal
%pkg load symbolic
%graphics_toolkit('gnuplot')

% All values are in standard SI units (i.e. m, kg, s, N, etc.)
% Prefixes
% m_ indicates a mass
% l_ indicates a length
% d_ inidcates a density (could be linear density not volume density)
% j_ indicates a moment of inertia
%
% Suffixes
% _t indicates a total values

% Measured Parameters
m_copter = 0.0142;        % mass of copter (motor and blade assembly)
m_rod_t = 0.0243;         % total mass of hover arm rod
l_rod_t = 0.4953;         % total length of hover arm rod
l_rod_to_pivot = 0.395;   % length of hover arm rod from copter to pivot point

% Estimated parameters
r_copter = 0.01;          % radius of copter when estimated as an sphere for inertia calculations
b = 0.41;                 % damping coefficient (guessed/estimated by trial and error)

% Constants and Basic Calculated Parameters
g = 9.80665;                              % gravitational constant in m/s^2
d_rod = m_rod_t/l_rod_t;                  % linear density of rod -- kg/m
l_rod_extra = l_rod_t - l_rod_to_pivot;   % length of rod that will stick out on the back side of the pivot point
m_rod_to_pivot = d_rod * l_rod_to_pivot;  % mass of rod from copter to pivot point
m_rod_t = d_rod * l_rod_t;                % total mass of rod
m_rod_extra = d_rod * l_rod_extra;        % mass of extra bit of rod that sticks out on the back side of the pivot point
j_system = 1/3*m_rod_to_pivot*l_rod_to_pivot^2 + 1/3*m_rod_extra*l_rod_extra^2 + 2/5 * m_copter * r_copter^2 + m_copter * (l_rod_to_pivot + r_copter)^2;  % moment of inertia for the system

% State Space Model (for actual use case)
A = [[0,1];[0,-b]];
B = [[0];[l_rod_to_pivot/j_system]];
% for C matrix: num rows = num outputs (and number of measurements being taken simultaneously) and num columns = num states
C = eye(2);   % 2x2 identity matrix -- assumes all states are being measured and reported as output y
%C = [1,0];   % just theta is being measured and reported as an ouput y
D = [0];

% Create state space model for pendulum
uncontrolled_sys = ss(A,B,C,D);

% Simulate results
t=0:0.05:25;              % times to simulate, start:step:stop
forcing_function = zeros(size(t));   % input, discrete values for each time
initial_angle = pi/4;     % initial angle
initial_omega = 0.2;      % initial theta dot or omega or angular velocity (all equivalent)
figure();
lsim(uncontrolled_sys,forcing_function,t,[initial_angle;initial_omega]);

% Check eigenvectors and eigenvalues
disp('The eigenvectors and eigenvalues of A for the uncontrolled system are:')
[eigenvectors, eigenvalues] = eig(uncontrolled_sys.a)

% Check observability
observability_matrix = obsv(A,C);
rank_of_observability_matrix = rank(observability_matrix)

% Check controlability
controlabilty_matrix = ctrb(A,B);
rank_of_controlability_matrix = rank(controlabilty_matrix)

% Get singular value decomposition to see how controlable it is in each direction
[U,S,V] = svd(controlabilty_matrix,'econ');

% Desired pole locations -- Manually place poles -- Find controller gain matrix G so that poles are where we want them
##desired_eigs = [-1.0;-1.1];     % Not aggresive enough
##desired_eigs = [-2.0;-2.1];
##desired_eigs = [-3.0;-3.1];
##desired_eigs = [-4.0;-4.1];
##desired_eigs = [-5.0;-5.1];
##desired_eigs = [-6.0;-6.1];
##desired_eigs = [-7.0;-7.1];     % Too aggressive?
desired_eigs = [-10;-11];
%desired_eigs = [-12;-13];

disp('Your propotinal feedback gain controller G is:')
G = place(A,B,desired_eigs)

% Verify poles/eigenvalues are where we want them with the controller implemented -- negative poles are stable since they are in the left hand plane
disp('The poles of the system with closed loop feedback A-B*G with the G controller implemented are:')
eig(A-B*G)

% Controlled System
Ac = A-B*G;
Bc = [0;0];
Cc = C;
Dc = 0;
controlled_sys = ss(Ac,Bc,Cc,Dc);
% Simulate results
t=0:0.05:10;              % times to simulate, start:step:stop
forcing_function = zeros(size(t));   % input, discrete values for each time
initial_angle = pi/3;     % initial angle
initial_omega = 0.2;      % initial theta dot or omega or angular velocity (all equivalent)
figure();
lsim(controlled_sys,forcing_function,t,[initial_angle;initial_omega]);
[Y,T,X] = lsim(controlled_sys,forcing_function,t,[initial_angle;initial_omega]);  % doesn't plot when output arguments are desired

% Check eigenvectors and eigenvalues
disp('The eigenvectors and eigenvalues of A for the controlled system are:')
[eigenvectors, eigenvalues] = eig(controlled_sys.a)

% Plot duty cycle -- make sure this small signal part isn't too big -- shows what small signal duty cycle is being added
figure();
plot(t,-G*Y');
title('Small signal duty cycle');
ylabel('Duty Cycle');
xlabel('Time (s)');

% Plot large signal response portion without small signal approximation
% plot controlled and uncontrolled system
% for controlled system just update the fM
% fM = FM + fm
% fm = -G*x
FM = 10;
fm = 0;
fM = FM + fm;
