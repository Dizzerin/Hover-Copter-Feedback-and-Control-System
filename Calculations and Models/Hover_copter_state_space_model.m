%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Caleb Nelson
% Revision: 0.1
% Revision Info: Initial System Model (Hanging down pendulum tests)
% Last Edit: 5/11/2021
% 
% Description
%   This script is used to model and control a hover arm
%   for the feedback and control systems course offered 
%   at Walla Walla University
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
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
% For unforced hanging down tests:
%   Experimental Period = 0.88 seconds
%   Experimental Frequency = 1.14 Hz

% Estimated parameters
r_copter = 0.01;          % radius of copter when estimated as an sphere for inertia calculations
b = 0.4;                  % damping coefficient (guessed/estimated by trial and error)

% Basic Calculated Parameters
g = 9.80665;                              % gravitational constant in m/s^2
d_rod = m_rod_t/l_rod_t;                  % linear density of rod -- kg/m
l_rod_extra = l_rod_t - l_rod_to_pivot;   % length of rod that will stick out on the back side of the pivot point
m_rod_to_pivot = d_rod * l_rod_to_pivot;  % mass of rod from copter to pivot point
m_rod_t = d_rod * l_rod_t;                % total mass of rod
m_rod_extra = d_rod * l_rod_extra;        % mass of extra bit of rod that sticks out on the back side of the pivot point
j_system = 1/3*m_rod_to_pivot*l_rod_to_pivot^2 + 1/3*m_rod_extra*l_rod_extra^2 + 2/5 * m_copter * r_copter^2 + m_copter * (l_rod_to_pivot + r_copter)^2;  % moment of inertia for the system

% State Space Model (for hanging down tests)
%   Note this negelects the extra rod (the only place it is taken into account is the intertia)
A = [[0,1];
     [-(g*l_rod_to_pivot*(m_copter+m_rod_to_pivot/2))/j_system,-b]];
B = [[0];
     [l_rod_to_pivot/j_system]];
%C = [1,0];
C = eye(2);                               % 2x2 identity matrix -- assumes all states are availble/observable as outputs
D = [0];

% Create state space model for pendulum
pendulum_ss = ss(A,B,C,D);

% Simulate results
t=0:0.05:25;            % times to simulate, start:step:stop
forcing_function = zeros(size(t));
initial_angle = pi;     % initial angle
initial_omega = 0;      % initial theta dot or omega or angular velocity (all equivalent)
lsim(pendulum_ss,forcing_function,t,[initial_angle;initial_omega]);

% Check eigenvectors and eigenvalues
disp('Eigenvectors and eigenvalues of A:')
[eigenvectors, eigenvalues] = eig(pendulum_ss.a) % should be all negative for hanging down.

% Check observability
observability_matrix = obsv(A,C);
rank_of_observability_matrix = rank(observability_matrix)

% Check controlability
controlabilty_matrix = ctrb(A,B);
rank_of_controlability_matrix = rank(controlabilty_matrix)

% Get singular value decomposition to see how controlable it is in each direction
[U,S,V] = svd(controlabilty_matrix,'econ');