%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Caleb Nelson
% Revision: 0.6
% Revision Info: System model with feedback and reduced order observer
% Last Edit: 6/2/2021
% 
% Description
%   This script is used to model and control a hover arm
%   for the feedback and control systems course offered 
%   at Walla Walla University
%   
%   This v0.6 of the script uses the same state space model for the system used 
%   in the previous version (v0.5) except this version uses a reduced order observer
%   instead of the full order observer.
%
%   The difference between the actual state and the
%   observer state can be plotted as x-xHat.  This is the error in the state
%   estimation by the observer which should converge to zero very quickly.
%   The rate at which it converges is based on the pole placement which affects
%   the gain matrix L for the observer.
%
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


%% DEVELOP STATE SPACE MODEL FOR UNCONTROLLED SYSTEM
% State Space Model
A = [0 1; 0 -b];
B = [0; l_rod_to_pivot/j_system];
% for C matrix: num rows = num outputs (and number of measurements being taken simultaneously) and num columns = num states
%C = eye(2);  % 2x2 identity matrix -- assumes all states are being measured and reported as output y
C = [1 0];   % just theta is being measured and reported as an ouput y
D = [0];
uncontrolled_sys = ss(A,B,C,D);
% Simulate results for uncontrolled system
t=0:0.05:10;              % times to simulate, start:step:stop
U = zeros(size(t));       % input, discrete values for each time
initial_angle = pi/3;     % initial angle
initial_omega = 0.2;      % initial theta dot or omega or angular velocity (all equivalent)
[Y,T,X] = lsim(uncontrolled_sys,U,t,[initial_angle;initial_omega]);  % doesn't plot when output arguments are desired
% Check eigenvectors and eigenvalues
##disp('The eigenvectors and eigenvalues of A for the uncontrolled system are:')
##[eigenvectors, eigenvalues] = eig(uncontrolled_sys.a)

%% DEVELOP THE REDUCED ORDER OBSEVER
% Check the controlability -- make sure the system is still controlloable when measuring only theta
Q = ctrb(A,B)
Rank_Q = rank(Q)

% Check the observability of full system assuming we can measure all outputs -- make sure the system is observable when measuring only theta
R = obsv(A,C)
Rank_R = rank(R)

% Find where we need to place the poles by finding the roots of the characteristic equation
poles_F = [-10];

% Define sub matricies
A11 = A(1,1);
A12 = A(1,2:end);
A21 = A(2:end, 1);
A22 = A(2:end, 2:end);
B1 = B(1);
B2 = B(2);
C1 = C(1);

% Find L' by placing the poles at the desired pole locations found earlier
% Note: Place gives L' not L in this case so we transpose it again
L = (place(A22', A12'*C1', poles_F))'

% Now find H, F, and G double bar
H = B2-L*C1*B1
F = A22 - L*C1*A12
G_double_bar = (A21 - L*C1*A11)*C1^(-1)


% TODO model/simulate/plot output
