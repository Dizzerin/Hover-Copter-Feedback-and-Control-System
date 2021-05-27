%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Caleb Nelson
% Revision: 0.5
% Revision Info: System model with feedback and full order observer
% Last Edit: 5/26/2021
% 
% Description
%   This script is used to model and control a hover arm
%   for the feedback and control systems course offered 
%   at Walla Walla University
%   
%   This v0.5 of the script uses the same state space model for the system used 
%   in the previous version (v0.4) except this version adds a full order observer
%   and various graphs and models regarding its full state estimation.
%   Here G is the gain matrix for the system and K is the gain matrix for the observer
%
%   The new system with the full order observer included is modeled two different
%   ways here.  The first is where the output of the first system (the normal 
%   feedback and control system) without the observer is used as part of the
%   input for the second system (the observer system) which is modeled and simulated
%   separately.  The second way of doing it is where the two systems are combined
%   into one new big system where the new state x includes both the x from the 
%   basic feedback and control system and the xHat from the observer.
%   In both of these cases, the difference between the actual state and the
%   observer state can be plotted as x-xHat.  This is the error in the state
%   estimation by the observer which should converge to zero very quickly.
%   The rate at which it converges is based on the pole placement which affects
%   the gain matrix K for the observer.
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
A = [[0,1];[0,-b]];
B = [[0];[l_rod_to_pivot/j_system]];
% for C matrix: num rows = num outputs (and number of measurements being taken simultaneously) and num columns = num states
C = eye(2);   % 2x2 identity matrix -- assumes all states are being measured and reported as output y
%C = [1,0];   % just theta is being measured and reported as an ouput y
D = [0];

%% DEVELOP STATE SPACE MODEL FOR CONTROLLED SYSTEM WITH FEEDBACK
% X for this system is [theta, omega]
% Find controller gain matrix G for the controlled system so that poles are in the left hand plane
% Use LQR to determine and choose poles
Q = [5,0;0,1];
R = 1;
G = lqr(A,B,Q,R)
% Verify poles/eigenvalues are where we want them with the controller implemented -- negative poles are stable since they are in the left hand plane
disp('The poles of the system with closed loop feedback A-B*G with the G controller implemented are:')
eig(A-B*G)

% Create Controlled System (system with feedback and control -- observer not included in this state space model)
% Reminder: X for this system is [theta, omega]
A_con = A-B*G;
B_con = [0;0];
C_con = [1 0;0 0];  % Saying we can only measure theta, theta is the only output, all the theta dot (omega) outputs will be 0 in Y
D_con = 0;
controlled_sys = ss(A_con,B_con,C_con,D_con);

% Simulate results for controlled system
t=0:0.05:10;              % times to simulate, start:step:stop
U = zeros(size(t));       % input, discrete values for each time
initial_angle = pi/3;     % initial angle
initial_omega = 0.2;      % initial theta dot or omega or angular velocity (all equivalent)
[Y,T,X] = lsim(controlled_sys,U,t,[initial_angle;initial_omega]);  % doesn't plot when output arguments are desired

% Check eigenvectors and eigenvalues
disp('The eigenvectors and eigenvalues of A for the controlled system are:')
[eigenvectors, eigenvalues] = eig(controlled_sys.a)



%% DEVELOP STATE SPACE MODEL FOR THE OBSERVER
% (this is method 1, where the two systems are modeled and calculated separtely and then the difference between them is plotted)
% X for this system is [theta_hat, omega_hat]

% Place poles for observer
% find gain matrix K for the observer so that poles are in the left hand plane
% Note the use of A and C transpose, they are transposed because the system has K*C instead of C*K
% See written documentation for more info on that.
% Using manual pole placement:
obs_poles = [-100, -101];
K = place(A',C',obs_poles)
% Using LQR pole placement:
Q = [10,0;0,10];
R = [0.01 0;0 0.01];
K = lqr(A',C',Q,R)
% Verify poles/eigenvalues are where we want them with the controller implemented -- negative poles are stable since they are in the left hand plane
disp('The poles of the observer are A-K*C which are:')
eig(A-K*C)

% Create Observer System
% Reminder: X for this system is [theta_hat, omega_hat]
A_obs = A-K*C
B_obs = [B K]    % Observer has both u and Y as inputs
C_obs = C
D_obs = D
observer_system = ss(A_obs, B_obs, C_obs, D_obs);

% Simulate results for observer system
% Give both u and Y as inputs to observer
t=0:0.05:10;              % times to simulate, start:step:stop
observer_input = [U;Y'(1,:)];                 % input to observer if system is uncontrolled is U and Y' (first row, all columns)
observer_input_controlled = [-G*Y';Y'(1,:)];  % input to observer is controlled is U=-G*Y' and Y' (first row, all columns)
initial_theta_hat = pi/3; % initial theta state for observer
initial_omega_hat = 0.2;  % initial omega state for observer
initial_conditions = [initial_theta_hat initial_omega_hat];
[Y_hat,T,X_hat] = lsim(observer_system,observer_input,t,initial_conditions);  % doesn't plot when output arguments are desired

% Plot observer error
% Difference between actual state X and estimated state X_hat
figure();
plot(t,X-Xhat);



%% SECOND METHOD -- COMBINE THE CONTROLLED FEEDBACK SYSTEM WITH THE OBSERVER SYSTEM
% Create one big, larger system, where its state includes both the controlled
% feedback system's states and the observers states
% so X for this combined system would be [theta, omega, theta_hat, omega_hat]
A_combined = [A 0; K*C A-K*C]
B_combined = [B B]
C_combined = C
D_combined = D
combined_system = ss(A_combined, B_combined, C_combined, D_combined);

% Simulate results for combined system
% Give both u and Y as inputs to observer
t=0:0.05:10;              % times to simulate, start:step:stop
combined_input = [U;Y'(1,:)];                 % input to combined system if system is uncontrolled is U and Y' (first row, all columns)
combined_input_controlled = [-G*Y';Y'(1,:)];  % input to combined system is controlled is U=-G*Y' and Y' (first row, all columns)
initial_theta = pi/3;     % initial theta for feedback system/actual system
initial_omega = 0.2;      % initial omega for feedback system/actual system
initial_theta_hat = pi/3; % initial theta state for observer
initial_omega_hat = 0.2;  % initial omega state for observer
initial_conditions = [initial_theta initial_omega initial_theta_hat initial_omega_hat];
figure();
lsim(combined_system,combined_input,t,initial_conditions);
[Y_combined,T,X_combined] = lsim(combined_system,combined_input,t,initial_conditions);  % doesn't plot when output arguments are desired

% Plot observer error
% Difference between actual state X and estimated state X_hat
X_substate = X_combined(1:2,:);     % Portion of combined X states that is just the X states -- (first two rows)
X_hat_substate = X_combined(3:4,:); % Portion of combined X states that is just the X hat states -- (second two rows)
figure();
plot(t,X_substate-X_hat_substate);
