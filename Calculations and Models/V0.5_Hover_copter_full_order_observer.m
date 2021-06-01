%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Info
% Author: Caleb Nelson
% Revision: 0.5
% Revision Info: System model with feedback and full order observer
% Last Edit: 5/31/2021
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

%% DEVELOP STATE SPACE MODEL FOR CONTROLLED SYSTEM WITH FEEDBACK
% X for this system is [theta, omega]
% Find controller gain matrix G for the controlled system so that poles are in the left hand plane
% Use LQR to determine and choose poles
Q = [5 0; 0 1];
R = 1;
G = lqr(A,B,Q,R)
% Verify poles/eigenvalues are where we want them with the controller implemented -- negative poles are stable since they are in the left hand plane
disp('The poles of the system with closed loop feedback A-B*G with the G controller implemented are:')
eig(A-B*G)

% Create Controlled System (system with feedback and control -- observer not included in this state space model)
% Reminder: X for this system is [theta, omega]
A_con = A-B*G;
B_con = [0; 0];   % No more small signal input for the controlled system, the input is now -G*X
C_con = [1 0];    % Measure only theta, theta is the only output, all the theta dot (omega) outputs will be 0 in Y
D_con = 0;
controlled_sys = ss(A_con,B_con,C_con,D_con);
% Simulate results for controlled system
t=0:0.05:10;              % times to simulate, start:step:stop
U = zeros(size(t));       % input, discrete values for each time
initial_angle = pi/3;     % initial angle
initial_omega = 0.2;      % initial theta dot or omega or angular velocity (all equivalent)
[Y_con,T,X_con] = lsim(controlled_sys,U,t,[initial_angle;initial_omega]);  % doesn't plot when output arguments are desired
% Check eigenvectors and eigenvalues
##disp('The eigenvectors and eigenvalues of A for the controlled system are:')
##[eigenvectors, eigenvalues] = eig(controlled_sys.a)



%% DEVELOP STATE SPACE MODEL FOR THE OBSERVER
% (this is method 1, where the two systems are modeled and calculated separtely and then the difference between them is plotted)
% X for this system is [theta_hat, omega_hat]

% Place poles for observer
% find gain matrix K for the observer so that poles are in the left hand plane
% Note the use of A and C transpose, they are transposed because the system has K*C instead of C*K
% See written documentation for more info on that.

% Using LQR pole placement:
Q = [10 0; 0 10];     % Minimize equation X'*Q*X - therefore Q could be thought of as Vd (input disturbances) in this context -- bigger Q means X stays smaller make this big so mistakes in this variable don't matter too much
##R = [0.1 0; 0 0.1];   % R is 2x2 if measuring both states in X  -- minimize equation Y'*R*Y -- Therefore R could be thought of as Vn (output noise) in this context
R = 0.1;              % R is 1x1 if measuring only one state in X
disp("K gain matrix with LQR placed kalman observer poles:")
K = (lqr(A',C',Q,R))'

% Using Kalman Filter pole placement:
Q = [10 0; 0 10];
##R = [0.1 0; 0 0.1];   % R is 2x2 if measuring both states in X
R = 0.1;              % R is 1x1 if measuring only one state in X
B_u = B;   % Matrix defining how much the u input affects each state in x
B_w = B;   % Matrix defining how much w noise input affects each state in x
Ts = -1;      % Mark it as discrete with no specific sample time
sys = ss(A,[B_u B_w],C,D,Ts,'InputName',{'u' 'w'},'OutputName','y');  % Plant dynamics and additive input noise w - B has to be a 2x2 for kalman function because there are two inputs, U and noise inputs
[kalman_filter_sys,K,Xf] = kalman(sys,Q,R);   % Creates a full system
disp("K gain matrix with kalman function placed kalman observer poles:")
K

% Using LQE pole placement to build Kalman Filter:
% Augment system with disturbances and noise
Vd = [0.1 0; 0 0.1];  % disturbance covariance
##Vn = [1 0; 0 1];    % noise covariance -- 2x2 if measuring both states in X
Vn = 1;               % noise covariance -- 1x1 if measuring only 1 state in X
[K,P,E] = lqe(A,Vd,C,Vd,Vn);  % design Kalman filter observer/estimator
disp("K gain matrix with LQE placed kalman observer poles:")
K
kalman_filter_observer_eigs = E

% Using manual pole placement:
obs_poles = [-10, -9];
disp("K gain matrix with manually placed kalman observer poles:")
K = (place(A',C',obs_poles))'


% Verify poles/eigenvalues are where we want them -- negative poles are stable since they are in the left hand plane
disp('The poles of the observer are determined by A-K*C and are:')
eig(A-K*C)


% Create Observer System
% Reminder: X for this system is [theta_hat, omega_hat]
A_obs = A-K*C;
B_obs = [B K];    % Observer has both u and Y as inputs
C_obs = eye(2);   % Output should include both estimates for both states in X
D_obs = 0*[B K];
observer_sys = ss(A_obs, B_obs, C_obs, D_obs);

% Simulate results for observer system
% Give both u and Y as inputs to observer
t=0:0.05:10;              % times to simulate, start:step:stop
observer_input = [U;Y'];  % input to observer if system is uncontrolled is U and Y'
observer_input_controlled = [-G*X';Y'];  % input to observer if controlled is U=-G*X' and Y'
initial_theta_hat = pi/3; % initial theta state for observer
initial_omega_hat = 0.2;  % initial omega state for observer
initial_conditions = [initial_theta_hat initial_omega_hat];
[Y_hat,T,X_hat] = lsim(observer_sys,observer_input',t,initial_conditions);  % doesn't plot when output arguments are desired



%% SECOND METHOD -- COMBINE THE CONTROLLED FEEDBACK SYSTEM WITH THE OBSERVER SYSTEM
% Create one big, larger system, where its state includes both the controlled
% feedback system's states and the observers states
% so X for this combined system would be [theta, omega, theta_hat, omega_hat]
B_combined = [B; B];   % 4x1 -- same B for the estimated states as the actual states
A_combined = [A zeros(2); K*C A-K*C];  % 4x4
C_combined = eye(4);   % 4x4 -- All states observable
D_combined = D;
combined_system = ss(A_combined, B_combined, C_combined, D_combined);

% Simulate results for combined system
% Give both u and Y as inputs to observer
t=0:0.05:10;              % times to simulate, start:step:stop
combined_input = [U];     % input to combined system if system is uncontrolled is U
combined_input_controlled = [-G*X'];  % input to combined system if controlled is U=-G*X'
initial_theta = pi/3;     % initial theta for feedback system/actual system
initial_omega = 0.2;      % initial omega for feedback system/actual system
initial_theta_hat = pi/3; % initial theta state for observer
initial_omega_hat = 0.2;  % initial omega state for observer
initial_conditions = [initial_theta; initial_omega; initial_theta_hat; initial_omega_hat];  % 4x1
[Y_combined,T,X_combined] = lsim(combined_system,combined_input',t,initial_conditions);  % doesn't plot when output arguments are desired

% Plot observer error
% Difference between actual state X and estimated state X_hat
X_substate = X_combined'(1:2,:);      % Portion of combined X states that is just the X states -- (first two rows)
X_hat_substate = X_combined'(3:4,:);  % Portion of combined X states that is just the X hat states -- (second two rows)

% Plot the difference between the first and second methods of approxiamtion for X
figure();
plot(t,X-X_substate');   % Should be 0

% Plot the difference between the first and second methods of approxiamtion for X_hat
figure();
plot(t,X_hat-X_hat_substate');   % Should be 0

% Plot observer error using the two different methods, these should be the same
% The error is the difference between actual state X and estimated state X_hat, should converge to 0
figure();
plot(t,X-X_hat);
figure();
plot(t,X_substate-X_hat_substate);


% Something is wrong with X_hat_substate