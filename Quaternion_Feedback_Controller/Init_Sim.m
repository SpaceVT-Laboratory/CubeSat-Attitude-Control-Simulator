%*********************************************************************
% Init_Sim.m
% A file to setup the Simulink reaction wheel simulation
% for nonlinear quaternion feedback control
%*********************************************************************
clear all; close all; clc;

%*********************************************************************
% CSACS Physical Properties
%*********************************************************************
% ACTUAL PARAMETERS (from CAD, after auto-balance algorithm)
I = [0.07920140 0.00973142 0.00215335;
     0.00973142 0.07766589 -0.00163344;
     0.00215335 -0.00163344 0.13645595];         % intertia of CSACS [kg-m^2] 
r = [0.00004275; -0.00008398; -0.9099]*10^(-3);  % CM offset vector
m = 7.75345629;                                  % total mass of table [kg]
g = 9.81;                                        % acceleration due to gravity [m/s^2]

%*********************************************************************
% Reaction Wheel Array Properties
%*********************************************************************
Jw = 2.4e-5;                % reaction wheel inertia [kg.m^2]
Jm = 1.39e-6;               % rotor inertia [kg-m^2]
J  = Jw + Jm;               % combined intertia [kg-m^2]
Bw = 1e-3;                  % reaction wheel friction
Ke = 1/(56*pi);             % back emf gain [V/(rad/s)]
Kt = 5.67e-3;               % torque gain [N.m/A]
Rm = 2.2;                   % rotor resistance [Ohm]
LR = 0.378e-3;              % rotor inductance [H]


% Pyramidal distribution matrix
% All reaction wheels are taken as identical, if desired
% they can be scaled by the force and weight matrices below
beta = pi/4;      % 45 degree reaction wheel tilt
T    = [ cos(beta)            0        sin(beta);
                0      cos(beta)       sin(beta);
        -cos(beta)            0        sin(beta);
                0     -cos(beta)       sin(beta)];
            
% force matrix
k1 = 1;
k2 = 1;
k3 = 1;
k4 = 1;
K  = [k1  0   0   0;
       0 k2   0   0;
       0  0  k3   0;
       0  0   0  k4];

% weight matrix
w1 = 1;
w2 = 1;
w3 = 1;
w4 = 1;
W = [w1   0   0   0;
      0  w2   0   0;
      0   0  w3   0;
      0   0   0  w4];
  
% Uncomment section to disable a wheel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T(1,:) = zeros(1,3); % Disabling wheel 1 (Case 2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T(2,:) = zeros(1,3); % Disabling wheel 2 (Case 3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T(3,:) = zeros(1,3); % Disabling wheel 3 (Case 4)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% T(4,:) = zeros(1,3); % Disabling wheel 4 (Case 5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%*********************************************************************
% 2nd Order Reference Trajectory Parameters
%*********************************************************************
% INPUT: Settling Time Based On Maneuver Requirements!
ts   = 60;   % settling time requirement
zeta = 1;    % damping ratio (critically damped second-order system)

% Settling time for underdamped, second-order system using 2% percent max
% overshoot criteria:
% --> ts = 4/(zeta*wn)
% --> wn = 4/(zeta*ts)
wn = 4/(zeta*ts);

%*********************************************************************
% Controller Gains
%*********************************************************************
Kp = 2*wn^2;    %*diag(diag(I));
Kd = 2*zeta*wn; %*diag(diag(I));

%*********************************************************************
% Initial Attitude
%*********************************************************************
% INPUT: Euler angles [deg]
psi_d0   =  0;
theta_d0 =  0;
phi_d0   =  0;

% Euler angles [rad]
deg_0 = (pi/180)*[psi_d0; theta_d0; phi_d0];

% Quaternions
q_0 = euler2q(deg_0);

%*********************************************************************
% Initial Body Rates
%*********************************************************************
% INPUT: body rates [deg/sec]
wx_d =  0;
wy_d =  0;
wz_d =  0;

% Body rates [rad/s]
wb_0 = (pi/180)*[wx_d; wy_d; wz_d];

%*********************************************************************
% Initial State Vector
%*********************************************************************
x0 = [q_0(1); q_0(2); q_0(3); q_0(4); wb_0(1); wb_0(2); wb_0(3)];

%*********************************************************************
% Desired Attitude
%*********************************************************************
% NOTE: CSACS IS MECHANICALLY CONSTRAINED TO +/- 15 DEGREE TILT FROM THE
%       HORIZONTAL IN PITCH AND ROLL

% INPUT: desired euler angle attitude [deg] (theta and phi must be kept at
%        zero degrees in order to keep reaction wheels from saturating
psi_d   =  -90;   % rotation about z-axis (yaw)
theta_d =    0;   % rotation about y-axis (pitch)
phi_d   =    0;   % rotation about x-axis (roll)