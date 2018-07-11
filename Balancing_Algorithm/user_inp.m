function param = user_inp()
% user_inp: CSACS, Allows user input of initial conditions
% tf,psi,theta,phi,wx,wy,wz,rx,ry,rz

% Used by CSAC_Sim

%% Default Values

% simulation time
tf = 60;

% intitial angular positions [rad]
psi   = 0;  % yaw
theta = 0;  % pitch 
phi   = 0;  % roll

% initial angular velocities [rad/sec]
wx = 0;
wy = 0;
wz = 0;

% offset from CR to CG (determined from CAD model) [m]
rx =  0.00002454;
ry = -0.00013875;
rz = -0.00099764;

param = [tf, psi, theta, phi, wx, wy, wz, rx, ry, rz];