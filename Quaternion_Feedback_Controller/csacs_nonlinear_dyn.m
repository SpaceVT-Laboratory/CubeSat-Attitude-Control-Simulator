%*********************************************************************
% csacs_nonlinear_dyn.m
% A file to calculate the nonlinear dynamic equations of the CSACS, 
% coupled with the RWA
%*********************************************************************
function xdot = csacs_nonlinear_dyn(input,r_offset,I,J,T,K)

%*********************************************************************
% Reassign Input Variables
%*********************************************************************
% Attitude quaternions
qbar_temp = input(1:4);
qbar      = [qbar_temp(1) qbar_temp(2) qbar_temp(3) qbar_temp(4)]';

% Body rates
w_temp    = input(5:7);
wb        = [w_temp(1) w_temp(2) w_temp(3)]';

% Control torques
u_temp    = input(8:11);
u         = [u_temp(1) u_temp(2) u_temp(3) u_temp(4)]';

% Wheel speeds
ww_temp   = input(12:15);
ww        = [ww_temp(1) ww_temp(2) ww_temp(3) ww_temp(4)]';

% Normalizing the quaternions - necessary for numerical accuracy
qbar      = qbar/(qbar'*qbar);
q0        = qbar(1);
q         = qbar(2:4);

%*********************************************************************
% Reaction Wheel Torque and Angular Momentum Redistribution
%*********************************************************************
Tc = T'*K*u;
hw = J*(T'*K*ww);

%*********************************************************************
% Gravitational Torque Contribution
%*********************************************************************
% CM offset vector
r_temp = r_offset(1:3);
r      = [r_temp(1) r_temp(2) r_temp(3)]';

m      = 7.75345629;        % total mass of the CSACS [kg]
g      = 9.81;              % acceleration due to gravity [m/s^2]

% Gravitational torque from when the CSACS is not centered on the CR (NOTE:
% both methods produce identical results, i.e., using euler angles vs.
% using quaternions -- comment out one method, and use the other)

% using euler angles:
% [~,theta,phi] = q2euler(qbar);
% Tg = m*g*[-r(2)*cos(theta)*cos(phi) + r(3)*sin(phi)*cos(theta);
%           r(3)*sin(theta)           + r(1)*cos(phi)*cos(theta);
%           -r(1)*sin(phi)*cos(theta) -          r(2)*sin(theta)];

% using quaternions:
R = Rquat(qbar);
Tg = -m*g*skew([r(1); r(2); r(3)])*R*[0; 0; 1];
      
%*********************************************************************
% Aerodynamic Torque Contribution
%*********************************************************************
% aerodynamic damping coeff (neglect for control model)
b     = [0; 0; 0]; 
Taero = -[b(1)*wb(1)^2; b(2)*wb(2)^2; b(3)*wb(3)^2];

%*********************************************************************
% Output Time Derivatives of X (from full, nonlinear equations of motion)
%*********************************************************************
% Kinematics
q0_dot = -(1/2)*q'*wb;
q_dot  = (1/2)*(q0*eye(3) + skew(q))*wb;

% Attitude dynamics of the CSACS
wb_dot = inv(I)*(Tc + Tg + Taero - skew(wb)*(I*wb + hw));

xdot = [q0_dot; q_dot; wb_dot; u];