%*********************************************************************
% q2euler.m
% Computes the Euler angles from the unit quaternions qbar = [q0 q1 q2 q3]'
% via a 3-2-1 sequence, meaning psi is the first rotation, theta is the 
% second rotation, and phi is the third rotation 
%*********************************************************************
function [psi,theta,phi] = q2euler(qbar)

q0  = qbar(1);
q1  = qbar(2);
q2  = qbar(3);
q3  = qbar(4);
 
R = Rquat(qbar);
if abs(R(3,1)) > 1.0
    error('solution is singular for theta = +- 90 degrees'); 
end

% Expanded terms (same as below)
% psi   = atan2(2*(q0*q3 + q1*q2), (q0^2 + q1^2 - q2^2 - q3^2));
% theta = asin(2*(q0*q2 - q3*q1));
% phi   = atan2(2*(q0*q1 + q2*q3), (q0^2 - q1^2 - q2^2 + q3^2));

psi   = atan2(R(1,2),R(1,1));
theta = asin(-R(1,3));
phi   = atan2(R(2,3),R(3,3));