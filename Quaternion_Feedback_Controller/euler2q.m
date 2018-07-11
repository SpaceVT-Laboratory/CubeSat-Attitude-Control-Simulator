%*********************************************************************
% euler2q.m
% Transforms Euler angles to quaternion parameters via a 3-2-1 sequence. 
% Meaning psi is the first rotation, theta is the second rotation, and phi
% is the third rotation 
%*********************************************************************
function qbar = euler2q(e)

k1   = 0.5*e(1);  % psi (yaw)
k2   = 0.5*e(2);  % theta (pitch)
k3   = 0.5*e(3);  % phi (roll)

cy   = cos(k1);
sy   = sin(k1);
cp   = cos(k2);
sp   = sin(k2);
cr   = cos(k3);
sr   = sin(k3);

q0   = cy * cr * cp + sy * sr * sp;
q1   = cy * sr * cp - sy * cr * sp;
q2   = cy * cr * sp + sy * sr * cp;
q3   = sy * cr * cp - cy * sr * sp;

qbar = [q0; q1; q2; q3];

end