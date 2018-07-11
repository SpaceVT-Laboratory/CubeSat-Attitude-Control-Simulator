%*********************************************************************
% Rquat.m
% Quaternion representation of the 3-2-1 rotation sequence, where psi is 
% the first rotation, theta is the second rotation, and phi is the third 
% rotation 
%*********************************************************************
function R = Rquat(qbar)

qn = bsxfun(@rdivide, [qbar(1) qbar(2) qbar(3) qbar(4)]',...
                sqrt(sum([qbar(1) qbar(2) qbar(3) qbar(4)].^2, 2)));
            
q0  = qn(1);
q1  = qn(2);
q2  = qn(3);
q3  = qn(4);

d11 = (q0^2 + q1^2 - q2^2 - q3^2);
d12 = 2*(q1*q2 + q0*q3);
d13 = 2*(q1*q3 - q0*q2);
d21 = 2*(q1*q2 - q0*q3);
d22 = (q0^2 - q1^2 + q2^2 - q3^2);
d23 = 2*(q2*q3 + q0*q1);
d31 = 2*(q1*q3 + q0*q2);
d32 = 2*(q2*q3 - q0*q1);
d33 = (q0^2 - q1^2 - q2^2 + q3^2);

R   = [d11 d12 d13;
       d21 d22 d23;
       d31 d32 d33];
   
end