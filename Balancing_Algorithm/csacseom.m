function ydot = csacseom(t, y)
% csacseom: CSACS, calculate the time derivatives of y, using the full EoM
% y = [psi,theta,phi,wx,wy,wz,rx,ry,rz,rm_x,rm_y]

% Used by CSAC_Sim

% CSACS physical properties
I     = [0.07916853     0.00944042    0.00214993;
         0.00944042     0.07766567   -0.00161408;
         0.00214993    -0.00161408    0.13642287]; % inertia matrix about center of gravity [kg*m^2]
m     = 7.75345629;                                % total mass of table [kg]
m_mmu = 0.043;                                     % mass of stepper motors [kg]
g = 9.81;                                          % acceleration due to gravity [m/s^2]


% aerodynamic damping coeff
b = [0.1; 0.1; 0.1]; 

rx = y(7); ry = y(8); rz = y(9);
wx = y(4); wy = y(5); wz = y(6);

cphi = cos(y(3));
sphi = sin(y(3));
cth  = cos(y(2));
sth  = sin(y(2));
cpsi = cos(y(1));
spsi = sin(y(1));

%% EOM terms

% A matrix
A=[(m*ry^2+m*rz^2+I(1,1)) (-m*rx*ry+I(1,2))      (-m*rx*rz+I(1,3));
   (-m*rx*ry+I(1,2))      (m*rx^2+m*rz^2+I(2,2)) (-m*ry*rz+I(2,3));
   (-m*rx*rz+I(1,3))      (-m*ry*rz+I(2,3))      (m*rx^2+m*ry^2+I(3,3))];

% B matrix
Bx = (-2*m*ry*rz+I(2,3))*wy^2+(2*m*ry*rz-I(2,3))*wz^2 ...
    +(-m*rx*rz+I(1,3))*wx*wy+(m*rx*ry-I(1,2))*wx*wz ...
    +(m*ry^2-m*rz^2-I(2,2)+I(3,3))*wy*wz;
By = (2*m*rx*rz-I(1,3))*wx^2+(-2*m*rx*rz+I(1,3))*wz^2 ...
    +(m*ry*rz-I(2,3))*wx*wy+(-m*rx^2-m*rz^2+I(1,1)-I(3,3))*wx*wz ...
    +(-m*rx*ry+I(1,2))*wy*wz;
Bz = (-2*m*rx*ry+I(1,2))*wx^2+(2*m*rx*ry-I(1,2))*wy^2 ...
    +(m*rx^2-m*ry^2-I(1,1)+I(2,2))*wx*wy+(-m*ry*rz+I(2,3))*wx*wz ...
    +(-m*rx*rz-I(1,3))*wy*wz;
B  = [Bx; By; Bz];

%% Torques

% gravitational torque
M1 = m*g*[(-ry*cth*cphi+rz*sphi*cth);
          (rz*sth+rx*cphi*cth);
          (-rx*sphi*cth-ry*sth)];

% aerodynamic torque
M2 = -[b(1)*wx^2; b(2)*wy^2; b(3)*wz^2];

% MMU "restoring torque"
% two MMUs
M3 = m_mmu*g*[-y(11)*cth*cphi; y(10)*cphi*cth; 0];

% three MMUs
% M3 = m_mmu*g*[(-y(11)*cth*cphi+y(12)*sphi*cth);
%               (y(12)*sth+y(10)*cphi*cth);
%               (-y(10)*sphi*cth-y(11)*sth)];        

% total torque
M = M1 + M2 + M3;

%% Derivatives

% psi
ydot(1) = sphi/cth*wy+cphi/cth*wz;
% theta
ydot(2) = cphi*wy-sphi*wz;
% phi
ydot(3) = wx+sth*sphi/cth*wy+sth*cphi/cth*wz;

% omegas
ydot(4:6) = pinv(A)*(M-B);

% CM offset
ydot(7) = 0;
ydot(8) = 0;
ydot(9) = 0;

% rm_x
ydot(10) = 0;
% rm_y
ydot(11) = 0;
% rm_z 
% ydot(12) = 0;

ydot = ydot';