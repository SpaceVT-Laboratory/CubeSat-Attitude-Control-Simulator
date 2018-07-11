function [r_hat] = r_est(y, t0, dt, tf, I)
% r_est: CSACS, estimates the vector to the CM of the CSACS
% from the center of rotation using the dynamic data

% Used by CSAC_Sim

% CSCACS Values
m = 7.75345629; % total mass of table [kg]
g = 9.81;       % acceleration due to gravity [m/s^2]

%% Method of Least Squares Algorithm to solve for r vector

Q1 = m*g*dt/(2*I(1,1));
Q2 = m*g*dt/(2*I(2,2));
Q3 = m*g*dt/(2*I(3,3));

cp = cos(y(3,:));
sp = sin(y(3,:));
ct = cos(y(2,:));
st = sin(y(2,:));

for t = t0:dt*3:tf*3
    tc = int32(t/dt+1);
    tp = int32((tc+2)/3);
    
    dw(tc)   = y(4,tp+1)-y(4,tp);
    dw(tc+1) = y(5,tp+1)-y(5,tp);
    dw(tc+2) = y(6,tp+1)-y(6,tp);
    
    D(:,tc)   = [0 -Q1*(cp(tp+1)*ct(tp+1)+cp(tp)*ct(tp)) Q1*(sp(tp+1)*ct(tp+1)+sp(tp)*ct(tp))]';
    D(:,tc+1) = [Q2*(cp(tp+1)*ct(tp+1)+cp(tp)*ct(tp)) 0 Q2*(st(tp+1)+st(tp))]';
    D(:,tc+2) = [-Q3*(sp(tp+1)*ct(tp+1)+sp(tp)*ct(tp)) -Q3*(st(tp+1)+st(tp)) 0]';
end

dw1 = dw';
D1 = D';
clear D;
clear dw;

r_hat = pinv(D1)*dw1;