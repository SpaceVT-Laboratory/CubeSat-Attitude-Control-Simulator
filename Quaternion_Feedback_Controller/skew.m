%*********************************************************************
% skew.m
% Function that takes skew symmetric matrix of input
% (i denotes the input matrix)
%*********************************************************************
function S = skew(i)

S =[  0  -i(3)   i(2);
    i(3)    0   -i(1)
   -i(2)  i(1)     0];

end