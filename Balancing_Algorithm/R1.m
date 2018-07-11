% simple "1" rotation
function [R1_out] = R1(angle)
    R1_out = [1,          0,            0;
              0, cos(angle),   sin(angle);
              0, -sin(angle), cos(angle)];
end