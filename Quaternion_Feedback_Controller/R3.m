% simple "3" rotation
function [R3_out] = R3(angle)
    R3_out = [cos(angle), sin(angle),   0;
             -sin(angle), cos(angle),   0;
                       0,          0,  1];
end