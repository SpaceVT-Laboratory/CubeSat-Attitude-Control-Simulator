% simple "2" rotation
function [R2_out] = R2(angle)
    R2_out = [cos(angle), 0, -sin(angle);
                       0, 1,           0;
              sin(angle), 0, cos(angle)];
end