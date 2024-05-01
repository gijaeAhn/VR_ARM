function [R_ret, p_ret] = TransToRp(T)
    R_ret = T(1:3, 1:3); % Extract the top-left 3x3 submatrix for rotation
    p_ret = T(1:3, 4);   % Extract the position vector from the fourth column
end
