function M = rotateZ(inputM, theta)
    % Create the rotation matrix for rotation around the Z-axis
    rotate = [cos(theta), -sin(theta), 0, 0;
              sin(theta),  cos(theta), 0, 0;
              0,           0,          1, 0;
              0,           0,          0, 1];
          
    % Apply the rotation to the input matrix
    M = inputM * rotate;
end