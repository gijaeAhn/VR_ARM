syms theta1(t) theta2(t) theta3(t)


% Define rotation matrices for each joint, assuming rotations about the z-axis
T1 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 0; 0 0 0 1];
T2 = [cos(theta2) -sin(theta2) 0 0; sin(theta2) cos(theta2) 0 0; 0 0 1 0; 0 0 0 1];
T3 = [cos(theta3) -sin(theta3) 0 0; sin(theta3) cos(theta3) 0 0; 0 0 1 0; 0 0 0 1];

% Optionally, include translation components if applicable


T = T1 * T2 * T3;  % Modify as necessary based on actual kinematic chain

dT = diff(T, t);
syms dtheta1(t) dtheta2(t) dtheta3(t)
dT = subs(dT, diff(theta1(t), t), dtheta1);
dT = subs(dT, diff(theta2(t), t), dtheta2);
dT = subs(dT, diff(theta3(t), t), dtheta3);
simplify(dT)

Tinv = inv(T);

Twist = Tinv * dT;


% Assuming dT is already expressed in terms of dtheta1, dtheta2, dtheta3
J = [diff(Twist, dtheta1) diff(Twist, dtheta2) diff(Twist, dtheta3)]
