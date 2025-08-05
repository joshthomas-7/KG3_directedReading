function [eul] = eulerZYX(R)
% Extract Euler angles from the rotation matrix (ZYX convention)
theta = -asin(R(3,1)); % Pitch
phi = atan2(R(3,2), R(3,3)); % Roll
psi = atan2(R(2,1), R(1,1)); % Yaw

eul = [theta;phi;psi];

end