function [quat] = unitQuat(R)

q0 = sqrt(1 + R(1,1) + R(2,2) + R(3,3)) / 2; % Scalar part
q1 = (R(3,2) - R(2,3)) / (4 * q0); % First component
q2 = (R(1,3) - R(3,1)) / (4 * q0); % Second component
q3 = (R(2,1) - R(1,2)) / (4 * q0); % Third component

% Normalize the quaternion to ensure it is a unit quaternion
norm_q = sqrt(q0^2 + q1^2 + q2^2 + q3^2);
quat = [q0;q1;q2;q3]/norm_q;

end